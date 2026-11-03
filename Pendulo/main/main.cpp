#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_adc/adc_oneshot.h" // NUEVA librería ADC para ESP-IDF v5+
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "MOTOR_CONTROL";

// ==========================================
// CONFIGURACIÓN DE PINES
// ==========================================
#define PIN_PWM      GPIO_NUM_18
#define PIN_IN1      GPIO_NUM_19
#define PIN_IN2      GPIO_NUM_21
#define PIN_ENC_A    GPIO_NUM_26
#define PIN_ENC_B    GPIO_NUM_27
#define PIN_SENSOR_IR GPIO_NUM_32 // Pin del sensor infrarrojo

// Configuración del ACS712
#define PIN_SENSOR_CORRIENTE ADC_CHANNEL_6   // Equivale al GPIO 34 en la nueva API
#define ACS_SENSIBILIDAD    0.185f           // Sensibilidad modelo 5A (0.185 V/A)
#define ACS_VOLTAJE_CERO    2.335f             // Voltaje con 0 Amperios (Mitad de 5V)

#define PWM_FREQ_HZ  20000
#define PWM_RES      LEDC_TIMER_10_BIT
#define PWM_MAX      1023

#define PULSOS_POR_REV_4X 937.0f // Valor calibrado de la caja reductora
#define MAX_RPM           330.0f
#define TIEMPO_TOTAL_CICLO 10.0f
#define TIEMPO_PAUSA       2.0f
#define INTERVALO_PID_MS   20

// ==========================================
// MÁQUINA DE ESTADOS Y VARIABLES GLOBALES
// ==========================================
typedef enum {
    ESTADO_DETENIDO,
    ESTADO_ESPERANDO_CURVA,
    ESTADO_CORRIENDO_CURVA,
    ESTADO_HOMING,        // Buscando el sensor IR
    ESTADO_POSICION       // Manteniendo posición (grados)
} EstadoMotor_t;

volatile EstadoMotor_t estado_motor = ESTADO_DETENIDO;
volatile int64_t tiempoCambio_us = 0;

typedef enum {
    CONSTANTE, RAMPA_LINEAL, TRAPEZOIDAL, SENOIDAL,
    ESCALONADA, EXPONENCIAL, GAUSSIANO, DIENTE_SIERRA, RUIDO_BLANCO
} PerfilVelocidad_t;

volatile PerfilVelocidad_t perfilSeleccionado = TRAPEZOIDAL;
volatile PerfilVelocidad_t perfilActual = TRAPEZOIDAL;

volatile long posicion_global_pulsos = 0; 
volatile float grados_objetivo = 0.0f;

float corrienteFiltrada = 0.0f;

float Kp_vel = 1.5f;
float Ki_vel = 4.0f;
float Kd_vel = 0.02f;
float errorIntegral_vel = 0;
float rpmFiltrada = 0;
float rpmPrevia = 0;
const float ALPHA_FILTRO = 0.3f;

float Kp_pos = 5.0f;  
float Kd_pos = 0.5f;  
float errorPrevio_pos = 0;

pcnt_unit_handle_t pcnt_unit = NULL;
adc_oneshot_unit_handle_t adc1_handle = NULL; // Manejador global del ADC

// ==========================================
// PERFILES MATEMÁTICOS DE VELOCIDAD
// ==========================================
float evaluarPerfil(PerfilVelocidad_t perfil, float t, float t_final) {
    float p0 = 0.0f;
    float p_max = 280.0f;
    switch (perfil) {
        case CONSTANTE:     return 200.0f;
        case RAMPA_LINEAL:  return p0 + ((p_max - p0) / t_final) * t;
        case TRAPEZOIDAL: {
            float ta = t_final * 0.3f, tb = t_final * 0.7f;
            float a = p_max / ta;
            if (t < ta) return a * t;
            if (t < tb) return p_max;
            return p_max - a * (t - tb);
        }
        case SENOIDAL:      return 150.0f + 100.0f * sinf(2.0f * M_PI * 0.5f * t);
        case ESCALONADA:    return (fmodf(t, 2.0f) < 1.0f) ? 50.0f : 250.0f;
        case EXPONENCIAL:   return p_max * powf(t / t_final, 2.0f);
        case GAUSSIANO:     return p_max * expf(-powf((t - (t_final/2.0f)) / (t_final/5.0f), 2.0f));
        case DIENTE_SIERRA: return p_max * fmodf(t / 2.5f, 1.0f);
        case RUIDO_BLANCO:  return 180.0f;
        default:            return 0.0f;
    }
}

// ==========================================
// HARDWARE Y PWM
// ==========================================
void init_hardware() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_IN1) | (1ULL << PIN_IN2);
    gpio_config(&io_conf);
    gpio_set_level(PIN_IN1, 0); gpio_set_level(PIN_IN2, 0);

    gpio_config_t ir_conf = {};
    ir_conf.intr_type = GPIO_INTR_DISABLE;
    ir_conf.mode = GPIO_MODE_INPUT;
    ir_conf.pin_bit_mask = (1ULL << PIN_SENSOR_IR);
    ir_conf.pull_up_en = GPIO_PULLUP_ENABLE; 
    gpio_config(&ir_conf);

    // NUEVA INICIALIZACIÓN DEL ADC (ESP-IDF v5+)
    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {};
    config.atten = ADC_ATTEN_DB_12; // Rango hasta ~3.3V
    config.bitwidth = ADC_BITWIDTH_12; // Resolución 0-4095
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PIN_SENSOR_CORRIENTE, &config));

    ledc_timer_config_t ledc_timer = {};
    ledc_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_timer.duty_resolution = PWM_RES;
    ledc_timer.timer_num = LEDC_TIMER_0;
    ledc_timer.freq_hz = PWM_FREQ_HZ;
    ledc_timer.clk_cfg = LEDC_AUTO_CLK;
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {};
    ledc_channel.gpio_num = PIN_PWM;
    ledc_channel.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_channel.channel = LEDC_CHANNEL_0;
    ledc_channel.timer_sel = LEDC_TIMER_0;
    ledc_channel_config(&ledc_channel);

    pcnt_unit_config_t unit_config = {};
    unit_config.low_limit = -32000; unit_config.high_limit = 32000;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {};
    filter_config.max_glitch_ns = 1000;
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {};
    chan_a_config.edge_gpio_num = PIN_ENC_A; chan_a_config.level_gpio_num = PIN_ENC_B;
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {};
    chan_b_config.edge_gpio_num = PIN_ENC_B; chan_b_config.level_gpio_num = PIN_ENC_A;
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

void aplicarMotorPWM(int pwm_value) {
    if (pwm_value > 0) {
        gpio_set_level(PIN_IN1, 1); gpio_set_level(PIN_IN2, 0);
    } else if (pwm_value < 0) {
        gpio_set_level(PIN_IN1, 0); gpio_set_level(PIN_IN2, 1);
    } else {
        gpio_set_level(PIN_IN1, 0); gpio_set_level(PIN_IN2, 0);
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(pwm_value));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ==========================================
// TAREA 1: CONTROL PID (Maneja el Motor)
// ==========================================
void control_motor_task(void *pvParameter) {
    int64_t tiempoInicioCiclo_us = esp_timer_get_time();
    int64_t tiempoUltimoPID_us = esp_timer_get_time();

    while (1) {
        int64_t tiempoActual_us = esp_timer_get_time();
        float dt = (tiempoActual_us - tiempoUltimoPID_us) / 1000000.0f;
        
        if (dt >= (INTERVALO_PID_MS / 1000.0f)) {
            tiempoUltimoPID_us = tiempoActual_us;
            
            int pulsos_delta = 0;
            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulsos_delta));
            ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit)); 
            posicion_global_pulsos += pulsos_delta; 
            
            float rpmCruda = ((float)pulsos_delta / PULSOS_POR_REV_4X) * (60.0f / dt);
            rpmFiltrada = (ALPHA_FILTRO * rpmCruda) + ((1.0f - ALPHA_FILTRO) * rpmFiltrada);
            float salidaControl = 0.0f;

            // =======================================================
            // LECTURA DEL SENSOR DE CORRIENTE ACS712 (NUEVA API)
            // =======================================================
            int adcRaw = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PIN_SENSOR_CORRIENTE, &adcRaw));
            float voltajeADC = ((float)adcRaw / 4095.0f) * 3.3f;
            float corrienteCruda = (voltajeADC - ACS_VOLTAJE_CERO) / ACS_SENSIBILIDAD;
            corrienteFiltrada = (0.1f * corrienteCruda) + (0.9f * corrienteFiltrada);
            // =======================================================

            if (estado_motor == ESTADO_DETENIDO) {
                salidaControl = 0.0f;
                errorIntegral_vel = 0;
            } 
            else if (estado_motor == ESTADO_ESPERANDO_CURVA) {
                salidaControl = 0.0f;
                if ((tiempoActual_us - tiempoCambio_us) >= 3000000LL) {
                    estado_motor = ESTADO_CORRIENDO_CURVA;
                    perfilActual = perfilSeleccionado;
                    tiempoInicioCiclo_us = tiempoActual_us;
                    errorIntegral_vel = 0.0f; 
                    printf(">Info: ¡Arrancando curva de velocidad!\n");
                }
            } 
            else if (estado_motor == ESTADO_CORRIENDO_CURVA) {
                float t_ciclo = (tiempoActual_us - tiempoInicioCiclo_us) / 1000000.0f;
                float rpmObjetivo = 0.0f;
                
                if (t_ciclo <= TIEMPO_TOTAL_CICLO) {
                    rpmObjetivo = evaluarPerfil(perfilActual, t_ciclo, TIEMPO_TOTAL_CICLO);
                } else if (t_ciclo <= (TIEMPO_TOTAL_CICLO + TIEMPO_PAUSA)) {
                    rpmObjetivo = 0.0f;
                } else {
                    tiempoInicioCiclo_us = esp_timer_get_time();
                    errorIntegral_vel = 0.0f; 
                }
                
                float error = rpmObjetivo - fabsf(rpmFiltrada);
                float P = Kp_vel * error;
                float D = -Kd_vel * ((fabsf(rpmFiltrada) - fabsf(rpmPrevia)) / dt);
                rpmPrevia = rpmFiltrada;
                
                salidaControl = P + (Ki_vel * errorIntegral_vel) + D;
                
                float PWM_MINIMO = 120.0f; 
                if (rpmObjetivo > 0.0f && fabsf(rpmFiltrada) < 5.0f && salidaControl > 0.0f) salidaControl += PWM_MINIMO;
                if (salidaControl > PWM_MAX) { salidaControl = PWM_MAX; if (error < 0) errorIntegral_vel += error * dt; } 
                else if (salidaControl < -PWM_MAX) { salidaControl = -PWM_MAX; if (error > 0) errorIntegral_vel += error * dt; } 
                else { errorIntegral_vel += error * dt; }
                
                printf(">RPM_Obj:%.2f\n", rpmObjetivo);
                printf(">RPM_Real:%.2f\n", fabsf(rpmFiltrada));
            }
            else if (estado_motor == ESTADO_HOMING) {
                if (gpio_get_level(PIN_SENSOR_IR) == 1) { 
                    salidaControl = 600.0f; 
                } else {
                    posicion_global_pulsos = 0; 
                    salidaControl = 0.0f;
                    estado_motor = ESTADO_POSICION;
                    grados_objetivo = 0.0f; 
                    errorPrevio_pos = 0.0f; 
                    printf(">Info: ¡Homing Completo! Centro establecido (0 grados).\n");
                }
            }
            else if (estado_motor == ESTADO_POSICION) {
                long pulsos_objetivo = (long)((grados_objetivo / 360.0f) * PULSOS_POR_REV_4X);
                long error_pos = pulsos_objetivo - posicion_global_pulsos;
                
                float P = Kp_pos * (float)error_pos;
                float D = Kd_pos * ((float)(error_pos - errorPrevio_pos) / dt);
                errorPrevio_pos = error_pos;
                
                salidaControl = P + D;
                
                if (salidaControl > PWM_MAX) salidaControl = PWM_MAX;
                if (salidaControl < -PWM_MAX) salidaControl = -PWM_MAX;

                float grados_actuales = ((float)posicion_global_pulsos / PULSOS_POR_REV_4X) * 360.0f;
                printf(">Grados_Obj:%.2f\n", grados_objetivo);
                printf(">Grados_Real:%.2f\n", grados_actuales);
            }

            aplicarMotorPWM((int)salidaControl);
            
            printf(">PWM_Salida:%.2f\n", salidaControl);
            printf(">Corriente:%.2f\n", corrienteFiltrada); 
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ==========================================
// TAREA 2: LECTURA SERIAL (Maneja el Menú)
// ==========================================
void lectura_serial_task(void *pvParameter) {
    char linea[32];
    printf("\n>Info: === SISTEMA PRINCIPAL (CON SENSOR CORRIENTE ACS712) ===\n");
    printf(">Info: [1-9] Ejecutar curva de Velocidad\n");
    printf(">Info: [h]   Ejecutar Homing (Busca centro IR)\n");
    printf(">Info: [p45] Ir a Posicion 45 grados (Usa p+Numero)\n");
    printf(">Info: [s]   Detener emergencia\n\n");

    while (1) {
        if (fgets(linea, sizeof(linea), stdin) != NULL) {
            char comando = linea[0];
            
            if (comando >= '1' && comando <= '9') {
                perfilSeleccionado = (PerfilVelocidad_t)(comando - '1');
                tiempoCambio_us = esp_timer_get_time();
                estado_motor = ESTADO_ESPERANDO_CURVA;
                printf(">Info: Curva %c solicitada. Arrancando en 3s...\n", comando);
            } 
            else if (comando == 'h' || comando == 'H') {
                estado_motor = ESTADO_HOMING;
                printf(">Info: Iniciando Homing...\n");
            }
            else if (comando == 'p' || comando == 'P') {
                float nuevos_grados = 0;
                if (sscanf(linea + 1, "%f", &nuevos_grados) == 1) {
                    grados_objetivo = nuevos_grados;
                    estado_motor = ESTADO_POSICION;
                    printf(">Info: Moviendo pendulo a %.2f grados.\n", grados_objetivo);
                } else {
                    printf(">Info: Formato incorrecto. Ejemplo: p90\n");
                }
            }
            else if (comando == 's' || comando == 'S' || comando == '0') {
                estado_motor = ESTADO_DETENIDO;
                printf(">Info: --- MOTOR DETENIDO DE EMERGENCIA ---\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

extern "C" void app_main(void) {
    init_hardware();
    xTaskCreatePinnedToCore(control_motor_task, "motor_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(lectura_serial_task, "serial_task", 3072, NULL, 2, NULL, 0);
}