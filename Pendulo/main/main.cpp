/**
 * @file main.cpp
 * @brief Sistema Híbrido de Control PID para Motor DC (Velocidad y Posición)
 * @details Incluye perfiles de velocidad, control de péndulo absoluto, Homing con IR, 
 *          monitoreo de corriente (ACS712) y FeedForward para seguimiento de trayectorias.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "esp_adc/adc_oneshot.h" 
#include "esp_log.h"
#include "esp_timer.h"

// ==============================================================================
// 1. DEFINICIONES DE HARDWARE Y PARÁMETROS FÍSICOS
// ==============================================================================
// Pines del Puente H y Motor
#define PIN_PWM             GPIO_NUM_18
#define PIN_IN1             GPIO_NUM_19
#define PIN_IN2             GPIO_NUM_21
#define PIN_ENC_A           GPIO_NUM_26
#define PIN_ENC_B           GPIO_NUM_27

// Sensores Externos
#define PIN_SENSOR_IR        GPIO_NUM_32 
#define PIN_SENSOR_CORRIENTE ADC_CHANNEL_6   
#define ACS_SENSIBILIDAD     0.185f           
#define ACS_VOLTAJE_CERO     2.0f 

// Parámetros de PWM y Control
#define PWM_FREQ_HZ         20000
#define PWM_RES             LEDC_TIMER_10_BIT
#define PWM_MAX             1023
#define INTERVALO_PID_MS    20
#define PULSOS_POR_REV_4X   937.0f 

// Parámetros de Curvas de Velocidad
#define MAX_RPM             330.0f
#define TIEMPO_TOTAL_CICLO  10.0f
#define TIEMPO_PAUSA        2.0f

// ==============================================================================
// 2. VARIABLES GLOBALES Y MÁQUINA DE ESTADOS
// ==============================================================================
typedef enum {
    ESTADO_DETENIDO,
    ESTADO_ESPERANDO_CURVA,
    ESTADO_CORRIENDO_CURVA,
    ESTADO_VELOCIDAD_FIJA,
    ESTADO_HOMING,        
    ESTADO_POSICION
} EstadoMotor_t;

typedef enum {
    CONSTANTE, RAMPA_LINEAL, TRAPEZOIDAL, SENOIDAL,
    ESCALONADA, EXPONENCIAL, GAUSSIANO, DIENTE_SIERRA, RUIDO_BLANCO
} PerfilVelocidad_t;

volatile EstadoMotor_t estado_motor = ESTADO_DETENIDO;
volatile PerfilVelocidad_t perfilSeleccionado = TRAPEZOIDAL;
volatile PerfilVelocidad_t perfilActual = TRAPEZOIDAL;

volatile int64_t tiempoCambio_us = 0;
volatile long posicion_global_pulsos = 0; 
volatile float grados_objetivo = 0.0f;
volatile float rpm_objetivo_fijo = 0.0f; 

// Variables de lectura y filtrado de sensores
float corrienteFiltrada = 0.0f;
float rpmFiltrada = 0.0f;
const float ALPHA_FILTRO = 0.3f;

// Instancias de periféricos
pcnt_unit_handle_t pcnt_unit = NULL;
adc_oneshot_unit_handle_t adc1_handle = NULL; 

// ==============================================================================
// 3. PARÁMETROS Y FUNCIONES DE LOS CONTROLADORES PID
// ==============================================================================

// --- Parámetros del PID de Velocidad ---
float Kp_vel = 2.0f;         // Ganancia Proporcional
float Ki_vel = 6.0f;         // Ganancia Integral
float Kd_vel = 0.02f;        // Ganancia Derivativa
float Kf_vel = 2.5f;         // FeedForward (Predicción para seguimiento de curvas)
float PWM_ARRANQUE = 250.0f; // Compensación de fricción estática inicial

float errorIntegral_vel = 0.0f;
float rpmPrevia = 0.0f;

// --- Parámetros del PID de Posición ---
float Kp_pos = 5.0f;
float Kd_pos = 0.5f;  
long errorPrevio_pos = 0;

/**
 * @brief Calcula el esfuerzo PWM necesario para alcanzar una velocidad (RPM)
 */
float calcular_PID_velocidad(float objetivo_rpm, float actual_rpm, float dt) {
    float error = objetivo_rpm - fabsf(actual_rpm);
    
    // Cálculo PID + FeedForward
    float P = Kp_vel * error;
    float D = -Kd_vel * ((fabsf(actual_rpm) - fabsf(rpmPrevia)) / dt);
    float FF = Kf_vel * objetivo_rpm; 
    
    rpmPrevia = actual_rpm;
    float salida = P + (Ki_vel * errorIntegral_vel) + D + FF;
    
    // Impulso inicial para vencer la inercia desde el reposo
    if (objetivo_rpm > 0.0f && fabsf(actual_rpm) < 5.0f && salida > 0.0f) {
        salida += PWM_ARRANQUE; 
    }
    
    // Anti-Windup Integral y saturación de salida
    if (salida > PWM_MAX) { 
        salida = PWM_MAX; 
        if (error < 0) errorIntegral_vel += error * dt; 
    } 
    else if (salida < -PWM_MAX) { 
        salida = -PWM_MAX; 
        if (error > 0) errorIntegral_vel += error * dt; 
    } 
    else { 
        errorIntegral_vel += error * dt; 
    }
    
    return salida;
}

/**
 * @brief Calcula el esfuerzo PWM necesario para mantener una posición (Pulsos)
 */
float calcular_PID_posicion(long objetivo_pulsos, long actual_pulsos, float dt) {
    long error_pos = objetivo_pulsos - actual_pulsos;
    
    float P = Kp_pos * (float)error_pos;
    float D = Kd_pos * ((float)(error_pos - errorPrevio_pos) / dt);
    errorPrevio_pos = error_pos;
    
    float salida = P + D;
    
    // Saturación de salida
    if (salida > PWM_MAX) salida = PWM_MAX;
    if (salida < -PWM_MAX) salida = -PWM_MAX;
    
    return salida;
}

// ==============================================================================
// 4. GENERADOR DE CURVAS MATEMÁTICAS
// ==============================================================================
float evaluarPerfil(PerfilVelocidad_t perfil, float t, float t_final) {
    float p_max = 280.0f;
    switch (perfil) {
        case CONSTANTE:     return 200.0f;
        case RAMPA_LINEAL:  return (p_max / t_final) * t;
        case TRAPEZOIDAL: {
            float ta = t_final * 0.3f, tb = t_final * 0.7f;
            float pendiente = p_max / ta;
            if (t < ta) return pendiente * t;
            if (t < tb) return p_max;
            return p_max - pendiente * (t - tb);
        }
        case SENOIDAL:      return 150.0f + 100.0f * sinf(2.0f * M_PI * 0.5f * t);
        case ESCALONADA:    return (fmodf(t, 2.0f) < 1.0f) ? 50.0f : 250.0f;
        case EXPONENCIAL:   return p_max * powf(t / t_final, 2.0f);
        case GAUSSIANO:     return p_max * expf(-powf((t - (t_final/2.0f)) / (t_final/5.0f), 2.0f));
        case DIENTE_SIERRA: return p_max * fmodf(t / 2.5f, 1.0f);
        case RUIDO_BLANCO: {
            static float rpm_aleatorio = 150.0f;
            static int ultimo_paso = -1;
            
            // Cambia de velocidad objetivo cada 0.25 segundos (4 veces por segundo)
            int paso_actual = (int)(t * 4.0f); 
            if (paso_actual != ultimo_paso) {
                rpm_aleatorio = 50.0f + (float)(rand() % 200); // Rango entre 50 y 250 RPM
                ultimo_paso = paso_actual;
            }
            return rpm_aleatorio;
        }
        default:            return 0.0f;
    }
}

// ==============================================================================
// 5. CONFIGURACIÓN DE HARDWARE E INICIALIZACIÓN
// ==============================================================================
void init_hardware() {
    // 5.1 Pines de Dirección (Puente H) y Sensor IR
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_IN1) | (1ULL << PIN_IN2);
    gpio_config(&io_conf);
    gpio_set_level(PIN_IN1, 0); 
    gpio_set_level(PIN_IN2, 0);

    gpio_config_t ir_conf = {};
    ir_conf.intr_type = GPIO_INTR_DISABLE;
    ir_conf.mode = GPIO_MODE_INPUT;
    ir_conf.pin_bit_mask = (1ULL << PIN_SENSOR_IR);
    ir_conf.pull_up_en = GPIO_PULLUP_ENABLE; 
    gpio_config(&ir_conf);

    // 5.2 ADC (Sensor de Corriente ACS712)
    adc_oneshot_unit_init_cfg_t init_config = {};
    init_config.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {};
    config.atten = ADC_ATTEN_DB_12; 
    config.bitwidth = ADC_BITWIDTH_12; 
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, PIN_SENSOR_CORRIENTE, &config));

    // 5.3 LEDC (Generador de señal PWM)
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

    // 5.4 PCNT (Contador de Pulsos para Encoder en Cuadratura)
    pcnt_unit_config_t unit_config = {};
    unit_config.low_limit = -32000; 
    unit_config.high_limit = 32000;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    pcnt_glitch_filter_config_t filter_config = {};
    filter_config.max_glitch_ns = 1000;
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    pcnt_chan_config_t chan_a_config = {};
    chan_a_config.edge_gpio_num = PIN_ENC_A; 
    chan_a_config.level_gpio_num = PIN_ENC_B;
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));

    pcnt_chan_config_t chan_b_config = {};
    chan_b_config.edge_gpio_num = PIN_ENC_B; 
    chan_b_config.level_gpio_num = PIN_ENC_A;
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
    if (pwm_value > 0)      { gpio_set_level(PIN_IN1, 1); gpio_set_level(PIN_IN2, 0); } 
    else if (pwm_value < 0) { gpio_set_level(PIN_IN1, 0); gpio_set_level(PIN_IN2, 1); } 
    else                    { gpio_set_level(PIN_IN1, 0); gpio_set_level(PIN_IN2, 0); }
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, abs(pwm_value));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ==============================================================================
// 6. TAREAS DE FREERTOS (LAZO CERRADO DE CONTROL)
// ==============================================================================
void control_motor_task(void *pvParameter) {
    int64_t tiempoInicioCiclo_us = esp_timer_get_time();
    int64_t tiempoUltimoPID_us = esp_timer_get_time();

    while (1) {
        int64_t tiempoActual_us = esp_timer_get_time();
        float dt = (tiempoActual_us - tiempoUltimoPID_us) / 1000000.0f;
        
        if (dt >= (INTERVALO_PID_MS / 1000.0f)) {
            tiempoUltimoPID_us = tiempoActual_us;
            float salidaControl = 0.0f;
            
            // --- ACTUALIZACIÓN DE SENSORES ---
            int pulsos_delta = 0;
            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulsos_delta));
            ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit)); 
            posicion_global_pulsos += pulsos_delta; 
            
            float rpmCruda = ((float)pulsos_delta / PULSOS_POR_REV_4X) * (60.0f / dt);
            rpmFiltrada = (ALPHA_FILTRO * rpmCruda) + ((1.0f - ALPHA_FILTRO) * rpmFiltrada);

            int adcRaw = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, PIN_SENSOR_CORRIENTE, &adcRaw));
            float voltajeADC = ((float)adcRaw / 4095.0f) * 3.3f;
            float corrienteCruda = (voltajeADC - ACS_VOLTAJE_CERO) / ACS_SENSIBILIDAD;
            corrienteFiltrada = (0.1f * corrienteCruda) + (0.9f * corrienteFiltrada);

            // --- LÓGICA DE LA MÁQUINA DE ESTADOS ---
            switch (estado_motor) {
                
                case ESTADO_DETENIDO:
                    salidaControl = 0.0f;
                    errorIntegral_vel = 0.0f;
                    break;

                case ESTADO_ESPERANDO_CURVA:
                    salidaControl = 0.0f;
                    if ((tiempoActual_us - tiempoCambio_us) >= 3000000LL) { 
                        estado_motor = ESTADO_CORRIENDO_CURVA;
                        perfilActual = perfilSeleccionado;
                        tiempoInicioCiclo_us = tiempoActual_us;
                        errorIntegral_vel = 0.0f; 
                        printf(">Info: ¡Arrancando curva de velocidad!\n");
                    }
                    break;

                case ESTADO_CORRIENDO_CURVA: {
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
                    
                    salidaControl = calcular_PID_velocidad(rpmObjetivo, rpmFiltrada, dt);
                    
                    printf(">RPM_Obj:%.2f\n", rpmObjetivo);
                    printf(">RPM_Real:%.2f\n", fabsf(rpmFiltrada));
                    break;
                }
                
                case ESTADO_VELOCIDAD_FIJA:
                    salidaControl = calcular_PID_velocidad(rpm_objetivo_fijo, rpmFiltrada, dt);
                    
                    printf(">RPM_Obj:%.2f\n", rpm_objetivo_fijo);
                    printf(">RPM_Real:%.2f\n", fabsf(rpmFiltrada));
                    break;

                case ESTADO_HOMING:
                    if (gpio_get_level(PIN_SENSOR_IR) == 1) { 
                        salidaControl = 600.0f; // Potencia de búsqueda hacia el sensor
                    } else {
                        posicion_global_pulsos = 0; 
                        salidaControl = 0.0f;
                        grados_objetivo = 0.0f; 
                        errorPrevio_pos = 0; 
                        estado_motor = ESTADO_POSICION;
                        printf(">Info: ¡Homing Completo! Centro establecido (0 grados).\n");
                    }
                    break;

                case ESTADO_POSICION: {
                    long pulsos_objetivo = (long)((grados_objetivo / 360.0f) * PULSOS_POR_REV_4X);
                    salidaControl = calcular_PID_posicion(pulsos_objetivo, posicion_global_pulsos, dt);
                    
                    float grados_actuales = ((float)posicion_global_pulsos / PULSOS_POR_REV_4X) * 360.0f;
                    printf(">Grados_Obj:%.2f\n", grados_objetivo);
                    printf(">Grados_Real:%.2f\n", grados_actuales);
                    break;
                }
            }

            aplicarMotorPWM((int)salidaControl);
            
            printf(">PWM_Salida:%.2f\n", salidaControl);
            printf(">Corriente:%.2f\n", corrienteFiltrada); 
        }
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

// ==============================================================================
// 7. TAREA DE LECTURA SERIAL (INTERFAZ DE CONSOLA)
// ==============================================================================
void lectura_serial_task(void *pvParameter) {
    char linea[32];
    printf("\n>Info: === SISTEMA HIBRIDO COMPLETO ===\n");
    printf(">Info: [1-9]  Ejecutar curva de Velocidad\n");
    printf(">Info: [v150] Velocidad Fija en RPM (ej. v150)\n"); 
    printf(">Info: [g1.5] Velocidad en Giros/segundo (ej. g2)\n"); 
    printf(">Info: [h]    Ejecutar Homing (Busca centro IR)\n");
    printf(">Info: [p45]  Ir a Posicion 45 grados (Usa p+Numero)\n");
    printf(">Info: [s]    Detener emergencia\n\n");

    while (1) {
        if (fgets(linea, sizeof(linea), stdin) != NULL) {
            char comando = linea[0];
            
            if (comando >= '1' && comando <= '9') {
                perfilSeleccionado = (PerfilVelocidad_t)(comando - '1');
                tiempoCambio_us = esp_timer_get_time();
                estado_motor = ESTADO_ESPERANDO_CURVA;
                printf(">Info: Curva %c solicitada. Arrancando en 3s...\n", comando);
            }
            else if (comando == 'v' || comando == 'V') {
                float nuevas_rpm = 0;
                if (sscanf(linea + 1, "%f", &nuevas_rpm) == 1) {
                    rpm_objetivo_fijo = nuevas_rpm;
                    errorIntegral_vel = 0.0f; 
                    estado_motor = ESTADO_VELOCIDAD_FIJA;
                    printf(">Info: Motor a velocidad constante de %.2f RPM.\n", rpm_objetivo_fijo);
                } else { printf(">Info: Formato incorrecto. Ejemplo: v150\n"); }
            }
            else if (comando == 'g' || comando == 'G') {
                float nuevos_giros = 0;
                if (sscanf(linea + 1, "%f", &nuevos_giros) == 1) {
                    rpm_objetivo_fijo = nuevos_giros * 60.0f; 
                    errorIntegral_vel = 0.0f; 
                    estado_motor = ESTADO_VELOCIDAD_FIJA;
                    printf(">Info: Motor a %.2f giros/s (Equivale a %.2f RPM).\n", nuevos_giros, rpm_objetivo_fijo);
                } else { printf(">Info: Formato incorrecto. Ejemplo: g1.5\n"); }
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
                } else { printf(">Info: Formato incorrecto. Ejemplo: p90\n"); }
            }
            else if (comando == 's' || comando == 'S' || comando == '0') {
                estado_motor = ESTADO_DETENIDO;
                printf(">Info: --- MOTOR DETENIDO DE EMERGENCIA ---\n");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==============================================================================
// 8. FUNCIÓN PRINCIPAL (PUNTO DE ENTRADA)
// ==============================================================================
extern "C" void app_main(void) {
    init_hardware();
    xTaskCreatePinnedToCore(control_motor_task, "motor_task", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(lectura_serial_task, "serial_task", 3072, NULL, 2, NULL, 0);
}