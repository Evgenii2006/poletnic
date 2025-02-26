#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "hal/ledc_types.h"
#include <math.h>

// Пины
#define MOTOR_PIN GPIO_NUM_13    // PWM для ESC
#define ELEVON1_PIN GPIO_NUM_14  // PWM для первого элевона
#define ELEVON2_PIN GPIO_NUM_15  // PWM для второго элевона
#define UART_CAM UART_NUM_1      // UART для камеры
#define UART_VTX UART_NUM_2      // UART для видеопередатчика
#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22

// Параметры IMU (ICM-42688-P)
#define I2C_PORT I2C_NUM_0
#define IMU_ADDR 0x68

// Глобальные переменные
volatile float roll = 0.0, pitch = 0.0;
uint8_t video_buffer[1024]; // Буфер для кадра

// Инициализация PWM
void init_pwm() {
    // Настройка таймера
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // разрешение 13 бит
        .freq_hz = 50,                        // частота 50 Гц
        .speed_mode = LEDC_LOW_SPEED_MODE,   // высокоскоростной режим
        .timer_num = LEDC_TIMER_0,            // таймер 0
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Настройка каналов
    ledc_channel_config_t ledc_channel = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = 4096, // 50% duty cycle (1000 мкс)
        .gpio_num   = MOTOR_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel);

    ledc_channel.channel = LEDC_CHANNEL_1;
    ledc_channel.gpio_num = ELEVON1_PIN;
    ledc_channel_config(&ledc_channel);

    ledc_channel.channel = LEDC_CHANNEL_2;
    ledc_channel.gpio_num = ELEVON2_PIN;
    ledc_channel_config(&ledc_channel);
}

void set_pwm_duty(int channel, uint32_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void i2c_master_init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint32_t clk_speed) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = clk_speed
    };
    i2c_param_config(i2c_num, &conf);
    i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);
}

// Чтение данных с IMU (упрощено)
void read_imu() {
    uint8_t data[6];
    i2c_master_read_from_device(I2C_PORT, IMU_ADDR, data, 6, 100);
    int16_t accel_x = (data[0] << 8) | data[1];
    int16_t accel_y = (data[2] << 8) | data[3];
    int16_t accel_z = (data[4] << 8) | data[5];
    // Вычисление крена и тангажа (упрощенно)
    pitch = atan2(accel_x, sqrt(accel_y * accel_y + accel_z * accel_z)) * 180 / M_PI;
    roll = atan2(accel_y, accel_z) * 180 / M_PI;
}

float pid_control(float setpoint, float measured) {
    static float error_prev = 0, integral = 0;
    float kp = 1.0, ki = 0.1, kd = 0.05;
    float error = setpoint - measured;
    integral += error;
    float derivative = error - error_prev;
    float output;

    output = (kp * error) + (ki * integral) + (kd * derivative);

    error_prev = error;
    return output;
}

// Управление полетом (Ядро 0)
void flight_task(void *pvParameters) {
    while (1) {
        read_imu();
        float throttle = 1500; // Пример команды с пульта (будет от Crossfire)
        float roll_setpoint = 0.0, pitch_setpoint = 10.0; // Задание от пульта
        float roll_output = pid_control(roll_setpoint, roll);
        float pitch_output = pid_control(pitch_setpoint, pitch);

        // Микширование для элевонов
        set_pwm_duty(LEDC_CHANNEL_0, throttle);
        set_pwm_duty(LEDC_CHANNEL_1, 1500 + roll_output + pitch_output);
        set_pwm_duty(LEDC_CHANNEL_2, 1500 - roll_output + pitch_output);
        vTaskDelay(10 / portTICK_PERIOD_MS); // 100 Гц
    }
}

// Обработка видео (Ядро 1)
void video_task(void *pvParameters) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_CAM, &uart_config);
    uart_param_config(UART_VTX, &uart_config);
    uart_set_pin(UART_CAM, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_pin(UART_VTX, GPIO_NUM_19, GPIO_NUM_18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    while (1) {
        // Чтение видео с камеры
        int len = uart_read_bytes(UART_CAM, video_buffer, sizeof(video_buffer), 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            // Наложение линии горизонта (упрощенно)
            int horizon_y = 512 - (int)(pitch * 5); // Пример масштабирования
            for (int x = 0; x < 1024; x += 2) {
                if (x / 2 == horizon_y) video_buffer[x] = 0xFF; // Белая линия
            }
            // Передача на видеопередатчик
            uart_write_bytes(UART_VTX, (const char*)video_buffer, len);
        }
        vTaskDelay(33 / portTICK_PERIOD_MS); // ~30 FPS
    }
}

void app_main() {
    init_pwm();
    i2c_master_init(I2C_PORT, 21, 22, 400000);

    xTaskCreatePinnedToCore(flight_task, "flight_task", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(video_task, "video_task", 4096, NULL, 5, NULL, 1);
}