#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "driver/pwm.h"

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
    uint32_t freq = 50; // 50 Гц для ESC и сервоприводов
    pwm_init(MOTOR_PIN, freq, 1000); // 1000 мкс - центр
    pwm_init(ELEVON1_PIN, freq, 1000);
    pwm_init(ELEVON2_PIN, freq, 1000);
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

// Ассемблер для быстрого вычисления PID (пример)
float pid_control(float setpoint, float measured) {
    static float error_prev = 0, integral = 0;
    float kp = 1.0, ki = 0.1, kd = 0.05;
    float error = setpoint - measured;
    integral += error;
    float derivative = error - error_prev;
    float output;
    asm volatile (
        "fmul %0, %1, %2\n"  // output = kp * error
        "fmul %3, %4, %5\n"  // temp = ki * integral
        "fadd %0, %0, %3\n"  // output += temp
        "fmul %3, %6, %7\n"  // temp = kd * derivative
        "fadd %0, %0, %3\n"  // output += temp
        : "=f" (output), "=f" (error), "=f" (integral), "=f" (derivative)
        : "f" (kp), "f" (ki), "f" (kd), "r" (error_prev)
    );
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
        pwm_set_duty(MOTOR_PIN, throttle);
        pwm_set_duty(ELEVON1_PIN, 1500 + roll_output + pitch_output);
        pwm_set_duty(ELEVON2_PIN, 1500 - roll_output + pitch_output);

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
    i2c_master_init(I2C_PORT, I2C_SDA, I2C_SCL, 400000);

    xTaskCreatePinnedToCore(flight_task, "flight_task", 4096, NULL, 10, NULL, 0);
    xTaskCreatePinnedToCore(video_task, "video_task", 4096, NULL, 5, NULL, 1);
}