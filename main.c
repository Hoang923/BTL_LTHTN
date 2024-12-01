#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "driver/i2c.h"
#include "esp_timer.h"

// Cấu hình I2C cho LCD
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_FREQ_HZ 100000
#define LCD_ADDR 0x27
#define I2C_MASTER_NUM I2C_NUM_0

// Cấu hình HC-SR04
#define TRIGGER_PIN 23
#define ECHO_PIN 19

// Cấu hình LED và buzzer
#define LED_PIN 18
#define BUZZER_PIN 5

// Tín hiệu LCD
#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE 0x04
#define LCD_COMMAND 0x00
#define LCD_DATA 0x01

// Hàm gửi tín hiệu I2C đến LCD
esp_err_t i2c_send_byte(uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LCD_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

// Gửi tín hiệu điều khiển LCD
void lcd_send(uint8_t data, uint8_t mode) {
    uint8_t upper_nibble = (data & 0xF0) | LCD_BACKLIGHT | mode;
    uint8_t lower_nibble = ((data << 4) & 0xF0) | LCD_BACKLIGHT | mode;

    i2c_send_byte(upper_nibble | LCD_ENABLE);
    i2c_send_byte(upper_nibble & ~LCD_ENABLE);

    i2c_send_byte(lower_nibble | LCD_ENABLE);
    i2c_send_byte(lower_nibble & ~LCD_ENABLE);
}

// Gửi lệnh LCD
void lcd_command(uint8_t cmd) {
    lcd_send(cmd, LCD_COMMAND);
}

// Gửi dữ liệu LCD
void lcd_data(uint8_t data) {
    lcd_send(data, LCD_DATA);
}

// Khởi tạo LCD
void lcd_init() {
    lcd_command(0x33);
    lcd_command(0x32);
    lcd_command(0x28);
    lcd_command(0x0C); 
    lcd_command(0x06);
    lcd_command(0x01); 
    vTaskDelay(5 / portTICK_PERIOD_MS); 
}

// In chuỗi lên LCD
void lcd_print(const char *str) {
    while (*str) {
        lcd_data((uint8_t)(*str));
        str++;
    }
}

// Hàm đo khoảng cách từ HC-SR04
float measure_distance() {
    gpio_set_level(TRIGGER_PIN, 0);
    esp_rom_delay_us(2);
    gpio_set_level(TRIGGER_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(TRIGGER_PIN, 0);

    int64_t start_time = esp_timer_get_time();
    while (gpio_get_level(ECHO_PIN) == 0);
    start_time = esp_timer_get_time();

    while (gpio_get_level(ECHO_PIN) == 1);
    int64_t end_time = esp_timer_get_time();

    int64_t duration = end_time - start_time;

    float distance = (duration / 2.0) * 0.0343;
    return distance;
}

// Task đo khoảng cách từ cảm biến HC-SR04
void task_measure_distance(void *pvParameter) {
    while (1) {
        float distance = measure_distance();
        printf("Distance: %.2f cm\n", distance);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // Đợi 1 giây
    }
}

// Task điều khiển LCD và hiển thị khoảng cách
void task_display_lcd(void *pvParameter) {
    lcd_init();  // Khởi tạo LCD

    while (1) {
        float distance = measure_distance(); // Đo khoảng cách
        lcd_command(0x01);  // Xóa màn hình
        lcd_command(0x80);  // Đặt con trỏ về đầu dòng
        lcd_print("Distance: ");
		lcd_command(0xC0);
        char buffer[16];
        snprintf(buffer, sizeof(buffer), "%.2f cm", distance);
        lcd_print(buffer);

        vTaskDelay(1000 / portTICK_PERIOD_MS); // Đợi 1 giây
    }
}

// Task điều khiển LED và buzzer
void task_led_warning(void *pvParameter) {
    while (1) {
        float distance = measure_distance();

        if (distance < 10) {
            // Dưới 10 cm: Đèn sáng, còi kêu liên tục
            gpio_set_level(LED_PIN, 1);
            gpio_set_level(BUZZER_PIN, 1);
        } else if (distance < 50) {
			gpio_set_level(LED_PIN, 1);
            gpio_set_level(BUZZER_PIN, 1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN, 0);
            gpio_set_level(BUZZER_PIN, 0);
            vTaskDelay(100 / portTICK_PERIOD_MS);
		} else if (distance < 100) {
            // Từ 10 cm đến 100 cm: Đèn nhấp nháy, còi bật/tắt
            gpio_set_level(LED_PIN, 1);
            gpio_set_level(BUZZER_PIN, 1);
            vTaskDelay(250 / portTICK_PERIOD_MS);
            gpio_set_level(LED_PIN, 0);
            gpio_set_level(BUZZER_PIN, 0);
            vTaskDelay(250 / portTICK_PERIOD_MS);
        } else {
            // Trên 100 cm: Tắt đèn và còi
            gpio_set_level(LED_PIN, 0);
            gpio_set_level(BUZZER_PIN, 0);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS); // Đợi 100 ms để cập nhật khoảng cách
    }
}

void app_main() {
    // Cấu hình I2C
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &i2c_config);
    i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);

    // Cấu hình GPIO cho HC-SR04
    gpio_set_direction(TRIGGER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ECHO_PIN, GPIO_MODE_INPUT);
    
    // Cấu hình GPIO cho LED và buzzer
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(BUZZER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
    gpio_set_level(BUZZER_PIN, 0);

    // Tạo Task đo khoảng cách
    xTaskCreate(task_measure_distance, "Measure Distance", 2048, NULL, 1, NULL);

    // Tạo Task hiển thị LCD
    xTaskCreate(task_display_lcd, "Display LCD", 2048, NULL, 1, NULL);
    
    // Tạo Task điều khiển LED và buzzer
    xTaskCreate(task_led_warning, "LED Warning", 2048, NULL, 1, NULL);
}
