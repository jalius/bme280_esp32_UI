/*
 *   Permission to use, copy, modify, and/or distribute this software for
 *   any purpose with or without fee is hereby granted.
 *
 *   THE SOFTWARE IS PROVIDED “AS IS” AND THE AUTHOR DISCLAIMS ALL
 *   WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES
 *   OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE
 *   FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY
 *   DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN
 *   AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
 *   OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "rom/ets_sys.h"
#include "esp_log.h"
#include "u8g2_esp32_hal.h"
#include "u8g2.h"
#include "common.h"

static const char* TAG = "ssh1106_bme280_test";
const gpio_num_t PIN_SDA = 5;
const gpio_num_t PIN_SCL = 6;
const gpio_num_t SSD1306_address = 0x78;
const uint32_t SELECTED_I2C_FREQUENCY = 100000;
const i2c_port_t SELECTED_I2C_PORT = I2C_NUM_0;

static bool bme280_setup(struct bme280_dev* dev, uint32_t* meas_time_us);
static bool get_measurements(uint32_t meas_time_us, struct bme280_dev *dev, struct bme280_data *out, bool Fahrenheit);
static void percent_str(const char* prefix, double percent, char* out_buf, int len);

uint8_t displaytype = 0;
const uint8_t DISPLAY_COUNT = 4;
int64_t button_time = 0;
const int BUTTON_KEEPALIVE_TIME = 60000 * 1000 * 3; // 3 minutes

static void IRAM_ATTR button_isr_handler(void* arg) {
  int gpio_num = (int) arg;
  int64_t cur_time = esp_timer_get_time();
  if (cur_time - button_time < BUTTON_KEEPALIVE_TIME){
          displaytype++; // only increment displaytype if display was awake
  }
  button_time = esp_timer_get_time();
  ESP_EARLY_LOGI(TAG, "Button pressed on GPIO %d, time %lld", gpio_num, button_time);
}

static bool bme280_setup(struct bme280_dev* dev, uint32_t* meas_time_us)
{
        // set up i2c config object, pins, frequency, etc

        bme280_i2c_conf conf = { 
                .addr = BME280_I2C_ADDR_PRIM,
                .i2c_port = SELECTED_I2C_PORT,
                .i2c_conf = {
                        .mode = I2C_MODE_MASTER,
                        .sda_io_num = PIN_SDA,
                        .sda_pullup_en = GPIO_PULLUP_ENABLE,
                        .scl_io_num = PIN_SCL,
                        .scl_pullup_en = GPIO_PULLUP_ENABLE,
                        .master.clk_speed = SELECTED_I2C_FREQUENCY
                                // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
                }	
        };

        bme280_init_i2c_dev(&conf, dev);

        if (bme280_init(dev) != BME280_OK) {
            return false;
        }

        struct bme280_settings settings;
        /* Always read the current settings before writing, especially when all the configuration is not modified */
        if (bme280_get_sensor_settings(&settings, dev) != BME280_OK) {
            return false;
        }


        /* Configuring the over-sampling rate, filter coefficient and standby time */
        /* Overwrite the desired settings */
        settings.filter = BME280_FILTER_COEFF_2;

        /* Over-sampling rate for humidity, temperature and pressure */
        settings.osr_h = BME280_OVERSAMPLING_1X;
        settings.osr_p = BME280_OVERSAMPLING_1X;
        settings.osr_t = BME280_OVERSAMPLING_1X;

        /* Setting the standby time */
        settings.standby_time = BME280_STANDBY_TIME_0_5_MS;

        if (bme280_set_sensor_settings(BME280_SEL_ALL_SETTINGS, &settings, dev) != BME280_OK) {
            return false;
        }

        /* Calculate measurement time in microseconds */
        bme280_cal_meas_delay(meas_time_us, &settings);

        // Normal Mode will continuously take measurements
        // Forced Mode will take 1 measurement and then go to sleep
        // 1 measurement period is meas_time_us(<9.3ms) plus standby time minimum (0.5ms)

        printf("\nHumidity calculation (Data displayed are compensated values)\n");
        printf("Measurement time : %lu us\n\n", (long unsigned int)*meas_time_us);
}
double C_to_F(double C)
{
        return C * (9/5) + 32;
}
static bool get_measurements(uint32_t meas_time_us, struct bme280_dev *dev, struct bme280_data *out, bool Fahrenheit)
{
        // start the measurement in FORCED mode
        bme280_set_sensor_mode(BME280_POWERMODE_FORCED, dev);
        uint8_t status_reg;
        bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev);
        // Check measurement status -> 1: running. 0: done
        if (status_reg & BME280_STATUS_MEAS_DONE){ 
                // wait for measurement to complete
                dev->delay_us(meas_time_us, dev->intf_ptr);
        }
        /* Read compensated data */
        bme280_get_sensor_data(BME280_ALL, out, dev);
        if (Fahrenheit)
        {
                out->temperature = C_to_F(out->temperature);
        }
        return true;
}
static void percent_str(const char* prefix, double percent, char* out_buf, int len)
{
        static const char hum_fmt[] = "%s:  %.2lf %%RH\n";
        snprintf(out_buf, len > 32 ? len : 32, hum_fmt, prefix, percent);
}
void setup_isr(void)
{
        // Настройка GPIO для кнопки
        button_time = esp_timer_get_time();

        gpio_config_t button_config = {
                .pin_bit_mask = (1ULL << GPIO_NUM_9),
                .mode = GPIO_MODE_INPUT,
                .pull_up_en = GPIO_PULLUP_ENABLE,
                .intr_type = GPIO_INTR_NEGEDGE 
        };
        gpio_config(&button_config);
        gpio_install_isr_service(0);
        gpio_isr_handler_add(GPIO_NUM_9, button_isr_handler, (void*) GPIO_NUM_9);

}
double calculate_avg(uint64_t cnt, double old_avg, double new_val)
{
        return ((cnt - 1) * old_avg + new_val)/(cnt);
}
void app_main(void)
{
        //esp_log_level_set("*", ESP_LOG_NONE);
        setup_isr();
        ESP_LOGI(TAG, "sleep for 2ms, wait for BME280 init");
        ets_delay_us(2000);

        ESP_LOGI(TAG, "i2c_driver_install %d", I2C_NUM_0);
        ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER,
                                I2C_MASTER_RX_BUF_DISABLE,
                                I2C_MASTER_TX_BUF_DISABLE, 0));

        u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
        u8g2_esp32_hal.bus.i2c.sda = PIN_SDA;
        u8g2_esp32_hal.bus.i2c.scl = PIN_SCL;
        u8g2_esp32_hal_init(u8g2_esp32_hal);

        u8g2_t u8g2;
        u8g2_Setup_sh1106_i2c_72x40_wise_f(&u8g2, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);
        u8x8_SetI2CAddress(&u8g2.u8x8, SSD1306_address);

        static struct bme280_dev dev;
        static uint32_t meas_time_us = 10000; //10ms default, actual value calculated by bme280 API
        const uint32_t idle_meas_time_us = 10000; // no change on idle measurement time for now
        bool setup_success = bme280_setup(&dev, &meas_time_us);
        ESP_LOGI(TAG, "u8g2_InitDisplay");
        u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
        while(1)
        {
                static uint64_t cnt = 0;
                cnt++;

                static char humidity[32];
                static char temperature[32];
                static char pressure[32];
                static struct bme280_data reading;
                static struct bme280_data maxread;
                static struct bme280_data minread;
                static struct bme280_data avgread;

                static int64_t last_time = 0;	
                int64_t cur_time = esp_timer_get_time();
                int64_t time_since_button = cur_time - button_time;

                bool idle_state = time_since_button > BUTTON_KEEPALIVE_TIME;

                // blocks for meas_time then gets data
                get_measurements(idle_state ? idle_meas_time_us : meas_time_us, &dev, &reading, true);

                // Store Max
                if (reading.humidity > maxread.humidity){
                        maxread.humidity = reading.humidity;
                }
                if (reading.temperature > maxread.temperature){
                        maxread.temperature = reading.temperature;
                }
                if (reading.pressure > maxread.pressure){
                        maxread.pressure = reading.pressure;
                }
                // Store Min
                if (reading.humidity < minread.humidity || minread.humidity == 0){
                        minread.humidity = reading.humidity;
                }
                if (reading.temperature < minread.temperature || minread.temperature == 0){
                        minread.temperature = reading.temperature;
                }
                if (reading.pressure < minread.pressure || minread.pressure == 0){
                        minread.pressure = reading.pressure;
                }
                // Store Avg
                avgread.humidity = calculate_avg(cnt, avgread.humidity, reading.humidity);
                avgread.temperature = calculate_avg(cnt, avgread.temperature, reading.temperature);
                avgread.pressure = calculate_avg(cnt, avgread.pressure, reading.pressure);
                // Switch display based on user input (pushbutton interrupt)
                switch (displaytype % DISPLAY_COUNT)
                {
                        case 0:
                        // Current
                        percent_str("Humi", reading.humidity, humidity, sizeof(humidity));
                        percent_str("Temp", reading.temperature, temperature, sizeof(temperature));
                        percent_str("Pres", reading.pressure, pressure, sizeof(pressure));
                        break;
                        case 1:
                        // Max
                        percent_str("HMax", maxread.humidity, humidity, sizeof(humidity));
                        percent_str("TMax", maxread.temperature, temperature, sizeof(temperature));
                        percent_str("PMax", maxread.pressure, pressure, sizeof(pressure));
                        break;
                        case 2:
                        // Min
                        percent_str("HMin", minread.humidity, humidity, sizeof(humidity));
                        percent_str("TMin", minread.temperature, temperature, sizeof(temperature));
                        percent_str("PMin", minread.pressure, pressure, sizeof(pressure));
                        break;
                        case 3:
                        // Avg
                        percent_str("HAvg", avgread.humidity, humidity, sizeof(humidity));
                        percent_str("TAvg", avgread.temperature, temperature, sizeof(temperature));
                        percent_str("PAvg", avgread.pressure, pressure, sizeof(pressure));
                        break;

                }

                int64_t delta_time = 0;
                if (last_time != 0){
                        delta_time = cur_time - last_time;
                }

                //ESP_LOGI(TAG, "deltatime %lld", delta_time);
                static char loopcount[32];
                snprintf(loopcount, sizeof(loopcount), "%lld", cnt);
                        
                // protect the display from multiple sleep commands
                static bool display_asleep = true;
                // check if button press occurred recently...
                if (idle_state)
                {
                        if (!display_asleep)
                        {
                                ESP_LOGI(TAG, "u8g2_SetPowerSave 1 (sleep)");
                                u8g2_SetPowerSave(&u8g2, 1); // sleep display
                                display_asleep = true;
                        }
                }
                else if (cnt % 10 == 0)
                {
                        if (display_asleep)
                        {
                                ESP_LOGI(TAG, "u8g2_SetPowerSave 0 (wake)");
                                u8g2_SetPowerSave(&u8g2, 0); // wake up display
                                display_asleep = false;
                        }

                        //ESP_LOGI(TAG, "u8g2_ClearBuffer");
                        u8g2_ClearBuffer(&u8g2);

                        //ESP_LOGI(TAG, "u8g2_SetFont");
                        u8g2_SetFont(&u8g2,u8g2_font_t0_11_mf);
                        //ESP_LOGI(TAG, "u8g2_DrawStr");
                        //u8g2_DrawStr(&u8g2, 1,1,"Hi welch!");
                        u8g2_DrawStr(&u8g2, 1, 8, humidity);
                        u8g2_DrawStr(&u8g2, 1, 18, temperature);
                        u8g2_DrawStr(&u8g2, 1, 28, pressure);
                        u8g2_DrawStr(&u8g2, 1, 28, pressure);
                        u8g2_DrawStr(&u8g2, 1, 38, loopcount);
                        //ESP_LOGI(TAG, "u8g2_SendBuffer");
                        u8g2_SendBuffer(&u8g2);

                        //ESP_LOGI(TAG, "All done!");
                }
                if (cnt % 100 == 0)
                {
                        printf("heap size: %" PRIu32 ", min %" PRIu32 " bytes\n", esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
                        //ESP_LOGI("Heapsize Cur: %u, Min: %u (bytes)\n", esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
                }
                last_time = cur_time;
        }
        vTaskDelete(NULL);
}
void old_main(void)
{
        printf("Hello world!\n");

        /* Print chip information */
        esp_chip_info_t chip_info = {};
        esp_chip_info(&chip_info);
        printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
                        CONFIG_IDF_TARGET,
                        chip_info.cores,
                        (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
                        (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
                        (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
                        (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

        unsigned major_rev = chip_info.revision / 100;
        unsigned minor_rev = chip_info.revision % 100;
        printf("silicon revision v%d.%d, ", major_rev, minor_rev);
        uint32_t flash_size = 0;
        if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
                printf("Get flash size failed");
                return;
        }

        printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
                        (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

        printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

        for (int i = 10; i >= 0; i--) {
                printf("Restarting in %d seconds...\n", i);
                vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        printf("Restarting now.\n");
        fflush(stdout);
        esp_restart();
}
