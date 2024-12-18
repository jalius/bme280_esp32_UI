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
#include <string.h>
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
const uint8_t DISPLAY_COUNT = 7;
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
        if (bme280_cal_meas_delay(meas_time_us, &settings) != BME280_OK) {
            return false;
        }


        // Normal Mode will continuously take measurements
        // Forced Mode will take 1 measurement and then go to sleep
        // 1 measurement period is meas_time_us(<9.3ms) plus standby time minimum (0.5ms)

        printf("\nHumidity calculation (Data displayed are compensated values)\n");
        printf("Measurement time : %lu us\n\n", (long unsigned int)*meas_time_us);
        return true;
}
double C_to_F(double C)
{
        return C * (9/5) + 32;
}
static bool get_measurements(uint32_t meas_time_us, struct bme280_dev *dev, struct bme280_data *out, bool Fahrenheit)
{
        // start the measurement in FORCED mode
        if(bme280_set_sensor_mode(BME280_POWERMODE_FORCED, dev) != BME280_OK){
            return false;
        }
        uint8_t status_reg;
        if (bme280_get_regs(BME280_REG_STATUS, &status_reg, 1, dev) != BME280_OK) {
            return false;
        }
        // Check measurement status -> 1: running. 0: done
        if (status_reg & BME280_STATUS_MEAS_DONE){ 
                // wait for measurement to complete
                dev->delay_us(meas_time_us, dev->intf_ptr);
        }
        /* Read compensated data */
        if (bme280_get_sensor_data(BME280_ALL, out, dev) != BME280_OK) {
            return false;
        }
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
        //cnt is the number of samples in the old avg
        return ((cnt) * old_avg + new_val)/(cnt+1);
}
typedef struct TempAvgBuffer
{
    double *temp_avgs;
    double *hum_avgs;
    int cur_idx; // index of the current average, one past the last stored element
    int avg_count; // count of the number of samples in the current average 
    int valid_count; // number of valid avgs currently stored
    int max; // the avgs array max count
    int64_t last_stored_time;// time the we last stored an avg

}TempAvgBuffer;
            
const int ONE_MINUTE_US = 1000*1000*60;
bool get_historical_reading(TempAvgBuffer* buf, struct bme280_data* out, int offset)
{
    // offset int range: [0, buf->valid_count)
    if (offset < 0 || offset >= buf->valid_count){
        return false;
    }
    int idx = buf->cur_idx - (offset+1); // offset = n, gets the avg from n+1 minute(s) ago...
    // clamp circular buffer [0, buf->max)
    if (idx < 0){
        idx+=buf->max;
    }
    else if (idx >= buf->max){
        idx-=buf->max;
    }
    out->temperature = buf->temp_avgs[idx];
    out->humidity = buf->hum_avgs[idx];
    ESP_LOGI(TAG, "avg from %d minutes ago: temp %f hum %f", offset+1, out->temperature, out->humidity);
    return true;
}
bool calculate_historical_average(TempAvgBuffer* buf, int start, int end, struct bme280_data* out)
{   
    //start offset and end offset.
    // example: 1H average -> start = 0, end = 60
    if (start > end || end > buf->valid_count) {
        return false;
    }
    int count = end - start;
    int cnt = 0;
    memset(out, 0, sizeof(struct bme280_data));
    for (int i = start; i < count; i++)
    {
       struct bme280_data cur = {0}; 
       if (!get_historical_reading(buf, &cur, i)){
            return false; 
       }
       out->temperature = calculate_avg(cnt, out->temperature, cur.temperature);
       out->humidity = calculate_avg(cnt, out->humidity, cur.humidity);
       cnt++;
    }
    return true;;
}
void print_historical_averages(TempAvgBuffer* buf)
{
    double avg_temp_tot = 0;
    double avg_hum_tot = 0;
    int avg_running_cnt = 0;
    for (int i = 0; i < buf->valid_count; i++)
    {
        struct bme280_data data;
        get_historical_reading(buf, &data, i);
        avg_temp_tot = calculate_avg(avg_running_cnt, avg_temp_tot, data.temperature);
        avg_hum_tot = calculate_avg(avg_running_cnt, avg_hum_tot, data.humidity);
        avg_running_cnt++;
    }
    for (int i = 0; i < buf->max; i++)
    {
        
        ESP_LOGI(TAG, "[%d] buf pos: temp %f hum %f", i, buf->temp_avgs[i], buf->hum_avgs[i]);
    }
    ESP_LOGI(TAG, "avg of %d samples: temp %f hum %f", avg_running_cnt, avg_temp_tot, avg_hum_tot);
}
void store_historical_averages(TempAvgBuffer* buf, double temp, double hum)
{
    int64_t cur_time = esp_timer_get_time();
    if (buf->last_stored_time == 0){
        buf->last_stored_time = cur_time;
    }
    // if it's been a minute since we last stored a sample, store it
    if (cur_time - buf->last_stored_time > ONE_MINUTE_US){
        ESP_LOGI(TAG, "Storing Avg: # %d temp %f hum %f", buf->avg_count, buf->temp_avgs[buf->cur_idx], buf->hum_avgs[buf->cur_idx]);
        // we store by advancing the circular index
       buf->cur_idx++; 
       // circular buffer: roll over and start overwriting when we pass max
       buf->cur_idx %= buf->max;
       // increment the valid entry count, range [0, max)
       if (buf->valid_count < buf->max - 1) {
            buf->valid_count++;
       }
        // then, start the averaging in the new position
        // reset the avg_count and the samples at the cur_idx
        buf->avg_count = 1;
        buf->temp_avgs[buf->cur_idx] = temp;
        buf->hum_avgs[buf->cur_idx] = hum;

        // track this as the last stored time
        buf->last_stored_time = cur_time; 
        ESP_LOGI(TAG, "cur_idx %d valid_count %d", buf->cur_idx, buf->valid_count);
        //print_historical_averages(buf);
    }
    else { // it's been less than a minute, continue adding to the current average
        buf->temp_avgs[buf->cur_idx] = calculate_avg(buf->avg_count, buf->temp_avgs[buf->cur_idx], temp);
        buf->hum_avgs[buf->cur_idx] = calculate_avg(buf->avg_count, buf->hum_avgs[buf->cur_idx], hum);
        // increment the average count after adding the new sample
        buf->avg_count++;

        //ESP_LOGI(TAG, "Added new sample temp %f hum %f", temp, hum);
    }

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
        bool setup_BME280_success = bme280_setup(&dev, &meas_time_us);
        ESP_LOGI(TAG, "u8g2_InitDisplay");
        u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
        TempAvgBuffer avg_buf = { 
            .temp_avgs = malloc (sizeof(double) * 10081),
            .hum_avgs = malloc (sizeof(double) * 10081), // 7 days worth at 1 minute per sample
            .cur_idx = 0,
            .avg_count = 0,
            .valid_count = 0,
            .max = 10081,
            .last_stored_time = 0
        };
        int64_t start_time = esp_timer_get_time();
        while(1)
        {
                static uint64_t cnt = 0;
                cnt++;
                int64_t active_seconds = (esp_timer_get_time() - start_time) / 1000000 % 60;
                int64_t active_minutes = (esp_timer_get_time() - start_time) / 1000000 / 60;

                static char line1[32];
                static char line2[32];
                static char line3[32];
                static char line4[32];
                static struct bme280_data reading;
                static struct bme280_data maxread;
                static struct bme280_data minread;
                static struct bme280_data avgread;

                //static int64_t last_time = 0;	
                int64_t cur_time = esp_timer_get_time();

                // update time since button (ISR) was pressed
                int64_t time_since_button = cur_time - button_time;

                // idle state is when we have exceeded the BUTTON_KEEPALIVE_TIME
                bool idle_state = time_since_button > BUTTON_KEEPALIVE_TIME;

                // blocks for meas_time then gets data
                if (!get_measurements(idle_state ? idle_meas_time_us : meas_time_us, &dev, &reading, true)) {
                    // comms failed, break!
                    break;
                }
                store_historical_averages(&avg_buf, reading.temperature, reading.humidity);
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
                avgread.humidity = calculate_avg(cnt-1, avgread.humidity, reading.humidity);
                avgread.temperature = calculate_avg(cnt-1, avgread.temperature, reading.temperature);
                avgread.pressure = calculate_avg(cnt-1, avgread.pressure, reading.pressure);
                // Switch display based on user input (pushbutton interrupt)
                switch (displaytype % DISPLAY_COUNT)
                {
                        case 0:
                        // Current
                        percent_str("Humi", reading.humidity, line1, sizeof(line1));
                        percent_str("Temp", reading.temperature, line2, sizeof(line2));
                        percent_str("Pres", reading.pressure, line3, sizeof(line3));
                        snprintf(line4, sizeof(line4), "%6lld:%02lld", active_minutes, active_seconds);
                        break;
                        case 1:
                        // Max
                        percent_str("HMax", maxread.humidity, line1, sizeof(line1));
                        percent_str("TMax", maxread.temperature, line2, sizeof(line2));
                        percent_str("PMax", maxread.pressure, line3, sizeof(line3));
                        snprintf(line4, sizeof(line4), "%6lld:%02lld", active_minutes, active_seconds);
                        break;
                        case 2:
                        // Min
                        percent_str("HMin", minread.humidity, line1, sizeof(line1));
                        percent_str("TMin", minread.temperature, line2, sizeof(line2));
                        percent_str("PMin", minread.pressure, line3, sizeof(line3));
                        snprintf(line4, sizeof(line4), "%6lld:%02lld", active_minutes, active_seconds);
                        break;
                        case 3:
                        // Avg
                        percent_str("HAvg", avgread.humidity, line1, sizeof(line1));
                        percent_str("TAvg", avgread.temperature, line2, sizeof(line2));
                        percent_str("PAvg", avgread.pressure, line3, sizeof(line3));
                        snprintf(line4, sizeof(line4), "%6lld:%02lld", active_minutes, active_seconds);
                        break;
                        case 4:
                        {
                            // Avg Past 10m
                            struct bme280_data avg10m = {0};
                            calculate_historical_average(&avg_buf, 0, 10, &avg10m);
                            percent_str("H10m", avg10m.humidity, line1, sizeof(line1));
                            percent_str("T10m", avg10m.temperature, line2, sizeof(line2));
                            struct bme280_data avg30m = {0};
                            // Avg Past 30m
                            calculate_historical_average(&avg_buf, 0, 30, &avg30m);
                            percent_str("H30m", avg30m.humidity, line3, sizeof(line3));
                            percent_str("T30m", avg30m.temperature, line4, sizeof(line4));
                            break;
                        }
                        case 5:
                        {
                            // Avg Past 1H
                            struct bme280_data avg1 = {0};
                            calculate_historical_average(&avg_buf, 0, 60, &avg1);
                            percent_str("H1Hr", avg1.humidity, line1, sizeof(line1));
                            percent_str("T1Hr", avg1.temperature, line2, sizeof(line2));
                            struct bme280_data avg2 = {0};
                            // Avg Past 8H
                            calculate_historical_average(&avg_buf, 0, 60*8, &avg2);
                            percent_str("H8Hr", avg2.humidity, line3, sizeof(line3));
                            percent_str("T8Hr", avg2.temperature, line4, sizeof(line4));
                            break;
                        }
                        case 6:
                        {
                            // Avg Past 1D
                            struct bme280_data avg1 = {0};
                            calculate_historical_average(&avg_buf, 0, 60*24, &avg1);
                            percent_str("H1Da", avg1.humidity, line1, sizeof(line1));
                            percent_str("T1Da", avg1.temperature, line2, sizeof(line2));
                            struct bme280_data avg2 = {0};
                            // Avg Past 7D
                            calculate_historical_average(&avg_buf, 0, 60*24*7, &avg2);
                            percent_str("H7Da", avg2.humidity, line3, sizeof(line3));
                            percent_str("T7Da", avg2.temperature, line4, sizeof(line4));
                            break;
                        }

                }

                //int64_t delta_time = 0;
                //if (last_time != 0){
                //        delta_time = cur_time - last_time;
                //}
                //ESP_LOGI(TAG, "deltatime %lld", delta_time);
                        
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
                        u8g2_DrawStr(&u8g2, 1, 8, line1);
                        u8g2_DrawStr(&u8g2, 1, 18, line2);
                        u8g2_DrawStr(&u8g2, 1, 28, line3);
                        u8g2_DrawStr(&u8g2, 1, 38, line4);
                        //ESP_LOGI(TAG, "u8g2_SendBuffer");
                        u8g2_SendBuffer(&u8g2);

                        //ESP_LOGI(TAG, "All done!");
                }
                if (cnt % 100 == 0)
                {
                        printf("heap size: %" PRIu32 ", min %" PRIu32 " bytes\n", esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
                        //ESP_LOGI("Heapsize Cur: %u, Min: %u (bytes)\n", esp_get_free_heap_size(), esp_get_minimum_free_heap_size());
                }
                //last_time = cur_time;
        }
        if (!setup_BME280_success){
            ESP_LOGI(TAG, "Setup Failed BME280");
            u8g2_SetPowerSave(&u8g2, 0); // wake up display
            u8g2_ClearBuffer(&u8g2);
            u8g2_SetFont(&u8g2,u8g2_font_t0_11_mf);
            u8g2_DrawStr(&u8g2, 1,8,"Setup Failed"); // max screen width is 12 of this font
            u8g2_SendBuffer(&u8g2);
        }
        else{
            ESP_LOGI(TAG, "Comms Failed BME280");
            u8g2_SetPowerSave(&u8g2, 0); // wake up display
            u8g2_ClearBuffer(&u8g2);
            u8g2_SetFont(&u8g2,u8g2_font_t0_11_mf);
            u8g2_DrawStr(&u8g2, 1,8,"Comms Failed BME280");
            u8g2_SendBuffer(&u8g2);
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
