#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "driver/rmt.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "throttle.h"

/* ===================== CONFIG ===================== */
#define TAG "SCOOTER"

/* -------- I2C -------- */
#define I2C_SLAVE_ADDR   0x08
#define I2C_PORT         I2C_NUM_0
#define I2C_SDA          21
#define I2C_SCL          22
#define I2C_TX_BUF       256
#define I2C_RX_BUF       64

/* -------- RMT -------- */
#define RMT_CH           RMT_CHANNEL_0
#define RMT_GPIO         GPIO_NUM_25
#define RMT_CLK_DIV      80      // 1 us tick
#define T_UNIT_US        1000
#define TOL_US           300
#define SYNC_GAP_US      100000

/* -------- SPEED -------- */
#define POLE_PAIRS       24
#define HALL_PER_E_REV   6
#define HALL_PER_W_REV   (POLE_PAIRS * HALL_PER_E_REV)
#define WHEEL_DIAM_M     0.3048
#define WHEEL_CIRC_M     (3.1416 * WHEEL_DIAM_M)

/* -------- EVENT BITS -------- */
#define NEW_DATA_BIT     BIT0

/* ===================== GLOBALS ===================== */
static RingbufHandle_t rb;
static SemaphoreHandle_t data_mutex;
static EventGroupHandle_t event_group;

static uint32_t rmt_frames = 0;
static uint32_t i2c_writes = 0;

/* ===================== DATA ===================== */
typedef struct {
    uint16_t seq;
    float voltage;
    uint8_t soc;
    float speed;
    int8_t current;
    uint8_t brake;
    uint8_t mode;
    float throttle;
    bool valid;
    uint32_t timestamp;
} scooter_data_t;

static scooter_data_t data = {0};

// Separate read buffer that never changes during I2C reads
static char i2c_tx_buffer[128] = {0};

/* ===================== HELPERS ===================== */
static bool near(uint32_t v, uint32_t r) {
    return (v > (r - TOL_US)) && (v < (r + TOL_US));
}

static uint8_t checksum(uint8_t *d) {
    uint8_t c = 0;
    for (int i = 0; i <= 10; i++) c ^= d[i];
    return c;
}

static uint8_t pluscode(uint8_t l, uint8_t h) {
    uint8_t p = l + 0xBA;
    p ^= 0xD5; p += 0x97; p ^= 0x6A;
    p += (h & 0x0F);
    p ^= 0xFB; p += 0x4B; p ^= 0xEB;
    return p & 0x7F;
}

/* ===================== I2C ===================== */
static void i2c_init(void) {
    i2c_config_t c = {
        .mode = I2C_MODE_SLAVE,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .slave.slave_addr = I2C_SLAVE_ADDR
    };
    i2c_param_config(I2C_PORT, &c);
    i2c_driver_install(I2C_PORT, c.mode, I2C_RX_BUF, I2C_TX_BUF, 0);
}

/* ===================== RMT ===================== */
static void rmt_init(void) {
    rmt_config_t c = {
        .rmt_mode = RMT_MODE_RX,
        .channel = RMT_CH,
        .gpio_num = RMT_GPIO,
        .clk_div = RMT_CLK_DIV,
        .mem_block_num = 4,
        .rx_config.filter_en = true,
        .rx_config.filter_ticks_thresh = 50,
        .rx_config.idle_threshold = SYNC_GAP_US
    };
    rmt_config(&c);
    rmt_driver_install(RMT_CH, 4096, 0);
    rmt_get_ringbuf_handle(RMT_CH, &rb);
    rmt_rx_start(RMT_CH, true);
}

/* ===================== DECODER TASK ===================== */
static void decoder_task(void *arg) {
    ESP_LOGI(TAG, "RMT decoder started");

    while (1) {
        size_t len;
        rmt_item32_t *it = xRingbufferReceive(rb, &len, pdMS_TO_TICKS(1000));
        if (!it) continue;

        uint8_t raw[12] = {0};
        int bit = 0, byte = 0;
        bool have_low = false;
        uint32_t low = 0;

        for (int i = 0; i < len / 4 && byte < 12; i++) {
            uint32_t d[2] = { it[i].duration0, it[i].duration1 };
            int l[2] = { it[i].level0, it[i].level1 };

            for (int p = 0; p < 2; p++) {
                if (l[p] == 0) { low = d[p]; have_low = true; }
                else if (have_low) {
                    uint8_t b;
                    if (near(low, T_UNIT_US) && near(d[p], 2*T_UNIT_US)) b = 1;
                    else if (near(low, 2*T_UNIT_US) && near(d[p], T_UNIT_US)) b = 0;
                    else continue;

                    raw[byte] = (raw[byte] << 1) | b;
                    if (++bit == 8) { bit = 0; byte++; }
                    have_low = false;
                }
            }
        }
        vRingbufferReturnItem(rb, it);
        if (byte != 12 || checksum(raw) != raw[11]) continue;

        uint8_t seqL = raw[1];
        uint8_t seqH = (raw[2] >> 4) & 0x0F;
        uint8_t plus = pluscode(seqL, seqH);

        uint16_t speed_raw = ((raw[7]-plus)<<8)|(raw[8]-plus);
        float rps = (float)speed_raw / HALL_PER_W_REV / 0.5f;
        float speed_kmh = rps * WHEEL_CIRC_M * 3.6f;
        
        // Sanity check: max realistic speed is 100 km/h
        // If speed is absurd, it's likely corrupted data
        if (speed_kmh > 100.0f || speed_kmh < 0.0f) {
            speed_kmh = 0.0f;  // Default to 0 for bad readings
        }
        
        uint8_t mode_raw = raw[4] - plus;
        uint8_t mode_bits = mode_raw & 0x03;
        uint8_t brake = (mode_raw >> 5) & 1;
        
        // Scooter uses: 1=LOW, 2=MED, 3=HIGH (not 0,1,2)
        // Map to: 0=LOW, 1=MED, 2=HIGH for display
        uint8_t mode;
        if (mode_bits == 1) mode = 0;      // LOW
        else if (mode_bits == 2) mode = 1; // MED
        else if (mode_bits == 3) mode = 2; // HIGH
        else mode = mode_bits;             // Unknown - keep as-is

        xSemaphoreTake(data_mutex, portMAX_DELAY);
        data.seq = (seqH<<8)|seqL;
        data.voltage = (raw[9]-plus)*0.5f;
        data.soc = (raw[10]-plus);
        data.speed = speed_kmh;
        data.current = raw[6] & 0x80 ? -(raw[6]&0x7F) : (raw[6]&0x7F);
        data.brake = brake;
        data.mode = mode;
        data.valid = true;
        data.timestamp = xTaskGetTickCount();
        rmt_frames++;
        
        // Log mode changes for debugging (show raw mode_bits)
        static uint8_t last_mode_bits = 0xFF;
        if (mode_bits != last_mode_bits) {
            ESP_LOGI(TAG, "MODE CHANGE: %u -> %u (raw_bits=%u, mapped=%u, raw4=0x%02X)", 
                     last_mode_bits, mode_bits, mode_bits, mode, raw[4]);
            last_mode_bits = mode_bits;
        }
        
        xSemaphoreGive(data_mutex);
        
        // Signal I2C task that new data is available
        xEventGroupSetBits(event_group, NEW_DATA_BIT);
    }
}

/* ===================== I2C TASK ===================== */
static void i2c_task(void *arg) {
    ESP_LOGI(TAG, "I2C task started");
    
    // Initialize buffer with a default message
    memset(i2c_tx_buffer, ' ', sizeof(i2c_tx_buffer));
    snprintf(i2c_tx_buffer, sizeof(i2c_tx_buffer), 
             "<{\"seq\":0,\"v\":0.0,\"soc\":0,\"spd\":0.0,\"cur\":0,\"brk\":0,\"mode\":0,\"th\":0.0}>");
    i2c_slave_write_buffer(I2C_PORT, (uint8_t*)i2c_tx_buffer, sizeof(i2c_tx_buffer), 0);
    
    while (1) {
        // Wait for new data from decoder (with 2 second timeout)
        EventBits_t bits = xEventGroupWaitBits(
            event_group,
            NEW_DATA_BIT,
            pdTRUE,  // Clear bit after reading
            pdFALSE,
            pdMS_TO_TICKS(2000)
        );
        
        if (bits & NEW_DATA_BIT) {
            // New data available - update I2C buffer
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            
            // Always update throttle regardless of new RMT data
            data.throttle = throttle_get_percent();

            if (data.valid) {
                // Build JSON in temporary buffer first
                char temp[128];
                int n = snprintf(temp, sizeof(temp),
                    "<{\"seq\":%u,\"v\":%.1f,\"soc\":%u,\"spd\":%.1f,\"cur\":%d,\"brk\":%u,\"mode\":%u,\"th\":%.1f}>",
                    data.seq, data.voltage, data.soc,
                    data.speed, data.current, data.brake, data.mode, data.throttle
                );
                
                // Pad with spaces to fill buffer (ensures clean reads)
                for (int i = n; i < sizeof(temp); i++) {
                    temp[i] = ' ';
                }
                
                // Copy to I2C TX buffer atomically
                memcpy(i2c_tx_buffer, temp, sizeof(i2c_tx_buffer));
                
                // Update I2C slave buffer (no FIFO reset)
                i2c_slave_write_buffer(I2C_PORT, (uint8_t*)i2c_tx_buffer, 
                                      sizeof(i2c_tx_buffer), pdMS_TO_TICKS(10));
                i2c_writes++;
            }
            xSemaphoreGive(data_mutex);
        } else {
            // Even if no new RMT data, we might want to update throttle?
            // For now, only update when some data exists or on timeout
            xSemaphoreTake(data_mutex, portMAX_DELAY);
            data.throttle = throttle_get_percent();
            
            char temp[128];
            int n = snprintf(temp, sizeof(temp),
                "<{\"seq\":%u,\"v\":%.1f,\"soc\":%u,\"spd\":%.1f,\"cur\":%d,\"brk\":%u,\"mode\":%u,\"th\":%.1f}>",
                data.seq, data.voltage, data.soc,
                data.speed, data.current, data.brake, data.mode, data.throttle
            );
            for (int i = n; i < sizeof(temp); i++) temp[i] = ' ';
            memcpy(i2c_tx_buffer, temp, sizeof(i2c_tx_buffer));
            i2c_slave_write_buffer(I2C_PORT, (uint8_t*)i2c_tx_buffer, sizeof(i2c_tx_buffer), pdMS_TO_TICKS(10));
            xSemaphoreGive(data_mutex);
        }
    }
}

/* ===================== STATUS ===================== */
static void status_task(void *arg) {
    while (1) {
        xSemaphoreTake(data_mutex, portMAX_DELAY);
        ESP_LOGI(TAG, "STATUS - RMT: %lu | I2C: %lu | Seq: %u | Speed: %.1f km/h | Mode: %u | Volt: %.1fV | Throttle: %.1f%%",
                 rmt_frames, i2c_writes, data.seq, data.speed, data.mode, data.voltage, data.throttle);
        xSemaphoreGive(data_mutex);
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

/* ===================== MAIN ===================== */
void app_main(void) {
    data_mutex = xSemaphoreCreateMutex();
    event_group = xEventGroupCreate();

    i2c_init();
    rmt_init();
    throttle_init();

    ESP_LOGI(TAG, "Boot OK. I2C addr 0x%02X", I2C_SLAVE_ADDR);

    xTaskCreate(decoder_task, "decoder", 4096, NULL, 10, NULL);
    xTaskCreate(i2c_task, "i2c", 4096, NULL, 5, NULL);
    xTaskCreate(status_task, "status", 2048, NULL, 1, NULL);
}