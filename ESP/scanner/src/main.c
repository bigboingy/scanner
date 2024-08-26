#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "hal/uart_ll.h"
#include "icm20948_funcs.h"
#include "icm20948_regs.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "nvs_flash.h"

static const char *TAG = "MAIN";

// Pin numbers
// USB
#define USB_TX 1
#define USB_RX 3

// Lidar UART
#define LIDAR_TX 23
#define LIDAR_RX 18 // SCK label
#define LIDAR_DATA_FREQ 100
#define UART_PORT_NO 2 // Port 0 is used for USB by default
#define BAUD 115200 // Used for lidar and USB
#define LIDAR_BUF_SIZE 1024 // DOES IT NEED TO BE THIS BIG?
#define LIDAR_PACKET_SIZE 9

// IMU
#define SCL 25 // D2. 4 (D12) Will be used in final PCB, but will ruin debugging now!
#define SDA 26 // D3. 12 (D13) Will be used in final PCB
#define IMU_BUF_SIZE 22
uint8_t imuBank; // Track current imu bank
uint8_t const ACC_SCALE = 1;
uint8_t const GYRO_SCALE = 2;
uint16_t const OFFS_VALS = 100u;

// LED
#define LED 2
static uint8_t led_state = 0;

// Tasks
static QueueHandle_t uart2_queue; // Queue handle for interrupt, which adds an item to the queue on data received
static TaskHandle_t imuTaskHandle = NULL; // Task handle for imu task

// Function declarations
static void ledBlink_task(void *pvParameters);
static void lidarRead_task(void *pvParameters);
static void imuRead_task(void *imu_handle);


void app_main() {

    ESP_LOGE(TAG, "This is error log");
    ESP_LOGW(TAG, "This is warning log");
    ESP_LOGI(TAG, "This is info log");
    ESP_LOGD(TAG, "This is debug log");
    ESP_LOGV(TAG, "This is verbose log");

    // Lidar UART setup
    uart_config_t uart_config = {
            .baud_rate = BAUD,
            .data_bits = UART_DATA_8_BITS,
            .parity    = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .source_clk = UART_SCLK_DEFAULT,
        };
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NO, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NO,LIDAR_TX,LIDAR_RX,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE));
    int intr_alloc_flags = 0;
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NO,LIDAR_BUF_SIZE,LIDAR_BUF_SIZE,LIDAR_BUF_SIZE,&uart2_queue,intr_alloc_flags));
    uart_intr_config_t uart_intr = { // Setup interrupts
    .intr_enable_mask = UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT,
    .rxfifo_full_thresh = 100, // fifo buffer is 128 bytes (this doesn't trigger anyway)
    .rx_timeout_thresh = 1, // If 1ms gap bw rx data detected, trigger interrupt
    };
    ESP_ERROR_CHECK(uart_intr_config(UART_PORT_NO, &uart_intr));
    ESP_ERROR_CHECK(uart_enable_rx_intr(UART_PORT_NO));

    // I2C setup
    i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = SCL,
    .sda_io_num = SDA,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x69,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t imu_device_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &imu_device_handle));

    // Bluetooth setup
    esp_err_t ret;
    ret = nvs_flash_init();
    // Initializing the non-volatile storage library
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialise icm20948
    imuBank = imu_init(imu_device_handle, imuBank, ACC_SCALE, GYRO_SCALE, OFFS_VALS);

    // LED setup
    gpio_set_direction(LED,GPIO_MODE_OUTPUT); // Setup GPIO
    
    // Tasks
    xTaskCreate(lidarRead_task, "uartRead", 2048, NULL, 10, NULL);
    //xTaskCreate(imuRead_task,"imuRead",2048,imu_device_handle,7,&imuTaskHandle);
    xTaskCreate(ledBlink_task, "ledBlink", 2048, NULL, 3, NULL);
}

// Task to blink the LED
static void ledBlink_task(void *pvParameters)
{
    for(;;)
    {
        // Turn on LED
        led_state = !led_state;
        gpio_set_level(LED,led_state);

        // Task info testing
        TaskHandle_t xHandle = xTaskGetHandle( "Task_Name" );
        TaskStatus_t xTaskDetails;
        vTaskGetInfo(xHandle,&xTaskDetails,pdTRUE,eInvalid);
        ESP_LOGI(TAG,"LED mode switched!\n");
        ESP_LOGI(TAG,"Time in LED task has been %.4f seconds!\n",(float)xTaskDetails.ulRunTimeCounter/1000);
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}

// Task to read uart from lidar
static void lidarRead_task(void *pvParameters)
{
    uart_event_t event;
    size_t len = 0;
    uint8_t *lidarBuffer = (uint8_t*) malloc(LIDAR_BUF_SIZE); // Buffer for lidar data
    uart_flush_input(UART_PORT_NO); // Flush existing data on startup
    xQueueReset(uart2_queue); // Remove any existing queue items
    
    for(;;)
    {
        xQueueReceive(uart2_queue, (void *)&event, portMAX_DELAY); // Blocks indefinately
        
        switch (event.type) {
            case UART_DATA:
                // Event interrupt validation
                if (event.size != LIDAR_PACKET_SIZE) {
                    ESP_LOGW(TAG,"Expected packet size not received: %d",event.size);
                    uart_flush(UART_PORT_NO); break; // Flush data and wait for next event
                }
                if (!event.timeout_flag) {ESP_LOGW(TAG,"Lidar data not sent by timeout as expected...");}

                // Data buildup warnings
                len = uxQueueMessagesWaiting( uart2_queue ); // Queue length
                if(len) ESP_LOGW(TAG,"There are %d items in the uart queue",len);
                uart_get_buffered_data_len(UART_PORT_NO,&len); // Ring buffer length
                if(len>LIDAR_PACKET_SIZE) ESP_LOGW(TAG,"There are %d bytes in the ring buffer",len);

                // Read bytes while packets remain in the ring buffer
                while (len)
                {
                    uart_read_bytes(UART_PORT_NO, lidarBuffer, LIDAR_PACKET_SIZE ,portMAX_DELAY);
                    if(imuTaskHandle != NULL) xTaskNotifyGive(imuTaskHandle); // Unblock imu task if it exists yet
                    ESP_LOGI(TAG,"Distance = %d mm",lidarBuffer[3]<<8 | lidarBuffer[2]);
                    len -= LIDAR_PACKET_SIZE; // Recalculate buffer length
                }
                break;
            
            case UART_FIFO_OVF: 
                // Hardware fifo overflows before the bytes can be copied out
                ESP_LOGW(TAG,"Uart fifo overflow");
                // Flush ring buffer so more data can be read from fifo, which is full
                uart_flush_input(UART_PORT_NO);
                xQueueReset(uart2_queue);
                break;

            case UART_BUFFER_FULL:
                // Bytes can't be copied into the ring buffer because it is full 
                ESP_LOGW(TAG,"Uart ring buffer full");
                // Flush ring buffer, which is full, to read more data from fifo
                uart_flush_input(UART_PORT_NO);
                xQueueReset(uart2_queue);
                break;
            case UART_BREAK: ESP_LOGW(TAG,"Uart break"); break;
            case UART_FRAME_ERR: ESP_LOGW(TAG,"Uart frame error"); break;
            case UART_PARITY_ERR: ESP_LOGW(TAG,"Uart parity error"); break;
            default: ESP_LOGW(TAG,"UART event: %d",event.type);
        }   
        
    }
}

// Task to read imu
static void imuRead_task(void *imu_device_handle)
{
    uint8_t imuBuffer[IMU_BUF_SIZE];

    for(;;)
    {
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY); // Block until notified by lidar task

        // Read accel, gyro and external sensor (mag) registers
        imuBank = imu_read(imu_device_handle, imuBank, ACCEL_XOUT_H, imuBuffer, 22);
        ESP_LOGI(TAG,"IMU value is %i\n",imuBuffer[18]);
        
    }
    vTaskDelete(imuTaskHandle);
}