#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "hal/uart_ll.h"
#include "icm20948_funcs.h"
#include "icm20948_regs.h"

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
static QueueHandle_t uart2_queue; // Queue handle for interrupt, which adds an item to the queue on data received

// IMU
#define SCL 25 // D2. 4 (D12) Will be used in final PCB, but will ruin debugging now!
#define SDA 26 // D3. 12 (D13) Will be used in final PCB
#define IMU_BUF_SIZE 22
static QueueHandle_t imu_queue;
uint8_t imuBank; // Track current imu bank
uint8_t const ACC_SCALE = 1;
uint8_t const GYRO_SCALE = 2;
uint16_t const OFFS_VALS = 100u;


// LED
#define LED 2
static uint8_t led_state = 0;

// Function declarations
static void ledBlink_task(void *pvParameters);
static void uartRead_task(void *pvParameters);
static void imuRead_task(void *imu_handle);


void app_main() {

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
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NO,LIDAR_BUF_SIZE,LIDAR_BUF_SIZE,20,&uart2_queue,intr_alloc_flags));
    uart_intr_config_t uart_intr = { // Seteup interrupts
    .intr_enable_mask = UART_INTR_RXFIFO_FULL | UART_INTR_RXFIFO_TOUT,
    .rxfifo_full_thresh = 10, // Shouldn't hit this
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
    i2c_master_dev_handle_t imu_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &imu_handle));

    // Initialise icm20948
    imuBank = imu_init(imu_handle, imuBank, ACC_SCALE, GYRO_SCALE, OFFS_VALS);

    // LED
    gpio_set_direction(LED,GPIO_MODE_OUTPUT); // Setup GPIO
    
    // Tasks
    xTaskCreate(ledBlink_task, "ledBlink", 2048, NULL, 3, NULL);
    xTaskCreate(uartRead_task, "uartRead", 2048, NULL, 10, NULL);
    xTaskCreate(imuRead_task,"imuRead",2048,imu_handle,9,NULL);

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
        printf("LED mode switched!\n");
        printf("Time in LED task has been %.4f seconds!\n",(float)xTaskDetails.ulRunTimeCounter/1000);
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
    vTaskDelete(ledBlink_task);
}

// Task to read uart from lidar
static void uartRead_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* lidarBuffer = (uint8_t*) malloc(LIDAR_BUF_SIZE); // Buffer for lidar data
    imu_queue = xQueueCreate(10,sizeof(uint8_t)); // Queue can hold 10 uint8s
    uint8_t message;

    for(;;)
    {
        xQueueReceive(uart2_queue, (void *)&event, (TickType_t)portMAX_DELAY); // Blocks indefinately
        if (!event.timeout_flag)
        {
            printf("Warning, lidar data not sent by timeout as expected...");
        }
        if ( event.type == UART_DATA )
        {
            printf("Found %d bytes! ",event.size);
            uart_read_bytes(UART_PORT_NO, lidarBuffer, event.size ,portMAX_DELAY);
            printf("Distance = %d mm\n",lidarBuffer[3]<<8 | lidarBuffer[2]);

            // Push frame header to lidar queue, don't wait.
            message = lidarBuffer[0];
            if( xQueueSendToBack(imu_queue,(void*) &message,0) != pdPASS )
            {
                // If failed to add
                printf("Queue send to imu failed! There are %i spaces available", (int) uxQueueSpacesAvailable(imu_queue));
            }
        }
        else
        {
            printf("Warning, event other than data triggered: %d",event.type);

        }
    }
    vTaskDelete(uartRead_task);
}

// Task to read imu
static void imuRead_task(void *imu_handle)
{
    uint8_t imuBuffer[IMU_BUF_SIZE];
    uint8_t message;
    imu_queue = xQueueCreate(10,sizeof(uint8_t)); // Queue can hold 10 uint8s

    for(;;)
    {
        xQueueReceive(imu_queue, (void *)&message, (TickType_t)portMAX_DELAY); // Wait indefinately

        // We expect to recieve 0x59 from lidar task
        if( message == 0x59 )
        {
            // Read accel, gyro and external sensor (mag) registers
            imuBank = imu_read(imu_handle, imuBank, ACCEL_XOUT_H, imuBuffer, 22);
            printf("IMU value is %i\n",imuBuffer[18]);
        }

        vTaskDelete(imuRead_task);
        
    }
}