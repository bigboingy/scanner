#include <stdio.h>
#include <inttypes.h>
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

#include "driver/gptimer.h"

// Logging tag
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
#define LIDAR_BUF_SIZE 256 // 129 is minimum (UART_HW_FIFO_LEN + 1), but not really :(
#define LIDAR_PACKET_SIZE 9

// IMU
#define SCL 4 // 25 (D2) used for debugging. 4 (D12) used in final PCB
#define SDA 12 // 26 (D3) used for debugging. 12 (D13) used in final PCB. Is also MTDI strapping, oops!
#define IMU_BUF_SIZE 22
uint8_t imuBank; // Track current imu bank
uint8_t const ACC_SCALE = 1;
uint8_t const GYRO_SCALE = 2;
uint16_t const OFFS_VALS = 100u;

struct imuTaskParams {
    i2c_master_dev_handle_t i2c_handle;
    gptimer_handle_t timer_handle;
};

// LED
#define LED 2
static uint8_t led_state = 0;

// BT
#define DEVICE_NAME          "SCANNER"
#define PROFILE_NUM          1 // lidar only
#define LIDAR_APP_ID         0
#define GATTS_HANDLE_NO      4 // Service, characteristic, characteristic value, characteristic descriptor
// Service uuids
#define GATTS_SERVICE_UUID_LIDAR   0x00FF // The S in GATTS is "server"
#define GATTS_SERVICE_UUID_IMU     0x00EE
// Characteristic uuids
#define GATTS_CHAR_UUID_DIST       0xFF01
#define GATTS_CHAR_UUID_ACC        0xFF02
#define GATTS_CHAR_UUID_GYRO       0xFF03
#define GATTS_CHAR_UUID_MAG        0xFF04
// Initial characteristic data
#define GATTS_CHAR_VAL_MAX_LEN 0x40
// Byte length of data to send over ble
#define BYTES_LIDAR 6
#define BYTES_IMU  22
#define BYTES_COMB BYTES_LIDAR+BYTES_IMU

uint8_t sampleData[3] = {0x00,0x01,0x02};
static esp_attr_value_t lidar_char_val = { // Attribute is a generic term for any piece of data within the GATT database.
    .attr_max_len = GATTS_CHAR_VAL_MAX_LEN, // Max length in bytes
    .attr_len = sizeof(sampleData),
    .attr_value = sampleData
};

// Function declarations
static void ledBlink_task(void *pvParameters);
static void lidarRead_task(void *pvParameters);
static void imuRead_task(void *imu_handle);
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gatts_lidar_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

// Profile UUIDs 128 bits
static uint8_t uuids[16*PROFILE_NUM] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //lidar + imu uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
    // //imu uuid, 12bit, [12], [13] is the value
    // 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    // //lidar + imu uuid, 12bit, [12], [13] is the value
    // 0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
};
// GAP advertisement parameters used in configuration (what's shown to client)
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0010, // Set in units of 1.25 ms
    .max_interval = 0x0010, // Apple reccommends 20ms
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(uuids),
    .p_service_uuid = uuids,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// GAP scan response parameters (packet sent when more info is requested by device)
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(uuids),
    .p_service_uuid = uuids,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// GAP advertisement parameters used when actually starting advertising (config to start GAP)
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20, // We want 20ms for apple = 0x20 * 0.625ms
    .adv_int_max        = 0x20,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
// GATT application profile structure
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if; // Apps on the GATT client use different interfaces, represented by the gatts_if parameter
    uint16_t app_id;
    uint16_t conn_id;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
};
// The Application Profiles are stored in an array
static struct gatts_profile_inst gatts_profile_tab[PROFILE_NUM] = {
    [LIDAR_APP_ID] = {
        .gatts_cb = gatts_lidar_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,
    },
};

// Struct for lidar data
struct lidarData {
    uint16_t dist; // Distance
    uint16_t str; // Strength
    uint16_t temp; // Temperature
};
// Structs for imu data
struct cartesian {
    int16_t x;
    int16_t y;
    int16_t z;
};
struct imuData {
    struct cartesian acc;
    struct cartesian gyro;
    struct cartesian mag;
    uint16_t temp;
    uint16_t count;
};

// Tasks and queues
static QueueHandle_t uart2_queue; // Queue handle for interrupt, which adds an item to the queue on data received
static QueueHandle_t ble_queue_lidar; // Queue for tracking lidar data to send over btle
static QueueHandle_t ble_queue_imu; // Queue for tracking imu data to send over btle
static TaskHandle_t imuTaskHandle = NULL; // Task handle for imu task

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
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NO,LIDAR_BUF_SIZE,LIDAR_BUF_SIZE,10,&uart2_queue,intr_alloc_flags));
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
    .flags.enable_internal_pullup = true, // Use internal 45 kOhm resistor for slightly stronger pullup
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

    // Timer setup
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, // Use default clock
        .direction = GPTIMER_COUNT_UP, // Count up
        .resolution_hz = 1000000, // 1 MHz, 1 tick = 1 us
        .intr_priority = 0, // Low priority interrupt (won't be used anyway)
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config,&gptimer));
    ESP_LOGI(TAG,"Enabling timer");
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_LOGI(TAG,"Starting timer");
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    // Bluetooth setup
    esp_err_t ret;
    // Initializing the non-volatile storage library
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
        ESP_LOGE(TAG, "%s initialize controller failed", __func__);
    }
    ESP_ERROR_CHECK(ret);
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg); // Controller initialisation
    if (ret) { ESP_LOGE(TAG, "%s initialize controller failed", __func__); return;}
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE); // Controller enable
    if (ret) { ESP_LOGE(TAG, "%s init bluetooth failed", __func__); return;}
    ret = esp_bluedroid_init(); // Bluedroid initialisation
    if (ret) { ESP_LOGE(TAG, "%s init bluetooth failed", __func__); return;}
    ret = esp_bluedroid_enable(); // Bluedroid enable
    if (ret) { ESP_LOGE(TAG, "%s enable bluetooth failed", __func__); return;}
    ret = esp_ble_gatts_register_callback(gatts_event_handler); // Set GATTS callback func
    if (ret){ ESP_LOGE(TAG, "gatts app register error, error code = %x", ret); return;}
    ret = esp_ble_gap_register_callback(gap_event_handler); // Set GAP callback func
    if (ret){ ESP_LOGE(TAG, "gatts app register error, error code = %x", ret); return;}
    // Register GATT application profiles
    ret = esp_ble_gatts_app_register(LIDAR_APP_ID);
    if (ret){ ESP_LOGE(TAG, "gatts app register error, error code = %x", ret); return;}
    // ret = esp_ble_gatts_app_register(IMU_APP_ID);
    // if (ret){ ESP_LOGE(TAG, "gatts app register error, error code = %x", ret); return;}
    // ret = esp_ble_gatts_app_register(LIDAR_IMU_APP_ID);
    // if (ret){ ESP_LOGE(TAG, "gatts app register error, error code = %x", ret); return;}

    // Initialise icm20948
    vTaskDelay(100/portTICK_PERIOD_MS); // 100ms delay so that flicking the switch isn't felt in gyroscope calibration
    ESP_LOGI(TAG,"Calibrating IMU");
    imuBank = imu_init(imu_device_handle, imuBank, ACC_SCALE, GYRO_SCALE, OFFS_VALS);

    // LED setup
    gpio_set_direction(LED,GPIO_MODE_OUTPUT); // Setup GPIO
    
    // Tasks
    struct imuTaskParams imuTask = {imu_device_handle,gptimer}; // Need to prepare params for imu task
    xTaskCreate(lidarRead_task, "uartRead", 4096, NULL, 10, NULL); // Stack overflow using 2048
    xTaskCreate(imuRead_task,"imuRead",2048,&imuTask,7,&imuTaskHandle);
    xTaskCreate(ledBlink_task, "ledBlink", 2048, NULL, 3, NULL);
}

// Task to blink the LED
static void ledBlink_task(void *pvParameters)
{
    // TaskHandle_t xHandle = xTaskGetHandle( "Task_Name" );
    // TaskStatus_t xTaskDetails;
    for(;;)
    {
        // Turn on LED
        led_state = !led_state;
        gpio_set_level(LED,led_state);

        // Task info testing
        // vTaskGetInfo(xHandle,&xTaskDetails,pdTRUE,eInvalid);
        // ESP_LOGI(TAG,"LED mode switched!\n");
        // ESP_LOGI(TAG,"Time in LED task has been %.4f seconds!\n",(float)xTaskDetails.ulRunTimeCounter/1000);
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
                // Event validation
                if (event.size != LIDAR_PACKET_SIZE) { // Unexpected number of bytes
                    uart_read_bytes(UART_PORT_NO, lidarBuffer, event.size ,portMAX_DELAY);
                    ESP_LOGW(TAG,"Expected packet size not received: %d.  This may indicate memory leak",event.size);
                    for( uint8_t i = 0; i<event.size; i++ )
                    {
                        ESP_LOGW(TAG,"Byte %d: %d",i,lidarBuffer[i]);
                    }
                    uart_flush(UART_PORT_NO); // Flush data from ring buffer (important, may be leftovers)
                    xQueueReset(uart2_queue); // Reset queue (or out of frame reading occurs?)
                    break; // Wait for next event
                }
                if (!event.timeout_flag) {ESP_LOGW(TAG,"Lidar data not sent by timeout as expected...");} // Unexpected reason

                // Data buildup warnings
                len = uxQueueMessagesWaiting( uart2_queue ); // Queue length
                if(len) ESP_LOGW(TAG,"There are %d items in the uart queue",len);
                uart_get_buffered_data_len(UART_PORT_NO,&len); // Ring buffer length
                if(len>LIDAR_PACKET_SIZE) ESP_LOGW(TAG,"There are %d bytes in the ring buffer",len);

                // Read all packets in the ring buffer
                while (len)
                {
                    uart_read_bytes(UART_PORT_NO, lidarBuffer, LIDAR_PACKET_SIZE ,portMAX_DELAY); // Read 1 packet
                    len -= LIDAR_PACKET_SIZE; // Recalculate buffer length

                    // Validate checksum
                    uint16_t checksum=0;
                    for(uint8_t i=0; i<LIDAR_PACKET_SIZE-1; i++)
                    {
                        checksum += lidarBuffer[i];
                    }
                    if ((checksum & 0xFF) != lidarBuffer[LIDAR_PACKET_SIZE-1])
                    {
                        ESP_LOGW(TAG,"Checksum failed!");
                        break;
                    }

                    if(imuTaskHandle != NULL) xTaskNotifyGive(imuTaskHandle); // Unblock imu task if it exists yet
                    else break; // Otherwise, we don't really want to keep this read
                    //ESP_LOGI(TAG,"Distance = %d mm",lidarBuffer[3]<<8 | lidarBuffer[2]); // Print distance (debugging)

                    // Make sure there's room in the queue to send to
                    UBaseType_t room = uxQueueSpacesAvailable(ble_queue_lidar);
                    if( !room ){ // If no space, remove top pointer
                        //ESP_LOGW(TAG,"ble_queue_lidar full, removing top pointer"); // Prints too much when disconnected
                        struct lidarData *top;
                        xQueueReceive(ble_queue_lidar, (void *)&top, (TickType_t)0);
                        free(top); // Free the dynamically allocated memory given to this struct
                    }

                    // Add lidar data to ble_queue_lidar
                    // Prepare structre
                    struct lidarData *dataPointer = malloc(sizeof(struct lidarData)); // Makes space for the struct
                    dataPointer->dist = lidarBuffer[3]<<8 | lidarBuffer[2];
                    dataPointer->str = lidarBuffer[5]<<8 | lidarBuffer[4];
                    dataPointer->temp = lidarBuffer[7]<<8 | lidarBuffer[6];
                    // Add pointer to queue. Don't block if queue is full
                    if(xQueueSendToBack(ble_queue_lidar,(void *) &dataPointer,(TickType_t) 0) == pdPASS){
                        //ESP_LOGI(TAG,"The distance %d was added to the ble queue",dataPointer->dist);
                    } else {
                        ESP_LOGW(TAG,"Failed to add pointer to ble_queue_lidar");
                    }
                    // free(dataPointer); // Don't free the memory yet. The pointer was queued by copy, not by reference
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
            default: ESP_LOGW(TAG,"UART event: %d",event.type); break;
        }
    }
    // Shouldn't reach this point
    free(lidarBuffer);
}

// Task to read imu
static void imuRead_task(void *pvParameters)
{
    // Extract params
    struct imuTaskParams* params = (struct imuTaskParams*)pvParameters; // Cast pointer to correct type
    i2c_master_dev_handle_t imu_device_handle = params->i2c_handle;
    gptimer_handle_t gptimer = params->timer_handle;

    uint8_t imuBuffer[IMU_BUF_SIZE]; // Initialise i2c read buffer
    uint64_t count; // For timer

    for(;;)
    {
        ulTaskNotifyTake(pdTRUE,portMAX_DELAY); // Block until notified by lidar task

        // Read accel, gyro and external sensor (mag) registers
        imuBank = imu_read(imu_device_handle, imuBank, ACCEL_XOUT_H, imuBuffer, 22);

        // Timer - get current count. Reset count to zero after filling the struct
        gptimer_get_raw_count(gptimer,&count); // Read

        // Make sure there's room in the queue to send to
        UBaseType_t room = uxQueueSpacesAvailable(ble_queue_imu);
        if( !room ){ // If no space, remove top pointer
            // ESP_LOGW(TAG,"ble_queue_imu full, removing top pointer");
            struct imuData *top;
            xQueueReceive(ble_queue_imu, (void *)&top, (TickType_t)0);
            free(top); // Free the dynamically allocated memory given to this struct
        }

        // Make imuData
        struct imuData *dataPointer = malloc(sizeof(struct imuData));
        struct cartesian acc,gyro,mag;
        acc.x = imuBuffer[0]<<8 | imuBuffer[1];
        acc.y = imuBuffer[2]<<8 | imuBuffer[3];
        acc.z = imuBuffer[4]<<8 | imuBuffer[5];
        gyro.x = imuBuffer[6]<<8 | imuBuffer[7];
        gyro.y = imuBuffer[8]<<8 | imuBuffer[9];
        gyro.z = imuBuffer[10]<<8 | imuBuffer[11]; 
        dataPointer->temp = imuBuffer[12]<<8 | imuBuffer[13]; // 12 and 13 are temp
        mag.x = imuBuffer[14] | imuBuffer[15]<<8;  // Low then high
        mag.y = imuBuffer[16] | imuBuffer[17]<<8;
        mag.z = imuBuffer[18] | imuBuffer[19]<<8; // 20 is nothing, 21 is magnetic overflow (control 2)
        if(imuBuffer[21] & 0x08) ESP_LOGW(TAG,"Magnetic overflow occurred");
        dataPointer->acc = acc; dataPointer->gyro = gyro; dataPointer->mag = mag;
        dataPointer -> count = (uint16_t) count; // Convert to uint16.
        // We expect a count around e-2/e-6=e4 which is within uint16's range, but if not print a warning
        if( count > 0xFFFF ) ESP_LOGW(TAG,"Count overflow, 64 bit value: %"PRIu64"",count);
        gptimer_set_raw_count(gptimer,0); // Reset timer, immediately starts counting

        // Add pointer to data to imu_queue
        if( xQueueSendToBack(ble_queue_imu, (void *)&dataPointer, (TickType_t)0 ) == pdPASS ){
            // Success
            // ESP_LOGI(TAG,"Sending mag %f x %f y %f z",(float)mag.x*0.15,(float)mag.y*0.15,(float)mag.z*0.15);
        } else {
            ESP_LOGW(TAG,"Failed to add pointer to ble_queue_imu");
        }
    }
    vTaskDelete(imuTaskHandle);
}

// BT
// GATT master event handler, forwards event to the profile event handlers
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    // Profile register event
    if( event == ESP_GATTS_REG_EVT ) {
        if( param->reg.status == ESP_GATT_OK ) {
            gatts_profile_tab[param->reg.app_id].gatts_if = gatts_if; // Store the GATT interface number
        }
        else {
            ESP_LOGI(TAG,"Reg app failed, app_id %04x, status %d",
                param->reg.app_id, param->reg.status);
            return;
        }
    }

    // Call corresponding profile's callback
    // Have to check each one in the tab to see if it has the right if number
    // Example uses a do while 0, idk why
    for( uint8_t i=0; i<PROFILE_NUM; i++) {
        // Call if the gatts_if number matches. If a certain profile hasn't been specified call then all
        if( gatts_profile_tab[i].gatts_if == gatts_if || gatts_if == ESP_GATT_IF_NONE ) {
            // Call the callback if it exists
            if (gatts_profile_tab[i].gatts_cb) {
            gatts_profile_tab[i].gatts_cb(event, gatts_if, param);
            }
        }
    }
}

// GATT profile event handlers
static void gatts_lidar_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch(event){
    case ESP_GATTS_REG_EVT: // On startup (registering the application)
        ESP_LOGI(TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        // Set parameters on the profile tab
        // Lidar service
        gatts_profile_tab[LIDAR_APP_ID].service_id.is_primary = true;
        gatts_profile_tab[LIDAR_APP_ID].service_id.id.inst_id = 0x00;
        gatts_profile_tab[LIDAR_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gatts_profile_tab[LIDAR_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_LIDAR;
        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(DEVICE_NAME);
        if (set_dev_name_ret){ESP_LOGE(TAG, "set device name failed, error code = %x", set_dev_name_ret);}
        // Configure data advertising
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){ESP_LOGE(TAG, "config adv data failed, error code = %x", ret);}
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){ESP_LOGE(TAG, "config scan response data failed, error code = %x", ret);}
        // Example sets flags here too...
        // Create services
        esp_ble_gatts_create_service(gatts_if,&gatts_profile_tab[LIDAR_APP_ID].service_id,GATTS_HANDLE_NO); // lidar service

        // Initialise the ble lidar queue, 10 long is frequently full, 20 long is rarely full, 30 is safe (using 15-40ms ble interval)
        ble_queue_lidar = xQueueCreate(30, sizeof(struct lidarData *)); // Stores pointers to lidarData
        if( ble_queue_lidar == NULL ){ 
            ESP_LOGE(TAG,"Ble lidar queue creation failed");
        } else {
            ESP_LOGI(TAG,"Ble lidar queue created");
        }
        
        // Same for imu queue
        ble_queue_imu = xQueueCreate(30, sizeof(struct imuData *));
        if( ble_queue_imu == NULL ){ 
            ESP_LOGE(TAG,"Ble imu queue creation failed");
        } else {
            ESP_LOGI(TAG,"Ble imu queue created");
        }
        break;

    case ESP_GATTS_CREATE_EVT: // When service created (by previous case)
        ESP_LOGI(TAG,"CREATE SERVICE EVT, status %d, service_handle %d", param->create.status,param->create.service_handle);
        // Set parameters of the profile in the gatts tab
        gatts_profile_tab[LIDAR_APP_ID].service_handle = param->create.service_handle;
        gatts_profile_tab[LIDAR_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gatts_profile_tab[LIDAR_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_DIST;
        // Start the lidar service
        esp_ble_gatts_start_service(gatts_profile_tab[LIDAR_APP_ID].service_handle);
        // Add the distance characteristic to the lidar service
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gatts_profile_tab[LIDAR_APP_ID].service_handle,
                                &gatts_profile_tab[LIDAR_APP_ID].char_uuid,
                                 // Advertise read/write allowed
                                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                 // Actually allow read/write/notification
                                ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                &lidar_char_val, NULL);
        if (add_char_ret){ESP_LOGE(TAG, "add char failed, error code =%x",add_char_ret);}
        break;
    // When characteristic has been created (by previous case)
    case ESP_GATTS_ADD_CHAR_EVT: {  // Need braces if want a local scope
        ESP_LOGI(TAG,"ADD_CHAR_EVT, status %d, attr_handle %d, service handle %d",
            param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
            // Add to gatts tab
            gatts_profile_tab[LIDAR_APP_ID].char_handle = param->add_char.attr_handle;
            gatts_profile_tab[LIDAR_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
            gatts_profile_tab[LIDAR_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
            // Get starting attribute length and values to print
            uint16_t len=0; const uint8_t *val;
            esp_err_t ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle,&len,&val);
            if( ret == ESP_FAIL) ESP_LOGE(TAG,"ILLEGAL HANDLE");
            ESP_LOGI(TAG, "the gatts characteristic length = %x", len);
            for( uint8_t i=0; i<len; i++) ESP_LOGI(TAG,"Characteristic value %x = %x,",i,val[i]);
            // Add characteristic description
            ret = esp_ble_gatts_add_char_descr(
                gatts_profile_tab[LIDAR_APP_ID].service_handle, // Can't be the characteristic handle!
                &gatts_profile_tab[LIDAR_APP_ID].descr_uuid,
                ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                NULL, NULL );
            if(ret != ESP_OK ) ESP_LOGE(TAG,"add char descr failed, error code = %x", ret);
            break;
        }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT: // When characteristic description has been added (by prev case)
        ESP_LOGI(TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                  param->add_char.status, param->add_char.attr_handle,
                  param->add_char.service_handle);
        break;

    case ESP_GATTS_START_EVT: // On service start (in ESP_GATTS_CREATE_EVT)
        ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d",
                param->start.status, param->start.service_handle);
        break;

    case ESP_GATTS_CONNECT_EVT:  // On connection
        // Setup connection parameters
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t)); // Copy over bt device address
        conn_params.latency = 0;
        conn_params.max_int = 0x18; // max_int = 0x18*1.25ms = 30ms (We could have 15ms and use a smaller ble queue)
        conn_params.min_int = 0x0C; // min_int = 0x0C*1.25ms = 15ms (Apple recommended)
        conn_params.timeout = 600; // timeout = 600*10ms = 6000ms (Apple says 6-18 s)
        esp_ble_gap_update_conn_params(&conn_params); // Process update
        // Conenction message
        ESP_LOGI(TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
            param->connect.conn_id,param->connect.remote_bda[0],param->connect.remote_bda[1],param->connect.remote_bda[2],
            param->connect.remote_bda[3],param->connect.remote_bda[4],param->connect.remote_bda[5]);

        // Store connection id in gatts tab 
        gatts_profile_tab[LIDAR_APP_ID].conn_id = param->connect.conn_id;
        break;

    case ESP_GATTS_DISCONNECT_EVT: // On disconnect
        ESP_LOGI(TAG,"ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x",param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params); // Start advertising again?
        break;

    case ESP_GATTS_READ_EVT: // On read request
        // ESP_LOGI(TAG,"GATT_READ_EVT, conn_id %d, trans id %"PRIu32", handle %d",
        //         param->read.conn_id, param->read.trans_id, param->read.handle);

        esp_gatt_rsp_t rsp; // Response struct
        memset(&rsp,0,sizeof(esp_gatt_rsp_t)); // Initialise rsp to 0
        rsp.attr_value.handle = param -> read.handle; // Which characteristic has been requested

        // 1. Make sure that there is at least 1 lidar and imu struct ready
        uint8_t numLidar = (uint8_t)uxQueueMessagesWaiting(ble_queue_lidar); // Number of items in lidar queue
        uint8_t numImu = (uint8_t)uxQueueMessagesWaiting(ble_queue_imu); // Number of items in imu queue
        while( !(numLidar && numImu) ) { // If there's not a lidar and an imu
            vTaskDelay(10/portTICK_PERIOD_MS); // Wait for 10 ms
            numLidar = (uint8_t)uxQueueMessagesWaiting(ble_queue_lidar); // Update queue sizes
            numImu = (uint8_t)uxQueueMessagesWaiting(ble_queue_lidar);
        }
        uint8_t readsToSend = (numLidar<numImu) ? numLidar : numImu; // Get the minimum queue size

        // Make sure we aren't exceeding MTU limit
        if( (BYTES_COMB)*readsToSend > ESP_GATT_MAX_ATTR_LEN ) {
            readsToSend = 18; // floor(ESP_GATT_MAX_ATTR_LEN / (BYTES_COMB)
            ESP_LOGW(TAG,"More data available than can be sent in one ble transaction");
        }

        // 2. Read the queue
        // The memory for the struct has already been allocated, 
        struct lidarData *data_lidar; // Points to the lidar struct when the queue is read
        struct imuData *data_imu;
        rsp.attr_value.len = BYTES_LIDAR * readsToSend + BYTES_IMU * readsToSend; // Lidar is 6 bytes, imu is 20
        // For each queue message
        for(uint8_t i=0; i<readsToSend; i++)
        {
            // Direct *data_lidar pointer to the lidarData struct
            if( xQueueReceive(ble_queue_lidar,&data_lidar,0) == pdTRUE ){ // &data_lidar is a pointer to a pointer
            //ESP_LOGI(TAG,"The distance %d was read from the ble queue",data->dist);
            rsp.attr_value.value[(BYTES_COMB)*i+0] = data_lidar->dist & 0xFF; // Dist low
            rsp.attr_value.value[(BYTES_COMB)*i+1] = data_lidar->dist >> 8; // Dist high
            rsp.attr_value.value[(BYTES_COMB)*i+2] = data_lidar->str & 0xFF; // Str low
            rsp.attr_value.value[(BYTES_COMB)*i+3] = data_lidar->str >> 8; // Str high
            rsp.attr_value.value[(BYTES_COMB)*i+4] = data_lidar->temp & 0xFF; // Temp low
            rsp.attr_value.value[(BYTES_COMB)*i+5] = data_lidar->temp >> 8; // Temp high
            free(data_lidar); // Now we can free the memory
            }
            else ESP_LOGW(TAG,"ble_queue_lidar receive failed"); // Leave as zeros

            // Set data_imu pointer
            if( xQueueReceive(ble_queue_imu,&data_imu,0) == pdTRUE ){
                rsp.attr_value.value[(BYTES_COMB)*i+6] = data_imu->acc.x >> 8; // Acc x high
                rsp.attr_value.value[(BYTES_COMB)*i+7] = data_imu->acc.x & 0xFF; // Acc x low
                rsp.attr_value.value[(BYTES_COMB)*i+8] = data_imu->acc.y >> 8; // Acc y high
                rsp.attr_value.value[(BYTES_COMB)*i+9] = data_imu->acc.y & 0xFF; // Acc y low
                rsp.attr_value.value[(BYTES_COMB)*i+10] = data_imu->acc.z >> 8; // Acc z high
                rsp.attr_value.value[(BYTES_COMB)*i+11] = data_imu->acc.z & 0xFF; // Acc z low
                rsp.attr_value.value[(BYTES_COMB)*i+12] = data_imu->gyro.x >> 8; // Gyro x high
                rsp.attr_value.value[(BYTES_COMB)*i+13] = data_imu->gyro.x & 0xFF; // Gyro x low
                rsp.attr_value.value[(BYTES_COMB)*i+14] = data_imu->gyro.y >> 8; // Gyro y high
                rsp.attr_value.value[(BYTES_COMB)*i+15] = data_imu->gyro.y & 0xFF; // Gyro y low
                rsp.attr_value.value[(BYTES_COMB)*i+16] = data_imu->gyro.z >> 8; // Gyro z high
                rsp.attr_value.value[(BYTES_COMB)*i+17] = data_imu->gyro.z & 0xFF; // Gyro z low
                rsp.attr_value.value[(BYTES_COMB)*i+18] = data_imu->temp >> 8; // Temp high
                rsp.attr_value.value[(BYTES_COMB)*i+19] = data_imu->temp & 0xFF; // Temp low
                rsp.attr_value.value[(BYTES_COMB)*i+20] = data_imu->mag.x & 0xFF; // Mag x low
                rsp.attr_value.value[(BYTES_COMB)*i+21] = data_imu->mag.x >> 8; // Mag x high
                rsp.attr_value.value[(BYTES_COMB)*i+22] = data_imu->mag.y & 0xFF; // Mag y low
                rsp.attr_value.value[(BYTES_COMB)*i+23] = data_imu->mag.y >> 8; // Mag y high
                rsp.attr_value.value[(BYTES_COMB)*i+24] = data_imu->mag.z & 0xFF; // Mag z low
                rsp.attr_value.value[(BYTES_COMB)*i+25] = data_imu->mag.z >> 8; // Mag z high
                rsp.attr_value.value[(BYTES_COMB)*i+26] = data_imu->count >> 8; // Count high
                rsp.attr_value.value[(BYTES_COMB)*i+27] = data_imu->count & 0xFF; // Count low
                free(data_imu); // Free the memory
            }
            else ESP_LOGW(TAG,"ble_queue_imu receive failed"); // Leave as zeros to send
        }
        // 3. Send!
        esp_ble_gatts_send_response(gatts_if,param->read.conn_id,param->read.trans_id,ESP_GATT_OK, &rsp);
        // ESP_LOGI(TAG,"Sent %d packets",readsToSend);
        break;
    default: break;
    }
}

// GAP
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT: // When the GAP advertising data has been set by gatts event handler
        esp_ble_gap_start_advertising(&adv_params); // Start advertising
        ESP_LOGI(TAG,"Starting ble advertising");
        // Example checks a flag first
        break;

    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT: // When the GAP scan response data has been set by gatts event handler
        esp_ble_gap_start_advertising(&adv_params); // Start advertising
        ESP_LOGI(TAG,"Starting ble scan response");
        // Example checks a flag first
        break;

    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT: // Advertisment has started (from this func)
        if( param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS ) ESP_LOGE(TAG, "Advertising start failed");
        else ESP_LOGI(TAG,"Successfully started advertising bt");
        break;

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT: // After connection params are updated (by GATT event handler)
        ESP_LOGI(TAG, "update connection params status = %d, conn_int = %d, latency = %d, timeout = %d",
                param->update_conn_params.status, param->update_conn_params.conn_int,
                param->update_conn_params.latency, param->update_conn_params.timeout);
        break;

    default: break;
    }
}