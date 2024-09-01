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
uint8_t sampleData[] = {0x00,0x01,0x02};
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
    // [IMU_APP_ID] = {
    //     .gatts_cb = gatts_imu_app_event_handler,
    //     .gatts_if = ESP_GATT_IF_NONE,
    // },
    // [LIDAR_IMU_APP_ID] = {
    //     .gatts_cb = gatts_lidar_app_event_handler,
    //     .gatts_if = ESP_GATT_IF_NONE,
    // },
};

// Tasks
static QueueHandle_t uart2_queue; // Queue handle for interrupt, which adds an item to the queue on data received
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
    imuBank = imu_init(imu_device_handle, imuBank, ACC_SCALE, GYRO_SCALE, OFFS_VALS);

    // LED setup
    gpio_set_direction(LED,GPIO_MODE_OUTPUT); // Setup GPIO
    
    // Tasks
    //xTaskCreate(lidarRead_task, "uartRead", 2048, NULL, 10, NULL);
    //xTaskCreate(imuRead_task,"imuRead",2048,imu_device_handle,7,&imuTaskHandle);
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
                    ESP_LOGW(TAG,"Expected packet size not received: %d",event.size);
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

                    ESP_LOGI(TAG,"Distance = %d mm",lidarBuffer[3]<<8 | lidarBuffer[2]); // Print distance (debugging)
                    
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
        conn_params.max_int = 0x30; // max_int = 0x30*1.25ms = 40ms
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