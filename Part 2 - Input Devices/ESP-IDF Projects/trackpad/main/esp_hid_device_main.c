#define CONFIG_BT_SDP_PAD_LEN (500)
#define CONFIG_BT_SDP_ATTR_LEN (400)

#define SDP_MAX_PAD_LEN (500)
#define SDP_MAX_ATTR_LEN (400)

#define HID_DEV_MTU_SIZE (384)

#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "rtc_wdt.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include <esp_log.h>
#include "driver/i2c_master.h"

#include "esp_hidd_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_bt.h"

#include "esp_err.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "esp_gap_bt_api.h"

#include "freertos/semphr.h"


#include "hid_desc.h"



#define REPORT_PROTOCOL_MOUSE_REPORT_SIZE      (39)
#define REPORT_BUFFER_SIZE                     REPORT_PROTOCOL_MOUSE_REPORT_SIZE

esp_hidd_app_param_t app_param;
esp_hidd_qos_param_t both_qos;
uint8_t protocol_mode;
SemaphoreHandle_t touch_mutex;
TaskHandle_t touch_task_hdl;
uint8_t outputBuffer[REPORT_BUFFER_SIZE];


i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .scl_io_num = 22,
    .sda_io_num = 21,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = false,
};

i2c_master_bus_handle_t bus_handle;


i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x15,
    .scl_speed_hz = 100000,
};

i2c_master_dev_handle_t dev_handle;

uint8_t buf[16];
uint8_t buffer[1024];

void connectTouchpad(){
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
	printf("Added TouchPad to I2C Bus\n");
	
	buf[0] = 0x05;
    buf[1] = 0x00;
    buf[2] = 0x00;
    buf[3] = 0x01;
    
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, 4, buffer, 2, -1));
    
    printf("Reset done!\n");
}

void printTouchpadDebug(){
	//buf[0] = 0x05;
    //buf[1] = 0x00;
    //buf[2] = 0x00;
    //buf[3] = 0x08;
    
   // ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, 4, buffer, 2, -1));
    
    //vTaskDelay(100 / portTICK_PERIOD_MS);
    
    //buf[0] = 0x00;
    //buf[1] = 0x03;
    //buf[2] = 0x01;
   // buf[3] = 0x00;
    
    //ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, 4, buffer, 2, -1));
    
    //vTaskDelay(100 / portTICK_PERIOD_MS);
    
   // buf[0] = 0x05;
    //buf[1] = 0x00;
    //buf[2] = 0x00;
    //buf[3] = 0x08;
    
    //ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, 4, buffer, 2, -1));
    
    //vTaskDelay(100 / portTICK_PERIOD_MS);
    
    
    buf[0] = 0x01;
    buf[1] = 0x00;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, 2, buffer, 30, -1));
	printf("\n---> HID Descriptor: ");
	for (int i = 0; i < 30; i++) {
        printf("%02x ", buffer[i]);
        fflush(stdout);
    }
    printf("\n");
    fflush(stdout);
    
    buf[0] = 0x02;
    buf[1] = 0x00;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, 2, buffer, 381, -1));
	printf("\n---> HID Report Descriptor: ");
	for (int i = 0; i < 381; i++) {
        printf("%02x ", buffer[i]);
        fflush(stdout);
    }
    printf("\n");
    fflush(stdout);
}

void touchpadSwitchMode(){
	buf[0] = 0x05;
	buf[1] = 0x00;
	buf[2] = 0b00110011;
	buf[3] = 0b00000010;
	buf[4] = 0x06;
	buf[5] = 0x00;
	
	ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, 6, buffer, 8, -1));
	
	printf("GET_REPORT 3: ");
	for (int i = 0; i < 8; i++) {
	    printf("%02x ", buffer[i]);
	}
	printf("\n");
	
	vTaskDelay(100 / portTICK_PERIOD_MS);
	
	buf[0] = 0x05;
	buf[1] = 0x00;
	buf[2] = 0b00110011;
	buf[3] = 0b00000011;
	
	buf[4] = 0x06;
	buf[5] = 0x00;
	
	buf[6] = 0x05;
	buf[7] = 0x00;
	buf[8] = 0x03;
	buf[9] = 0x03;
	buf[10] = 0x00;
	
	//ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, 6, buffer, 8, -1));
	ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buf, 11, -1));
	printf("SET_REPORT 3: ");
	printf("\n");
	
	vTaskDelay(100 / portTICK_PERIOD_MS);
	
	buf[0] = 0x05;
	buf[1] = 0x00;
	buf[2] = 0b00110011;
	buf[3] = 0b00000010;
	buf[4] = 0x06;
	buf[5] = 0x00;
	
	ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, 6, buffer, 8, -1));
	
	printf("GET_REPORT 3: ");
	for (int i = 0; i < 8; i++) {
	    printf("%02x ", buffer[i]);
	}
	printf("\n");
}

esp_err_t ret;

static void print_bt_address(void)
{
    const char *TAG = "bt_address";
    const uint8_t *bd_addr;

    bd_addr = esp_bt_dev_get_address();
    ESP_LOGI(TAG, "my BT address is %02X:%02X:%02X:%02X:%02X:%02X",
             bd_addr[0], bd_addr[1], bd_addr[2], bd_addr[3], bd_addr[4], bd_addr[5]);
}


bool running = false;


void touch_move_task(void *pvParameters){
	uint8_t buf[16];
	uint8_t buffer[1024];
	while(true){
		xSemaphoreTake(touch_mutex, portMAX_DELAY);
		if(running){
			for(int i = 0; i < 128; i++){
				buffer[i] = 0;
			}
			ESP_ERROR_CHECK(i2c_master_receive(dev_handle, buffer, 16, -1));
			if(buffer[0] != 0 || buffer[1] != 0 || buffer[2] != 0){
				//printf("Incoming report: ");
		    	unsigned short xCoord = (buffer[4] | (buffer[5] << 8));
		    	unsigned short yCoord = (buffer[6] | (buffer[7] << 8));
		    	bool onTouchpad = (buffer[3] & 0b00000010) >> 1;
		    	bool button = (buffer[11] & 0b00000001);
		    	bool amogus = (buffer[11] & 0b00001000) >> 3;
		    	//printf("X: %d, Y: %d, Removed: %s, Button: %s, Amog: %s", xCoord, yCoord, onTouchpad ? "no" : "yes", button ? "pressed" : "depressed :(", amogus ? "ussy" : "us");
		    	
		        esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 4, 16, &buffer[3]);
		        
		    	int initialNumberOfFingers = buffer[10];
		    	int numberOfFingers = buffer[10] - 1;
		    	while(numberOfFingers > 0){
					numberOfFingers--;
					
					//printf(" - Additional Finger (%d): ", initialNumberOfFingers - numberOfFingers);
					ESP_ERROR_CHECK(i2c_master_receive(dev_handle, buffer, 16, -1));
					if(buffer[0] != 0 || buffer[1] != 0 || buffer[2] != 0){
						unsigned short xCoord = (buffer[4] | (buffer[5] << 8));
				    	unsigned short yCoord = (buffer[6] | (buffer[7] << 8));
				    	bool onTouchpad = (buffer[3] & 0b00000010) >> 1;
				    	bool button = (buffer[11] & 0b00000001);
				    	bool amogus = (buffer[11] & 0b00001000) >> 3;
						//printf("X: %d, Y: %d, Removed: %s, Button: %s, Amog: %s", xCoord, yCoord, onTouchpad ? "no" : "yes", button ? "pressed" : "depressed :(", amogus ? "ussy" : "us");
						
				        esp_bt_hid_device_send_report(ESP_HIDD_REPORT_TYPE_INTRDATA, 4, 16, &buffer[3]);
				        
					}
					else{
						printf(" - received goofy ahh data");
						
					}
					
				}
				
				//printf("\n");
			}
		}
		else{
			vTaskDelay(40 / portTICK_PERIOD_MS);
		}
        xSemaphoreGive(touch_mutex);
        rtc_wdt_feed();
		vTaskDelay(12 / portTICK_PERIOD_MS);
		
	}
	return;
	
}
void bt_app_task_start_up(void)
{
	const char *TAG = "bt_app_task_start_up";
	ESP_LOGI(TAG, "starting...");
    xSemaphoreTake(touch_mutex, portMAX_DELAY);
    running = true;
    xSemaphoreGive(touch_mutex);
 
    return;
}

void bt_app_task_shut_down(void)
{
    xSemaphoreTake(touch_mutex, portMAX_DELAY);
    running = false;
    xSemaphoreGive(touch_mutex);
    return;
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    const char *TAG = "esp_bt_gap_cb";
    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "authentication success: %s", param->auth_cmpl.device_name);
            ESP_LOG_BUFFER_HEX(TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT: {
        ESP_LOGI(TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%"PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif
    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;
    default:
        ESP_LOGI(TAG, "event: %d", event);
        break;
    }
    return;
}

void esp_bt_hidd_cb(esp_hidd_cb_event_t event, esp_hidd_cb_param_t *param)
{
    static const char *TAG = "esp_bt_hidd_cb";
    switch (event) {
    case ESP_HIDD_INIT_EVT:
        if (param->init.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "setting hid parameters");
            esp_bt_hid_device_register_app(&app_param, &both_qos, &both_qos);
        } else {
            ESP_LOGE(TAG, "init hidd failed!");
        }
        break;
    case ESP_HIDD_DEINIT_EVT:
        break;
    case ESP_HIDD_REGISTER_APP_EVT:
        if (param->register_app.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "setting hid parameters success!");
            ESP_LOGI(TAG, "setting to connectable, discoverable");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            if (param->register_app.in_use) {
                ESP_LOGI(TAG, "start virtual cable plug!");
                esp_bt_hid_device_connect(param->register_app.bd_addr);
            }
        } else {
            ESP_LOGE(TAG, "setting hid parameters failed!");
        }
        break;
    case ESP_HIDD_UNREGISTER_APP_EVT:
        if (param->unregister_app.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "unregister app success!");
        } else {
            ESP_LOGE(TAG, "unregister app failed!");
        }
        break;
    case ESP_HIDD_OPEN_EVT:
        if (param->open.status == ESP_HIDD_SUCCESS) {
            if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTING) {
                ESP_LOGI(TAG, "connecting...");
            } else if (param->open.conn_status == ESP_HIDD_CONN_STATE_CONNECTED) {
                ESP_LOGI(TAG, "connected to %02x:%02x:%02x:%02x:%02x:%02x", param->open.bd_addr[0],
                         param->open.bd_addr[1], param->open.bd_addr[2], param->open.bd_addr[3], param->open.bd_addr[4],
                         param->open.bd_addr[5]);
                bt_app_task_start_up();
                ESP_LOGI(TAG, "making self non-discoverable and non-connectable.");
                esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "open failed!");
        }
        break;
    case ESP_HIDD_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_CLOSE_EVT");
        if (param->close.status == ESP_HIDD_SUCCESS) {
            if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTING) {
                ESP_LOGI(TAG, "disconnecting...");
            } else if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "disconnected!");
                bt_app_task_shut_down();
                ESP_LOGI(TAG, "making self discoverable and connectable again.");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "close failed!");
        }
        break;
    case ESP_HIDD_SEND_REPORT_EVT:
        if (param->send_report.status == ESP_HIDD_SUCCESS) {
            ESP_LOGI(TAG, "ESP_HIDD_SEND_REPORT_EVT id:0x%02x, type:%d", param->send_report.report_id,
                     param->send_report.report_type);
        } else {
            ESP_LOGE(TAG, "ESP_HIDD_SEND_REPORT_EVT id:0x%02x, type:%d, status:%d, reason:%d",
                     param->send_report.report_id, param->send_report.report_type, param->send_report.status,
                     param->send_report.reason);
        }
        break;
    case ESP_HIDD_REPORT_ERR_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_REPORT_ERR_EVT");
        break;
    case ESP_HIDD_GET_REPORT_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_GET_REPORT_EVT id:0x%02x, type:%d, size:%d", param->get_report.report_id,
                 param->get_report.report_type, param->get_report.buffer_size);
        
        if (param->get_report.report_id == 2) {
            uint8_t tempBuffer[2] = {0x03, 0x00};
            esp_bt_hid_device_send_report(param->get_report.report_type, 2, 2, tempBuffer);
            ESP_LOGI(TAG, "ANSWERED 2");
            break;
        }
        
        if (param->get_report.report_id == 6) {
            esp_bt_hid_device_send_report(param->get_report.report_type, 6, blob_len, blob);
            //uint8_t tempBuffer[2] = {0xFF, 0xCE};
            //esp_bt_hid_device_send_report(param->get_report.report_type, 6, 2, tempBuffer);
            ESP_LOGI(TAG, "ANSWERED 6");
            break;
        }
        
        if (param->get_report.report_id == 7) {
			ESP_LOGI(TAG, "ANSWERING 7");
            uint8_t tempBuffer[2] = {0x03, 0x00};
            esp_bt_hid_device_send_report(param->get_report.report_type, 7, 1, tempBuffer);
            ESP_LOGI(TAG, "ANSWERED 7");
            break;
        }
        
        if (param->get_report.report_id == 3) {
			ESP_LOGI(TAG, "ANSWERING 3");
            uint8_t tempBuffer[2] = {0x03, 0x00};
            esp_bt_hid_device_send_report(param->get_report.report_type, 3, 2, tempBuffer);
            ESP_LOGI(TAG, "ANSWERED 3");
            break;
        }
        
        if (param->get_report.report_id == 5) {
			ESP_LOGI(TAG, "ANSWERING 5");
            uint8_t tempBuffer[2] = {0x03, 0x00};
            esp_bt_hid_device_send_report(param->get_report.report_type, 5, 2, tempBuffer);
            ESP_LOGI(TAG, "ANSWERED 5");
            break;
        }
        
		ESP_LOGI(TAG, "UNKNOWN COMMAND??");
		uint8_t tempBuffer = 0;
		esp_bt_hid_device_send_report(param->get_report.report_type, 0, 0, &tempBuffer);
		
        
        break;
    case ESP_HIDD_SET_REPORT_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_SET_REPORT_EVT");
        break;
    case ESP_HIDD_SET_PROTOCOL_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_SET_PROTOCOL_EVT");
        if (param->set_protocol.protocol_mode == ESP_HIDD_BOOT_MODE) {
            ESP_LOGI(TAG, "  - boot protocol");
        } else if (param->set_protocol.protocol_mode == ESP_HIDD_REPORT_MODE) {
            ESP_LOGI(TAG, "  - report protocol");
        }
        break;
    case ESP_HIDD_INTR_DATA_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_INTR_DATA_EVT");
        break;
    case ESP_HIDD_VC_UNPLUG_EVT:
        ESP_LOGI(TAG, "ESP_HIDD_VC_UNPLUG_EVT");
        if (param->vc_unplug.status == ESP_HIDD_SUCCESS) {
            if (param->close.conn_status == ESP_HIDD_CONN_STATE_DISCONNECTED) {
                ESP_LOGI(TAG, "disconnected!");
                bt_app_task_shut_down();
                ESP_LOGI(TAG, "making self discoverable and connectable again.");
                esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            } else {
                ESP_LOGE(TAG, "unknown connection status");
            }
        } else {
            ESP_LOGE(TAG, "close failed!");
        }
        break;
    default:
        break;
    }
}

void setupBluetooth(){
	const char *TAG = "setupBluetooth";
	
	ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    //ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BTDM));
    
    touch_mutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "starting touch...");
    
    
    xTaskCreate(touch_move_task, "touch_move_task", 32 * 1024, NULL, configMAX_PRIORITIES - 1, &touch_task_hdl);
    
    
	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(TAG, "initialize controller failed: %s\n", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_BTDM)) != ESP_OK) {
        ESP_LOGE(TAG, "enable controller failed: %s\n", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(TAG, "initialize bluedroid failed: %s\n", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(TAG, "enable bluedroid failed: %s\n", esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(TAG, "gap register failed: %s\n", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "setting device name");
    esp_bt_gap_set_device_name("Precision't TouchPad");
    
    ESP_LOGI(TAG, "setting cod major, peripheral");
    esp_bt_cod_t cod;
    cod.major = ESP_BT_COD_MAJOR_DEV_PERIPHERAL;
    esp_bt_gap_set_cod(cod, ESP_BT_SET_COD_MAJOR_MINOR);

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    
    app_param.name = "Touchpad";
    app_param.description = "yes???";
    app_param.provider = "ohasanov";
    app_param.subclass = ESP_HID_CLASS_MIC;
    app_param.desc_list = hid_touch_descriptor;
    app_param.desc_list_len = hid_touch_descriptor_len;
        
        
    ESP_LOGI(TAG, "register hid device callback");
    esp_bt_hid_device_register_callback(esp_bt_hidd_cb);

    ESP_LOGI(TAG, "starting hid device");
    esp_bt_hid_device_init();

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    print_bt_address();
    ESP_LOGI(TAG, "exiting");
    
}

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
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
    if(esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());
    
	
    connectTouchpad();
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    printTouchpadDebug();
    
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    touchpadSwitchMode();
	
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	
	setupBluetooth();
    
    
    while(false){
		uint8_t buffer[1024];
		
		//ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, buf, sizeof(buf), buffer, 39, -1));
		ESP_ERROR_CHECK(i2c_master_receive(dev_handle, buffer, 39, -1));
		
		if(buffer[0] != 0 || buffer[1] != 0 || buffer[2] != 0){
			printf("Incoming report: ");
			
			
	    	
	    	unsigned short xCoord = (buffer[4] | (buffer[5] << 8));
	    	unsigned short yCoord = (buffer[6] | (buffer[7] << 8));
	    	bool onTouchpad = (buffer[3] & 0b00000010) >> 1;
	    	bool button = (buffer[11] & 0b00000001);
	    	bool amogus = (buffer[11] & 0b00001000) >> 3;
	    	printf("X: %d, Y: %d, Removed: %s, Button: %s, Amog: %s", xCoord, yCoord, onTouchpad ? "no" : "yes", button ? "pressed" : "depressed :(", amogus ? "ussy" : "us");
	    	
	    	int initialNumberOfFingers = buffer[10];
	    	int numberOfFingers = buffer[10] - 1;
	    	while(numberOfFingers > 0){
				numberOfFingers--;
				
				printf(" - Additional Finger (%d): ", initialNumberOfFingers - numberOfFingers);
				ESP_ERROR_CHECK(i2c_master_receive(dev_handle, buffer, 39, -1));
				if(buffer[0] != 0 || buffer[1] != 0 || buffer[2] != 0){
					unsigned short xCoord = (buffer[4] | (buffer[5] << 8));
			    	unsigned short yCoord = (buffer[6] | (buffer[7] << 8));
			    	bool onTouchpad = (buffer[3] & 0b00000010) >> 1;
			    	bool button = (buffer[11] & 0b00000001);
			    	bool amogus = (buffer[11] & 0b00001000) >> 3;
					printf("X: %d, Y: %d, Removed: %s, Button: %s, Amog: %s", xCoord, yCoord, onTouchpad ? "no" : "yes", button ? "pressed" : "depressed :(", amogus ? "ussy" : "us");
				}
				else{
					printf(" - received goofy ahh data");
					break;
				}
				
			}
			
			printf("\n");
		}
		
		
		
		vTaskDelay(20 / portTICK_PERIOD_MS);
	}
	
    //printf("Restarting now.\n");
    
    //esp_restart();
}
