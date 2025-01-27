#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_nimble_hci.h>
#include <driver/gpio.h>
#include <driver/i2c.h>

#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <host/ble_hs.h>
#include <host/util/util.h>

#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#include <esp_system.h>
#include <stdlib.h>

// ESP-IDF
// BLE documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/bluetooth/nimble/index.html
// BLE Heart Rate Measurement Example: https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/nimble/blehr

// Heart rate Sensor Configuration

// BLE specification: Assigned Numbers
// link: https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Assigned_Numbers/out/en/Assigned_Numbers.pdf?v=1736689131263
//
// This is a regularly updated document listing assigned numbers, codes, and
// identifiers in the Bluetooth specifications.

// BLE specification: GATT Specification Supplement 5
// link: https://www.bluetooth.org/DocMan/handlers/DownloadDoc.ashx?doc_id=524815
//
// This specification contains the normative definitions for all GATT characteristics and
// characteristic descriptors, with the exception of those defined in the Bluetooth Core Specification
// or in Bluetooth Service specifications.

#define GATT_HRS_UUID 0x180D
#define GATT_HRS_MEASUREMENT_UUID 0x2A37
#define GATT_HRS_BODY_SENSOR_LOC_UUID 0x2A38
uint16_t hrs_hrm_handle;
static bool is_connected = false;
static uint16_t conn_handle;

// Variable to simulate heart beats
static uint8_t heartrate = 90;

static void wait_ms(unsigned delay)
{
    vTaskDelay(delay / portTICK_PERIOD_MS);
}

static uint8_t random_heartrate(uint8_t min, uint8_t max)
{
    return min + (rand() % (max - min + 1)); //ZADANIE 4
}
static int read_heart_rate_measurement(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    static uint8_t hrm[2];
    heartrate = random_heartrate(20, 120); //ZADANIE 4
    hrm[0] = 0b00000000; // Field flags
    hrm[1] = heartrate;  // Heart Rate Measurement Value

    // More about Mbufs -> https://mynewt.apache.org/latest/os/core_os/mbuf/mbuf.html
    os_mbuf_append(ctxt->om, &hrm, sizeof(hrm));

    return 0;
}

static int read_body_sensor_location(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t hrm = 6; //ZADANIE 3 , hrm = 6 to FOOT 

    os_mbuf_append(ctxt->om, &hrm, sizeof(hrm)); // ZADANIE 3 rozszerzamy bufor bo zmieniliśmy hrm 
    // return BLE_ATT_ERR_INSUFFICIENT_RES;
    return 0;
}

/* This function simulates heart beat and notifies it to the client */
// static void
// blehr_tx_hrate(TimerHandle_t ev)
// {
//     static uint8_t hrm[2];
//     int rc;
//     struct os_mbuf *om;

//     // if (!notify_state) {
//     //     blehr_tx_hrate_stop();
//     //     heartrate = 90;
//     //     return;
//     // }

//     hrm[0] = 0x06; /* contact of a sensor */
//     hrm[1] = random_heartrate(20,120); /* storing dummy data */

//     om = ble_hs_mbuf_from_flat(hrm, sizeof(hrm));
//     rc = ble_gatts_notify_custom(1, hrs_hrm_handle, om);

//     assert(rc == 0);

//     //blehr_tx_hrate_reset();
// }


static const struct ble_gatt_svc_def kBleServices[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(GATT_HRS_UUID),
     .characteristics = (struct ble_gatt_chr_def[]){
         {
             .uuid = BLE_UUID16_DECLARE(GATT_HRS_MEASUREMENT_UUID),
             .access_cb = read_heart_rate_measurement,
             //.flags = BLE_GATT_CHR_F_READ,
             .val_handle = &hrs_hrm_handle,
             .flags = BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_READ, //ZADANIE 5 zamiast READ jest NOTIFY
         },
         {
             .uuid = BLE_UUID16_DECLARE(GATT_HRS_BODY_SENSOR_LOC_UUID),
             .access_cb = read_body_sensor_location,
             .flags = BLE_GATT_CHR_F_READ,
         },
         {
             0, // no more characteristics
         },
     }},
    {
        0, // no more services
    },
};
static uint8_t s_led_state = 0;
static void blink_led(void)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(4, s_led_state);
}

static void configure_led(void)
{
    gpio_reset_pin(4);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(4, 2);
}

static void start_advertisement(void);

static int on_ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI("BLE GAP Event", "Connected");
        s_led_state = !s_led_state;
        blink_led();
        is_connected = !is_connected;
        conn_handle = event->connect.conn_handle;
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI("BLE GAP Event", "Disconnected");
        start_advertisement();
        s_led_state = !s_led_state;
        blink_led();
        is_connected = !is_connected;
        break;

    default:
        ESP_LOGI("BLE GAP Event", "Type: 0x%02X", event->type);
        break;
    }

    return 0;
}

static void start_advertisement(void)
{
    struct ble_gap_adv_params adv_parameters;
    memset(&adv_parameters, 0, sizeof(adv_parameters));

    adv_parameters.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_parameters.disc_mode = BLE_GAP_DISC_MODE_GEN;

    if (ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                          &adv_parameters,
                          on_ble_gap_event, NULL) != 0)
    {
        ESP_LOGE("BLE", "Can't start Advertisement");
        return;
    }

    ESP_LOGI("BLE", "Advertisement started...");
}

static void set_device_name(const char *device_name)
{
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));

    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;

    if (ble_gap_adv_set_fields(&fields) != 0)
    {
        ESP_LOGE("BLE", "Can't configure BLE advertisement fields");
        return;
    }

    ble_svc_gap_device_name_set(device_name);
}

static void start_ble_service(void *param)
{
    ESP_LOGI("BLE task", "BLE Host Task Started");

    nimble_port_run();
    nimble_port_freertos_deinit();
}

void app_main(void)
{
    // Initialize Non-Volatile Memory
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGI("NVS", "Initializing NVS...");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    // Initialize BLE peripheral
    nimble_port_init();
    esp_nimble_hci_init();

    // Initialize BLE library (nimble)
    ble_svc_gap_init();
    ble_svc_gatt_init();

    // Configure BLE library (nimble)
    int rc = ble_gatts_count_cfg(kBleServices);
    if (rc != 0)
    {
        ESP_LOGE("BLE GATT", "Service registration failed");
        goto error;
    }

    // Register all services
    rc = ble_gatts_add_svcs(kBleServices);
    if (rc != 0)
    {
        ESP_LOGE("BLE GATT", "Service registration failed");
        goto error;
    }

    configure_led();

    // Start BLE stack
    nimble_port_freertos_init(start_ble_service);

    // Start BLE device
    set_device_name("MatOs");
    start_advertisement();

error:            
static uint8_t hrm[2];  
//int rc;
struct os_mbuf *om;
// Inside your while loop in app_main
while (1)
{
    wait_ms(1000);
    if (is_connected)
    {
        heartrate = random_heartrate(20, 120);  // Update heart rate before sending notification

        hrm[0] = 0x06; /* contact of a sensor */
        hrm[1] = heartrate; /* storing dummy data */

        om = ble_hs_mbuf_from_flat(hrm, sizeof(hrm));
        rc = ble_gatts_notify_custom(conn_handle, hrs_hrm_handle, om);
        
        if (rc == 0) {
            printf("Halko\n");
        } else {
            printf("rc = %d\n", rc);
        }

        assert(rc == 0);
    }
};

}
