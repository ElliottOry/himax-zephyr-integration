
#include <zephyr/device.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/printk.h>

#define IMAGE_SIZE 2500
#define CHUNK_SIZE 244

static uint8_t image_data[IMAGE_SIZE];
static uint16_t sent_offset = 0;
// static struct k_work_delayable send_work;



static struct bt_conn *current_conn;
static bool notify_enabled = false;

// Custom UUIDs
static struct bt_uuid_128 service_uuid = BT_UUID_INIT_128(
    0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0xaa, 0xbb);

static struct bt_uuid_128 char_uuid = BT_UUID_INIT_128(
    0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
    0x78, 0x56, 0x34, 0x12, 0x00, 0x00, 0xaa, 0xbb);

    // Forward declaration
static void send_image_chunk(struct k_work *work);


K_WORK_DELAYABLE_DEFINE(send_work, send_image_chunk);
// Notification callback
static void on_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
    notify_enabled = (value == BT_GATT_CCC_NOTIFY);
    if (notify_enabled) {
        sent_offset = 0;
        k_work_schedule(&send_work, K_NO_WAIT);
    }
}

// Attribute table
BT_GATT_SERVICE_DEFINE(custom_svc,
    BT_GATT_PRIMARY_SERVICE(&service_uuid),
    BT_GATT_CHARACTERISTIC(&char_uuid.uuid,
                           BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_NONE,
                           NULL, NULL, NULL),
    BT_GATT_CCC(on_ccc_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

// Connection callbacks
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (!err) {
        current_conn = bt_conn_ref(conn);
        printk("Connected\n");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason 0x%02x)\n", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    notify_enabled = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

// Send chunked data over notification
static void send_image_chunk(struct k_work *work)
{
    if (!notify_enabled || !current_conn) return;

    size_t remaining = IMAGE_SIZE - sent_offset;
    size_t len = remaining > CHUNK_SIZE ? CHUNK_SIZE : remaining;

    int err = bt_gatt_notify(NULL, &custom_svc.attrs[1], &image_data[sent_offset], len);
    if (err == 0) {
        sent_offset += len;
        if (sent_offset >= IMAGE_SIZE) {
            sent_offset = 0; // Restart stream
        }
        k_work_schedule(&send_work, K_MSEC(10)); // Adjust speed here
    } else {
        printk("Notify failed: %d\n", err);
    }
}


void main(void)
{
    int err;

    printk("Starting BLE image streamer...\n");

    // Fill dummy image data
    for (int i = 0; i < IMAGE_SIZE; i++) {
        image_data[i] = i % 256;
    }

    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, NULL, 0, NULL, 0);
    if (err) {
        printk("advertising failed: %d\n", err);
    } else {
        printk("advertising successful \n");
    }

    printk("Advertising started\n");
}
