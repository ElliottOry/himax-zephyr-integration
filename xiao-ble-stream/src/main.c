/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/sys/util.h>

// #define insect_cam_SERVICE_TYPE BT_UUID_128_ENCODE(0x2e2b8dc3, 0x06e0, 0x4f93, 0x9bb2, 0x734091c356f0)

#define BT_UUID_IMG_SERVICE_VAL \
    BT_UUID_128_ENCODE(0x6e400001, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca3e)
#define BT_UUID_IMG_RX_VAL \
    BT_UUID_128_ENCODE(0x6e400002, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca3e)
#define BT_UUID_IMG_TX_VAL \
    BT_UUID_128_ENCODE(0x6e400003, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca3e)
#define BT_UUID_IMG_INFO_VAL \
    BT_UUID_128_ENCODE(0x6e400004, 0xb5a3, 0xf393, 0xe0a9, 0xe50e24dcca3e)



/* Overhead: opcode (u8) + handle (u16) */
#define ATT_NTF_SIZE(payload_len) (1 + 2 + payload_len)

static const struct bt_uuid_128 insect_cam_service = BT_UUID_INIT_128(BT_UUID_IMG_SERVICE_VAL);

static const struct bt_uuid_128 tx_characteristic_uuid =
	BT_UUID_INIT_128(BT_UUID_IMG_TX_VAL);

static const struct bt_uuid_128 rx_characteristic_uuid =
	BT_UUID_INIT_128(BT_UUID_IMG_RX_VAL);


static const struct bt_uuid_128 img_info_characteristic_uuid =
	BT_UUID_INIT_128(BT_UUID_IMG_INFO_VAL);

static const struct bt_data adv_ad_data[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_IMG_SERVICE_VAL),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static struct bt_conn *default_conn;
static struct k_sem conn_sem;
static struct k_work advertise_work;



//Defing dummy data buffer
static uint8_t data_to_send[1000];
uint16_t ble_mtu =0;
bool stream_en = false;



void advertising_work_handler(struct k_work *work)
{
	bt_le_adv_start(BT_LE_ADV_CONN_ONE_TIME, adv_ad_data, ARRAY_SIZE(adv_ad_data), NULL, 0);
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	// ARG_UNUSED(attr);
    bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    printk("Notifications %s\n", notif_enabled ? "enabled" : "disabled");

	// bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);

	// printk("MTU Test Update: notifications %s\n", notif_enabled ? "enabled" : "disabled");
}

static ssize_t on_rx_received(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf,
                              uint16_t len,
                              uint16_t offset,
                              uint8_t flags);

static void send_large_data(struct bt_conn *conn);            

BT_GATT_SERVICE_DEFINE(
    insect_cam,
    BT_GATT_PRIMARY_SERVICE(&insect_cam_service),
    BT_GATT_CHARACTERISTIC(&tx_characteristic_uuid.uuid,(BT_GATT_CHRC_NOTIFY), (BT_GATT_PERM_NONE), NULL, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&img_info_characteristic_uuid.uuid,(BT_GATT_CHRC_NOTIFY), (BT_GATT_PERM_NONE), NULL, NULL, NULL),
    BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
    BT_GATT_CHARACTERISTIC(&rx_characteristic_uuid.uuid,(BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_WRITE), BT_GATT_PERM_WRITE, NULL, on_rx_received, NULL)
    );

#define CHUNK_SIZE CONFIG_BT_L2CAP_TX_MTU
static const struct bt_gatt_attr *notify_attr_global = NULL;


static void send_large_data(struct bt_conn *conn)
{
    if (!conn){
        printk("No active connection\n");
        return;
    }

    notify_attr_global = bt_gatt_find_by_uuid(insect_cam.attrs, 0xffff, &tx_characteristic_uuid.uuid);

    if (!notify_attr_global) {
        printk("Tx Notify attribute not found!\n");
        return;
    }

        

    // uint16_t mtu = bt_gatt_get_mtu(conn);
    
    uint16_t max_payload = ble_mtu - 3;  // ATT notification overhead

    // Filling in dummy data into the img buffer
    for (int i = 0; i < sizeof(data_to_send); i++) {
        data_to_send[i] = i & 0xFF;
    }


    uint16_t offset = 0;
    while (offset < sizeof(data_to_send)) {
        uint16_t len = MIN(max_payload, sizeof(data_to_send) - offset);
        printk("Attempting to send pkt of size [%d] and first data [%d]",len,data_to_send[offset]);
        int err = bt_gatt_notify(conn, notify_attr_global, &data_to_send[offset], len);
        if (err) {
            printk("Notify failed: %d", err);
            // return;
        }
        printk("\n");

        offset += len;
        k_sleep(K_MSEC(1));  // Throttle to avoid flooding
    }

    printk("All chunks sent\n");
}


static ssize_t on_rx_received(struct bt_conn *conn,
                              const struct bt_gatt_attr *attr,
                              const void *buf,
                              uint16_t len,
                              uint16_t offset,
                              uint8_t flags) {
    printk("RX received %d bytes -->", len);

    const uint8_t *rx_data = buf;
    for (uint8_t i = 0; i< len; i++){
        printk("%x",rx_data[i]);
    }
    printk("\n");


    if (len == 1 && rx_data[0] == 2){
        printk("Enabling Stream..\n");
        stream_en = true;
        // send_large_data(conn);
    }

    if(len == 1 && rx_data[0] == 3){ 
        printk("Disabling Stream..\n");
        stream_en = false;

    }

    return len;
}



void mtu_updated(struct bt_conn *conn, uint16_t tx, uint16_t rx)
{
	printk("Updated MTU: TX: %d RX: %d bytes\n", tx, rx);
}

static struct bt_gatt_cb gatt_callbacks = {
	.att_mtu_updated = mtu_updated,
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0) {
		return;
	}

	default_conn = bt_conn_ref(conn);
	k_sem_give(&conn_sem);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	bt_conn_unref(conn);
	default_conn = NULL;
	k_work_submit(&advertise_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

void run_peripheral_sample( uint16_t seconds)
{
	

	bool infinite = seconds == 0;

    struct bt_gatt_attr *img_info_notify_attr_global = bt_gatt_find_by_uuid(insect_cam.attrs, 0xffff, &img_info_characteristic_uuid.uuid);

    if (!img_info_notify_attr_global) {
        printk("Img Info notify attribute not found!\n");
        return;
    }


	for (int i = 0; (i < seconds) || infinite; i++) {
		if (default_conn == NULL) {
			k_sem_take(&conn_sem, K_FOREVER);
		}

		k_sleep(K_MSEC(5));
		if (default_conn == NULL) {
			printk("Skipping notification since connection is not yet established\n");
		/* Only send the notification if the UATT MTU supports the required length */
		} else {
            uint16_t current_mtu = bt_gatt_get_uatt_mtu(default_conn);
            if (current_mtu != ble_mtu){
                ble_mtu = current_mtu;
                uint8_t img_info[4] = {100,(uint8_t)ble_mtu,100,0};
                int err = bt_gatt_notify(default_conn, img_info_notify_attr_global, &img_info, 4);
                if (err) {
                    printk("Img Information Notify failed: %d\n", err);
                }
                printk("MTU setting for Tx: %d!\n",ble_mtu);
            }

            if(stream_en){
                send_large_data(default_conn);
            }
            
            if (ble_mtu < CHUNK_SIZE) {

                printk("Skipping notification since UATT MTU is not sufficient."
			       "Required: %d, Actual: %d\n",
			       CHUNK_SIZE,
			       ble_mtu);
            }
		}
	}
}

int main(void)
{
    int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	k_sem_init(&conn_sem, 0, 1);
	bt_gatt_cb_register(&gatt_callbacks);

	k_work_init(&advertise_work, advertising_work_handler);

    
	bt_le_adv_start(BT_LE_ADV_CONN_ONE_TIME, adv_ad_data, ARRAY_SIZE(adv_ad_data), NULL, 0);

	
	run_peripheral_sample(0);
	return 0;
}