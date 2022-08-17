#include <application.h>

// LED instance
twr_led_t led;

// Button instance
twr_button_t button;

// Write and read FIFOs for RS-485 Module async tranfers
twr_fifo_t write_fifo;
twr_fifo_t read_fifo;
uint8_t write_fifo_buffer[64];
uint8_t read_fifo_buffer[64];

void rs485_relay_set(uint64_t *id, const char *topic, void *value, void *param);
void relay_send_command(uint8_t address, uint8_t relay, uint8_t command, uint8_t delay);

#define RELAY_COMMAND_OPEN 0x01
#define RELAY_COMMAND_CLOSE 0x02
#define RELAY_COMMAND_TOGGLE 0x03
#define RELAY_COMMAND_LATCH 0x04
#define RELAY_COMMAND_MOMENTARY 0x05

static const twr_radio_sub_t subs[] = {
    // state/set
    {"rs485-relay/q1/state/set", TWR_RADIO_SUB_PT_BOOL, rs485_relay_set, (void *) 1},
    {"rs485-relay/q2/state/set", TWR_RADIO_SUB_PT_BOOL, rs485_relay_set, (void *) 2},
    {"rs485-relay/q3/state/set", TWR_RADIO_SUB_PT_BOOL, rs485_relay_set, (void *) 3},
    {"rs485-relay/q4/state/set", TWR_RADIO_SUB_PT_BOOL, rs485_relay_set, (void *) 4},
    {"rs485-relay/q5/state/set", TWR_RADIO_SUB_PT_BOOL, rs485_relay_set, (void *) 5},
    {"rs485-relay/q6/state/set", TWR_RADIO_SUB_PT_BOOL, rs485_relay_set, (void *) 6},
    {"rs485-relay/q7/state/set", TWR_RADIO_SUB_PT_BOOL, rs485_relay_set, (void *) 7},
    {"rs485-relay/q8/state/set", TWR_RADIO_SUB_PT_BOOL, rs485_relay_set, (void *) 8},
};

void rs485_relay_set(uint64_t *id, const char *topic, void *value, void *param)
{
    twr_led_pulse(&led, 30);

    uint8_t channel = (uint32_t) param;
    uint8_t command = (*(bool*)value) ? RELAY_COMMAND_OPEN : RELAY_COMMAND_CLOSE;

    twr_log_debug("ch: %d, value: %ld, command %d", channel, *((uint32_t*)value), command);

    relay_send_command(0x01, channel, command, 0);
}

uint16_t modbus_crc(uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)data[pos];

        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void relay_send_command(uint8_t address, uint8_t relay, uint8_t command, uint8_t delay)
{
    uint8_t packet[8];

    packet[0] = address;   // modbus address
    packet[1] = 0x06;      // Function 0x06
    packet[2] = 0x00;      // 16bit relay index (MODBUS address)
    packet[3] = relay;
    packet[4] = command;   // Command
    packet[5] = delay;     // Delay

    // Calculate CRC
    uint16_t crc = modbus_crc(packet, 6);
    packet[6] = crc;
    packet[7] = crc >> 8;

    twr_module_rs485_async_write(packet, sizeof(packet));
}

void button_event_handler(twr_button_t *self, twr_button_event_t event, void *event_param)
{
    static uint16_t button_event_count = 0;

    if (event == TWR_BUTTON_EVENT_CLICK)
    {
        twr_log_debug("Relay 1 toggle");
        relay_send_command(0x01, 0x01, RELAY_COMMAND_TOGGLE, 0);
        twr_led_pulse(&led, 100);

        twr_radio_pub_push_button(&button_event_count);
        button_event_count++;
    }

    if (event == TWR_BUTTON_EVENT_HOLD)
    {
        twr_log_debug("Relay 2 toggle");
        relay_send_command(0x01, 0x02, RELAY_COMMAND_TOGGLE, 0);
        twr_led_pulse(&led, 100);
    }
}

void module_rs485_event_handler(twr_module_rs485_event_t event, void *param)
{
    (void) param;

    if (event == TWR_MODULE_RS485_EVENT_VOLTAGE)
    {
        float voltage;
        twr_module_rs485_get_voltage(&voltage);
        twr_log_debug("Input voltage: %0.2f V", voltage);
    }
}

void application_init(void)
{
    // Disable sleep in case of receiving data
    twr_system_deep_sleep_disable();

    // Initialize logging
    twr_log_init(TWR_LOG_LEVEL_DUMP, TWR_LOG_TIMESTAMP_ABS);

    // Initialize LED
    twr_led_init(&led, TWR_GPIO_LED, false, false);
    twr_led_pulse(&led, 2000);

    // Initialize button
    twr_button_init(&button, TWR_GPIO_BUTTON, TWR_GPIO_PULL_DOWN, false);
    twr_button_set_hold_time(&button, 250);
    twr_button_set_event_handler(&button, button_event_handler, NULL);

    // Initialize radio
    twr_radio_init(TWR_RADIO_MODE_NODE_LISTENING);
    twr_radio_set_subs((twr_radio_sub_t *) subs, sizeof(subs)/sizeof(twr_radio_sub_t));

    // Init FIFOs
    twr_fifo_init(&write_fifo, write_fifo_buffer, sizeof(write_fifo_buffer));
    twr_fifo_init(&read_fifo, read_fifo_buffer, sizeof(read_fifo_buffer));

    // Init RS-485 Module
    twr_module_rs485_init();
    twr_module_rs485_set_event_handler(module_rs485_event_handler, NULL);
    twr_module_rs485_set_update_interval(5000);
    twr_module_rs485_set_baudrate(TWR_MODULE_RS485_BAUDRATE_9600);
    twr_module_rs485_set_async_fifo(&write_fifo, &read_fifo);

    twr_radio_pairing_request("twr-rs485-relay", FW_VERSION);

}

// Example reading task using polling
/*
void application_task(void)
{
    size_t data_size;
    static uint8_t rx_buffer[32];

    twr_module_rs485_available(&data_size);

    if (data_size)
    {
        size_t b = twr_module_rs485_read(rx_buffer, sizeof(rx_buffer), 0);

        twr_log_dump(rx_buffer, b, "RX bytes %d", b);
    }

    twr_scheduler_plan_current_from_now(500);
}
*/

