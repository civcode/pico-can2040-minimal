#include <stdio.h>
#include "pico/stdlib.h"
#include "can2040.h"
#include "RP2040.h"
#include <string.h>

#define CAN_GPIO_RX 10
#define CAN_GPIO_TX 11

static struct can2040 cbus;

static void can2040_cb(struct can2040 *cd, uint32_t notify, struct can2040_msg *msg) {

    // Add message processing code here...
    switch (notify) {
        case CAN2040_NOTIFY_RX:
            printf("Received a CAN message!\n");
            break;
        case CAN2040_NOTIFY_TX:
            printf("Transmitted a CAN message!\n");
            break;
        case CAN2040_NOTIFY_ERROR:
            printf("Messages may have been lost\n");

    }
}

static void PIOx_IRQHandler(void) {
    can2040_pio_irq_handler(&cbus);
}

void canbus_setup(void) 
{
    uint32_t pio_num = 0;
    uint32_t sys_clock = 125E6;
    uint32_t bitrate = 250E3;
    uint32_t gpio_rx = CAN_GPIO_RX;
    uint32_t gpio_tx = CAN_GPIO_TX;

    // Setup canbus
    can2040_setup(&cbus, pio_num);
    can2040_callback_config(&cbus, can2040_cb);

    // Enable irqs
    irq_set_exclusive_handler(PIO0_IRQ_0_IRQn, PIOx_IRQHandler);
    NVIC_SetPriority(PIO0_IRQ_0_IRQn, 1);
    NVIC_EnableIRQ(PIO0_IRQ_0_IRQn);

    // Start canbus
    can2040_start(&cbus, sys_clock, bitrate, gpio_rx, gpio_tx);
}

void can_transmit() {
    static uint8_t val;
    //uint8_t data[8] = {42,0,0,0,0,0,0,0};
    uint8_t data[8] = {0};

    data[0] = val++;
    struct can2040_msg msg;
    
    // struct can2040_msg msg = {.id = 0x18FFA0f9 | CAN2040_ID_EFF,
    //                           .dlc = 8};

    msg.id = 0x18FFA0f9 | CAN2040_ID_EFF;
    msg.dlc = 8;
    memcpy(msg.data, data, 8);

    int ret = can2040_transmit(&cbus, &msg);

    printf("ret = %d\n", ret);
}

int main() {

    const uint led_pin = 25;

    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    // Initialize chosen serial port
    stdio_init_all();

    // Start can2040
    canbus_setup();

    // Loop forever
    size_t cnt = 0;
    while (true) {

        // Blink LED
        printf("%d Blinking!\n", cnt++);
        gpio_put(led_pin, true);
        sleep_ms(1000);
        gpio_put(led_pin, false);
        sleep_ms(1000);
        can_transmit();
    }
}


















