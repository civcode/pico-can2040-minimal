#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"

#include "can2040.h"
#include "RP2040.h"

#define CAN_GPIO_RX 10
#define CAN_GPIO_TX 11

// #define MPU6050_GPIO_SDA 4
// #define MPU6050_GPIO_SCL 5


static int mpu6050_addr = 0x68;

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

// void can_transmit() {
//     static uint8_t val;
//     //uint8_t data[8] = {42,0,0,0,0,0,0,0};
//     uint8_t data[8] = {0};

//     data[0] = val++;
//     struct can2040_msg msg;
    
//     // struct can2040_msg msg = {.id = 0x18FFA0f9 | CAN2040_ID_EFF,
//     //                           .dlc = 8};

//     msg.id = 0x18FFA0f9 | CAN2040_ID_EFF;
//     msg.dlc = 8;
//     memcpy(msg.data, data, 8);

//     int ret = can2040_transmit(&cbus, &msg);

//     printf("ret = %d\n", ret);
// }

void can_transmit(struct can2040_msg msg) {

}

static void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, mpu6050_addr, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    printf("mpu6050_read_raw\n");
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, mpu6050_addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, mpu6050_addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, mpu6050_addr, &val, 1, true);
    i2c_read_blocking(i2c_default, mpu6050_addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, mpu6050_addr, &val, 1, true);
    i2c_read_blocking(i2c_default, mpu6050_addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

static void UINT16_TO_BUFFER(uint8_t *buffer, uint16_t value) {
    // buffer[idx+0] = (kLogicalAddressClient_ >> 8) & 0xFF;
    // buffer[idx+1] = (kLogicalAddressClient_ >> 0) & 0xFF;
    *(buffer+0) = (value >> 8) & 0xFF;
    *(buffer+1) = (value >> 0) & 0xFF;
}

int main() {

    const uint led_pin = 25;

    // Initialize LED pin
    gpio_init(led_pin);
    gpio_set_dir(led_pin, GPIO_OUT);

    // Initialize I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    // Reset MPU6050 to start measuremets
    mpu6050_reset();


    // Initialize chosen serial port
    stdio_init_all();

    // Start can2040
    canbus_setup();

    int16_t acceleration[3], gyro[3], temp;
    struct can2040_msg msg;

    // Loop forever
    size_t cnt = 0;
    while (true) {

        // Blink LED
        gpio_put(led_pin, true);
        sleep_ms(50);
        gpio_put(led_pin, false);
        sleep_ms(50);

        mpu6050_read_raw(acceleration, gyro, &temp);
        
        printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1], acceleration[2]);
        printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
        printf("Temp. = %f\n", (temp / 340.0) + 36.53);

        /* Transmitting Accelerometer data */
        msg.id = 0x18FFA0f9 | CAN2040_ID_EFF;
        msg.dlc = 8;

        memset(msg.data, 0xFF, sizeof(msg.data));

        uint16_t offset = 32767;
        UINT16_TO_BUFFER(msg.data+0, acceleration[0] + offset); 
        UINT16_TO_BUFFER(msg.data+2, acceleration[1] + offset); 
        UINT16_TO_BUFFER(msg.data+4, acceleration[2] + offset); 

        int ret = can2040_transmit(&cbus, &msg);

        /* Transmitting Gyroscope data */
        msg.id = 0x18FFA1f9 | CAN2040_ID_EFF;
        msg.dlc = 8;

        memset(msg.data, 0xFF, sizeof(msg.data));

        UINT16_TO_BUFFER(msg.data+0, gyro[0] + offset); 
        UINT16_TO_BUFFER(msg.data+2, gyro[1] + offset); 
        UINT16_TO_BUFFER(msg.data+4, gyro[2] + offset); 

        ret = can2040_transmit(&cbus, &msg);

        /* Transmitting Termperature */
        /* SAE J1939 temperature: -40 deg C offset, range -40 deg C to +215 deg C */        
        // msg.id = 0x18FFA2f9 | CAN2040_ID_EFF;
        // msg.dlc = 8;

        // memset(msg.data, 0xFF, sizeof(msg.data));

        // UINT16_TO_BUFFER(msg.data+0, (temp / 340.0) + 36.53 + offset); 

        // ret = can2040_transmit(&cbus, &msg);
    }
}


















