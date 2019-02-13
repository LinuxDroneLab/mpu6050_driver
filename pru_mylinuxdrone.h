/*
 * pru_mylinuxdrone.h
 *
 *  Created on: 04 gen 2019
 *      Author: andrea
 */

#ifndef PRU_MYLINUXDRONE_H_
#define PRU_MYLINUXDRONE_H_

#define INT_P1_TO_P0            18
#define INT_P0_TO_P1            19

/*
 * Register R31 values to generate interrupt
 * R31_P0_TO_P1 : The value that when written to R31 register will
 *              generate INT_P1_TO_P0 interrupt
 */
#define R31_P0_TO_P1    (1<<5) | (INT_P0_TO_P1 - 16)
#define R31_P1_TO_P0    (1<<5) | (INT_P1_TO_P0 - 16)

/*
 * HOST_PRU0_TO_PRU1_CB : The check bit to check HOST_PRU0_TO_PRU1 interrupt
 *              Since the value of HOST_PRU0_TO_PRU1 is 1, HOST_PRU0_TO_PRU1_CB
 *              has the value 31
 */
#define HOST_PRU0_TO_PRU1_CB    31 // host channel 1

/* Address of the external peripherals
 * SHARED_MEM_ADDR : Absolute local address of the 12 KB shared RAM that will be
 *              used to communicate sampling configuration data between the two
 *              PRUs
 */
#define SHARED_MEM_ADDR 0x00010000

/*
 * Scratch Pad Bank IDs
 * The scratch pad inside the PRU-ICSS has 3 banks. Each bank has different
 * ID numbers.
 * SP_BANK_0 : ID number for scratch pad bank 0
 * SP_BANK_1 : ID number for scratch pad bank 1
 * SP_BANK_2 : ID number for scratch pad bank 2
 */
#define SP_BANK_0       10
#define SP_BANK_1       11
#define SP_BANK_2       12

enum message_types {
    MPU_DATA_MSG_TYPE = 0,
    COMPASS_DATA_MSG_TYPE,
    BAROMETER_DATA_MSG_TYPE,
    MPU_CONFIG_MSG_TYPE,
    MOTORS_DATA_MSG_TYPE,
    RC_DATA_MSG_TYPE,
    MPU_ENABLE_MSG_TYPE,
    COMPASS_ENABLE_MSG_TYPE,
    BAROMETER_ENABLE_MSG_TYPE,
    MOTORS_ENABLE_MSG_TYPE,
    MPU_DISABLE_MSG_TYPE,
    COMPASS_DISABLE_MSG_TYPE,
    BAROMETER_DISABLE_MSG_TYPE,
    MOTORS_DISABLE_MSG_TYPE,
    RC_ENABLE_MSG_TYPE,
    RC_DISABLE_MSG_TYPE
};

typedef struct
{
    uint32_t message_type;
    union
    {
        struct {
            uint32_t throttle;
            uint32_t yaw;
            uint32_t pitch;
            uint32_t roll;
            uint32_t aux1;
            uint32_t aux2;
            uint32_t aux3;
            uint32_t aux4;
        } rc;
        struct
        {
            uint16_t m[4];
        } motors;
        struct
        {
            int16_t accel[3];
            int16_t gyro[3];
        } mpu_accel_gyro_vect;
        struct
        {
            int16_t value[6];
        } mpu_accel_gyro_single_vect;
        struct
        {
            int16_t ax;
            int16_t ay;
            int16_t az;
            int16_t gx;
            int16_t gy;
            int16_t gz;
        } mpu_accel_gyro;
        struct
        {
            uint8_t gyro_scale;
            uint8_t accel_scale;
            uint16_t frequency_hz;
            uint16_t gyro_offset[3];
            uint16_t accel_offset[3];
        } MpuConfMessage;
    };
} PrbMessageType;

#define DATA_SIZE 44

#endif /* PRU_MYLINUXDRONE_H_ */
