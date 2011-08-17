#ifndef ITG3200
#define ITG3200

/* default I2C address */
#define ITG3200_ADDR            0xD0
#define ITG3200_ADDR_ALT        0xD2

/* Registers */
#define ITG3200_REG_WHO_AM_I    0X00
#define ITG3200_REG_SMPLRT_DIV  0X15
#define ITG3200_REG_DLPF_FS     0X16
#define ITG3200_REG_INT_CFG     0X17
#define ITG3200_REG_INT_STATUS  0X1A
#define ITG3200_REG_TEMP_OUT_H  0X1B
#define ITG3200_REG_TEMP_OUT_L  0X1C
#define ITG3200_REG_GYRO_XOUT_H 0X1D
#define ITG3200_REG_GYRO_XOUT_L 0X1E
#define ITG3200_REG_GYRO_YOUT_H 0X1F
#define ITG3200_REG_GYRO_YOUT_L 0X20
#define ITG3200_REG_GYRO_ZOUT_H 0X21
#define ITG3200_REG_GYRO_ZOUT_L 0X22
#define ITG3200_REG_PWR_MGM     0X3E



#endif /* ITG3200 */
