

#include <libhal-util/bit.hpp>

namespace hal::icm {

static constexpr hal::byte AK09916_ADDRESS = 0x0C;

/* Registers ICM20948 USER BANK 0*/
static constexpr hal::byte ICM20948_WHO_AM_I = 0x00;
static constexpr hal::byte ICM20948_USER_CTRL = 0x03;
static constexpr hal::byte ICM20948_LP_CONFIG = 0x05;
static constexpr hal::byte ICM20948_PWR_MGMT_1 = 0x06;
static constexpr hal::byte ICM20948_PWR_MGMT_2 = 0x07;
static constexpr hal::byte ICM20948_INT_PIN_CFG = 0x0F;
static constexpr hal::byte ICM20948_INT_ENABLE = 0x10;
static constexpr hal::byte ICM20948_INT_ENABLE_1 = 0x11;
static constexpr hal::byte ICM20948_INT_ENABLE_2 = 0x12;
static constexpr hal::byte ICM20948_INT_ENABLE_3 = 0x13;
static constexpr hal::byte ICM20948_I2C_MST_STATUS = 0x17;
static constexpr hal::byte ICM20948_INT_STATUS = 0x19;
static constexpr hal::byte ICM20948_INT_STATUS_1 = 0x1A;
static constexpr hal::byte ICM20948_INT_STATUS_2 = 0x1B;
static constexpr hal::byte ICM20948_INT_STATUS_3 = 0x1C;
static constexpr hal::byte ICM20948_DELAY_TIME_H = 0x28;
static constexpr hal::byte ICM20948_DELAY_TIME_L = 0x29;
static constexpr hal::byte ICM20948_ACCEL_OUT =
  0x2D;  // accel data registers begin
static constexpr hal::byte ICM20948_GYRO_OUT =
  0x33;  // gyro data registers begin
static constexpr hal::byte ICM20948_TEMP_OUT = 0x39;
static constexpr hal::byte ICM20948_EXT_SLV_SENS_DATA_00 = 0x3B;
static constexpr hal::byte ICM20948_EXT_SLV_SENS_DATA_01 = 0x3C;
static constexpr hal::byte ICM20948_FIFO_EN_1 = 0x66;
static constexpr hal::byte ICM20948_FIFO_EN_2 = 0x67;
static constexpr hal::byte ICM20948_FIFO_RST = 0x68;
static constexpr hal::byte ICM20948_FIFO_MODE = 0x69;
static constexpr hal::byte ICM20948_FIFO_COUNT = 0x70;
static constexpr hal::byte ICM20948_FIFO_R_W = 0x72;
static constexpr hal::byte ICM20948_DATA_RDY_STATUS = 0x74;
static constexpr hal::byte ICM20948_FIFO_CFG = 0x76;

/* Registers ICM20948 USER BANK 1*/
static constexpr hal::byte ICM20948_SELF_TEST_X_GYRO = 0x02;
static constexpr hal::byte ICM20948_SELF_TEST_Y_GYRO = 0x03;
static constexpr hal::byte ICM20948_SELF_TEST_Z_GYRO = 0x04;
static constexpr hal::byte ICM20948_SELF_TEST_X_ACCEL = 0x0E;
static constexpr hal::byte ICM20948_SELF_TEST_Y_ACCEL = 0x0F;
static constexpr hal::byte ICM20948_SELF_TEST_Z_ACCEL = 0x10;
static constexpr hal::byte ICM20948_XA_OFFS_H = 0x14;
static constexpr hal::byte ICM20948_XA_OFFS_L = 0x15;
static constexpr hal::byte ICM20948_YA_OFFS_H = 0x17;
static constexpr hal::byte ICM20948_YA_OFFS_L = 0x18;
static constexpr hal::byte ICM20948_ZA_OFFS_H = 0x1A;
static constexpr hal::byte ICM20948_ZA_OFFS_L = 0x1B;
static constexpr hal::byte ICM20948_TIMEBASE_CORR_PLL = 0x28;

/* Registers ICM20948 USER BANK 2*/
static constexpr hal::byte ICM20948_GYRO_SMPLRT_DIV = 0x00;
static constexpr hal::byte ICM20948_GYRO_CONFIG_1 = 0x01;
static constexpr hal::byte ICM20948_GYRO_CONFIG_2 = 0x02;
static constexpr hal::byte ICM20948_XG_OFFS_USRH = 0x03;
static constexpr hal::byte ICM20948_XG_OFFS_USRL = 0x04;
static constexpr hal::byte ICM20948_YG_OFFS_USRH = 0x05;
static constexpr hal::byte ICM20948_YG_OFFS_USRL = 0x06;
static constexpr hal::byte ICM20948_ZG_OFFS_USRH = 0x07;
static constexpr hal::byte ICM20948_ZG_OFFS_USRL = 0x08;
static constexpr hal::byte ICM20948_ODR_ALIGN_EN = 0x09;
static constexpr hal::byte ICM20948_ACCEL_SMPLRT_DIV_1 = 0x10;
static constexpr hal::byte ICM20948_ACCEL_SMPLRT_DIV_2 = 0x11;
static constexpr hal::byte ICM20948_ACCEL_INTEL_CTRL = 0x12;
static constexpr hal::byte ICM20948_ACCEL_WOM_THR = 0x13;
static constexpr hal::byte ICM20948_ACCEL_CONFIG = 0x14;
static constexpr hal::byte ICM20948_ACCEL_CONFIG_2 = 0x15;
static constexpr hal::byte ICM20948_FSYNC_CONFIG = 0x52;
static constexpr hal::byte ICM20948_TEMP_CONFIG = 0x53;
static constexpr hal::byte ICM20948_MOD_CTRL_USR = 0x54;

/* Registers ICM20948 USER BANK 3*/
static constexpr hal::byte ICM20948_I2C_MST_ODR_CFG = 0x00;
static constexpr hal::byte ICM20948_I2C_MST_CTRL = 0x01;
static constexpr hal::byte ICM20948_I2C_MST_DELAY_CTRL = 0x02;
static constexpr hal::byte ICM20948_I2C_SLV0_ADDR = 0x03;
static constexpr hal::byte ICM20948_I2C_SLV0_REG = 0x04;
static constexpr hal::byte ICM20948_I2C_SLV0_CTRL = 0x05;
static constexpr hal::byte ICM20948_I2C_SLV0_DO = 0x06;

/* Registers ICM20948 ALL BANKS */
static constexpr hal::byte ICM20948_REG_BANK_SEL = 0x7F;

/* Registers AK09916 */
static constexpr hal::byte AK09916_WIA_1 = 0x00;  // Who I am, Company ID
static constexpr hal::byte AK09916_WIA_2 = 0x01;  // Who I am, Device ID
static constexpr hal::byte AK09916_STATUS_1 = 0x10;
static constexpr hal::byte AK09916_HXL = 0x11;
static constexpr hal::byte AK09916_HXH = 0x12;
static constexpr hal::byte AK09916_HYL = 0x13;
static constexpr hal::byte AK09916_HYH = 0x14;
static constexpr hal::byte AK09916_HZL = 0x15;
static constexpr hal::byte AK09916_HZH = 0x16;
static constexpr hal::byte AK09916_STATUS_2 = 0x18;
static constexpr hal::byte AK09916_CNTL_2 = 0x31;
static constexpr hal::byte AK09916_CNTL_3 = 0x32;

/* Register Bits */
static constexpr hal::byte ICM20948_RESET = 0x80;
static constexpr hal::byte ICM20948_I2C_MST_EN = 0x20;
static constexpr hal::byte ICM20948_SLEEP = 0x40;
static constexpr hal::byte ICM20948_LP_EN = 0x20;
static constexpr hal::byte ICM20948_BYPASS_EN = 0x02;
static constexpr hal::byte ICM20948_GYR_EN = 0x07;
static constexpr hal::byte ICM20948_ACC_EN = 0x38;
static constexpr hal::byte ICM20948_FIFO_EN = 0x40;
static constexpr hal::byte ICM20948_INT1_ACTL = 0x80;
static constexpr hal::byte ICM20948_INT_1_LATCH_EN = 0x20;
static constexpr hal::byte ICM20948_ACTL_FSYNC = 0x08;
static constexpr hal::byte ICM20948_INT_ANYRD_2CLEAR = 0x10;
static constexpr hal::byte ICM20948_FSYNC_INT_MODE_EN = 0x06;
static constexpr hal::byte AK09916_16_BIT = 0x10;
static constexpr hal::byte AK09916_OVF = 0x08;
static constexpr hal::byte AK09916_READ = 0x80;

/* Others */
static constexpr hal::byte AK09916_WHO_AM_I_1 = 0x4809;
static constexpr hal::byte AK09916_WHO_AM_I_2 = 0x0948;
static constexpr hal::byte ICM20948_WHO_AM_I_CONTENT = 0xEA;
static constexpr float ICM20948_ROOM_TEMP_OFFSET{ 0.0 };
static constexpr float ICM20948_T_SENSITIVITY{ 333.87 };
static constexpr float AK09916_MAG_LSB{ 0.1495 };

}  // namespace hal::icm