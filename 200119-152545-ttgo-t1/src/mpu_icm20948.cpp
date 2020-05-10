#include "mpu_icm20948.hpp"

FusionBias MPU_20948::fusionBias = FusionBias();
FusionAhrs MPU_20948::fusionAhrs = FusionAhrs();
FusionVector3 MPU_20948::velocity = FusionVector3();
FusionVector3 MPU_20948::position = FusionVector3();
ICM_20948_I2C MPU_20948::myICM = ICM_20948_I2C();
portMUX_TYPE MPU_20948::mpu_latest_mutex = portMUX_INITIALIZER_UNLOCKED;