#pragma once

#include "globals.h"
#include "Wire.h"

#include "esp_common.h"



#if MPU_BOLDER
#include "MPU9250.h"
extern MPU9250 IMU;
#elif MPU_HIDEAKITAI
#include "MPU9250.h"
#include "eeprom_utils.h"
extern MPU9250 mpu;
#elif MPU_SPARKFUN
#include "SparkFunMPU9250-DMP.h"

#endif

// estimator
#include "altitude.h"

static int status;

// Kalman filter parameters
// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
static float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
static float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
static float beta = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute beta
static float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

//compute deviation from self test calibration
// https://github.com/google/periph/blob/master/experimental/devices/mpu9250/mpu9250.go

// need to copy selftest and get results out of it to get std deviation for gyro and accel


extern AltitudeEstimator g_altitude;
// bias matrix used later on the altitude estimator
extern float g_gyroBias[3];
extern float g_accBias[3];
static float g_gyro[3] = {0.0 , 0.0, 0.0};
static float g_accel[3] = {0.0 , 0.0, 0.0};


typedef struct {
    float q[4];
    float yaw;
    float pitch;
    float roll;
    portMUX_TYPE mpu_latest_mutex;
} imu_raw_t; 


#define TAG __file__

void wakeUp() {
  Serial.println("Awake!");
}

float correct_magnetic_declination( float yaw, float magnetic_declination)
{
    yaw *= 180.0f / PI;
    yaw += magnetic_declination;
    if (yaw >= +180.f)
        yaw -= 360.f;
    else if (yaw < -180.f)
        yaw += 360.f;

    return yaw;
}

#if MPU_BOLDER

void getIMUBolderInterrupt(){ 
  // read the sensor
  IMU.readSensor();
  // display the data
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(),6);
}


void setup_interrupt_wom()
{
    // enabling wake on motion low power mode with a threshold of 400 mg and
    // an accelerometer data rate of 15.63 Hz. 
    IMU.enableWakeOnMotion(400, MPU9250::LP_ACCEL_ODR_15_63HZ);
    pinMode(WAKE_ON_MOTION_INTERRUPT_PIN,INPUT);
    attachInterrupt(WAKE_ON_MOTION_INTERRUPT_PIN, wakeUp ,RISING);
}
#endif




#if MPU_SPARKFUN
const signed char orientationDefault[9] = { 0, 1, 0, 0, 0, 1, 1, 0, 0 };

void setup_mpu9265(MPU9250_DMP* imu)
#else
void setup_mpu9265()
#endif
{
#if MPU_BOLDER
    // espDelay(2000);
    // IMU.setMagneticDeclination(NELSON_MAGNETIC_DECLINATION);

    // espDelay(5000);

    // start communication with IMU 
    if(!(status = IMU.begin()))
    {
        // don't do anything more:
        bool ok = false;
        uint count = 10;
        uint i = 0;
        while (!ok && i++ < count)
        {
            if (!(status = IMU.begin()))
            {
                ESP_LOGE(TAG, "IMU not detected.");
            }
            else
            {
                ok = true;
                Serial.println("IMU detection success!");
                break;
            }
            espDelay(2000);
        }
    }

    if (status < 0) {
        ESP_LOGE(TAG, "IMU initialization unsuccessful");
        ESP_LOGE(TAG, "Check IMU wiring or try cycling power");
        ESP_LOGE(TAG, "Status: %d", status);
        //TODO print to screen.
        while(1) {}
    }

    Serial.println("IMU detection success!");

    // setting the accelerometer full scale range to +/-8G 
    status = IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
    status = IMU.enableDataReadyInterrupt();

    // setting the gyroscope full scale range to +/-500 deg/s
    status = IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
    // setting DLPF bandwidth to 20 Hz
    status = IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
    // setting SRD to 19 for a 50 Hz update rate
    status = IMU.setSrd(19);

    if (status < 0) {
        Serial.println("IMU initialization unsuccessful");
        ESP_LOGE(TAG, "Check IMU wiring or try cycling power");
        ESP_LOGE(TAG, "Status: %d", status);
        //TODO print to screen.
        while(1) {}
    }
    // IMU.enableDataReadyInterrupt();
    // IMU.calibrateGyro();
    // attaching the interrupt to microcontroller pin 1
    // pinMode(IMU_INT,INPUT);
    // attachInterrupt(IMU_INT, getIMUBolderInterrupt, RISING);
#elif MPU_HIDEAKITAI
    // Wire.begin(SDA_2, SCL_2);

    espDelay(2000);
    mpu.setup(spi_IMU);

    espDelay(5000);

    // calibrate anytime you want to
    mpu.calibrateAccelGyro();
    g_gyroBias[0] = mpu.getGyroBias(0);
    g_gyroBias[1] = mpu.getGyroBias(1);
    g_gyroBias[2] = mpu.getGyroBias(2);

    g_accBias[0] = mpu.getAccBias(0);
    g_accBias[1] = mpu.getAccBias(1);
    g_accBias[2] = mpu.getAccBias(2);

    mpu.setMagneticDeclination(NELSON_MAGNETIC_DECLINATION);
    mpu.calibrateMag();

     // save to eeprom
    // saveCalibration();

    // load from eeprom
    // loadCalibration();


    mpu.printCalibration();
#endif

#if MPU_SPARKFUN
    
    pinMode(IMU_INT_PIN, INPUT_PULLUP);
    if (imu->begin() != INV_SUCCESS) {
    while (1) {
        Serial.println("Unable to communicate with MPU-9250");
        Serial.println("Check connections, and try again.");
        Serial.println();
        delay(5000);
    }
    }

    imu->enableInterrupt();
    imu->setIntLevel(INT_ACTIVE_LOW);
    imu->setIntLatched(INT_LATCHED);

    imu->dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
                DMP_FEATURE_GYRO_CAL, // Use gyro calibration
                10); // Set DMP FIFO rate to 10 Hz
    imu->dmpSetOrientation(orientationDefault);

#endif
}


void imuRead(float* gyro, float* accel)
{
    static float _ax, _ay, _az, _gx, _gy, _gz;
#if MPU_HIDEAKITAI
    mpu.update();
    // mpu.print();

    Serial.print("x-roll ");
    Serial.print(mpu.getRoll()); Serial.print(" ");

    // Serial.print("pitch (y-right (east))    : ");
    Serial.print("y-pitch ");
    Serial.print(mpu.getPitch()); Serial.print(" ");

    // Serial.print("yaw   (z-down (down))     : ");
    Serial.print("z-yaw ");
    Serial.print(mpu.getYaw()); Serial.print(" ");
    Serial.println(" ");


    _ax = mpu.getAcc(X);
    _ay = mpu.getAcc(Y);
    _az = mpu.getAcc(Z);

    _gx = mpu.getGyro(X);
    _gy = mpu.getGyro(Y);
    _gz = mpu.getGyro(Z);

    // Negate to support board orientation
    // _ax = -_ax;
    // _gy = -_gy;
    // _gz = -_gz;

    // Copy gyro values back out in rad/sec
    g_gyro[0] = _gx * DEG_TO_RAD;
    g_gyro[1] = _gy * DEG_TO_RAD;
    g_gyro[2] = _gz * DEG_TO_RAD;
    // and acceleration values
    g_accel[0] = _ax;
    g_accel[1] = _ay;
    g_accel[2] = _az;

#endif
}


void mpu_task(void * param)
{


#if HIDEAKITAI
    static uint32_t prev_ms = millis();

   
    if ((millis() - prev_ms) > 16)
    {
       imuRead(g_gyro, g_accel);


        prev_ms = millis();
    }

#endif

#if MPU_BOLDER

    static uint32_t prev_ms = millis();
    if ((millis() - prev_ms) > 20)
    {
                // read the sensor
        IMU.readSensor();

        // display the data
        Serial.print(IMU.getAccelX_mss(),6);
        Serial.print("\t");
        Serial.print(IMU.getAccelY_mss(),6);
        Serial.print("\t");
        Serial.print(IMU.getAccelZ_mss(),6);
        Serial.print("\t");
        Serial.print(IMU.getGyroX_rads(),6);
        Serial.print("\t");
        Serial.print(IMU.getGyroY_rads(),6);
        Serial.print("\t");
        Serial.print(IMU.getGyroZ_rads(),6);
        Serial.print("\t");
        Serial.print(IMU.getMagX_uT(),6);
        Serial.print("\t");
        Serial.print(IMU.getMagY_uT(),6);
        Serial.print("\t");
        Serial.print(IMU.getMagZ_uT(),6);
        Serial.print("\t");
        Serial.println(IMU.getTemperature_C(),6);

        prev_ms = millis();
    }

#endif
}


