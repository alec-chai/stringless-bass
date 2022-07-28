#include <Wire.h>
#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer;

#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>

//Adafruit_FXOS8700 fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
//Adafruit_FXAS21002C fxas = Adafruit_FXAS21002C(0x0021002C);

Adafruit_FXOS8700 fxos = Adafruit_FXOS8700(0x8700A, 0x8700B);
Adafruit_FXAS21002C fxas = Adafruit_FXAS21002C(0x0021002C);

Adafruit_NXPSensorFusion filter; // slowest

#define FILTER_UPDATE_RATE_HZ 100

float linx, liny, linz;// these will be the linear acceleration values with no gravity

float gyrox_off = 0.00808;
float gyroy_off = -0.00310;
float gyroz_off = 0.00662;

float gyrox_scale = 1.0;
float gyroy_scale = 1.0;
float gyroz_scale = 1.0;

float accelx_off = -0.017;
float accely_off = -0.007;
float accelz_off = -0.054;

float accelx_scale = .9865/9.81;
float accely_scale = 1.0232/9.81;
float accelz_scale = 0.9999/9.81;

float magx_off = -1.9;
float magy_off = -19.04;
float magz_off = -56.9;

float magxx = 0.975;
float magyy = 0.960;
float magzz = 1.069;

float magxy = -0.015;// note that the matrix is equal to its transpose, so only 6 indep vals
float magxz = -0.016;//most of the correction is in the offset and diag matrix elements
float magyz = +0.010;

float roll, pitch, heading;
float gx, gy, gz;
float ax, ay, az;
float mx, my, mz;
float magx, magy, magz;//these are temp values to do the matrix multiplication for mag calib
float qw, qx, qy, qz;//these are the raw quat values

float q0prime, q1prime, q2prime, q3prime;//quat values rotated to match bno055 quats
float plucksig;
float lin_accelx;

bool init_sensors(void) {
  if (!fxos.begin()) {
    Serial.println("Cannot detect fxos");
  }

  if (!fxas.begin()) {
    Serial.println("Cannot detect fxas");
  }

  
  if (!fxos.begin() || !fxas.begin()) {
    return false;
  }
  accelerometer = fxos.getAccelerometerSensor();
  gyroscope = &fxas;
  magnetometer = fxos.getMagnetometerSensor();

  return true;
}

void setup_sensors(void) {}

void fxa_fxo_setup_subs(void){
  if (!init_sensors()) {
    Serial.println("Failed to find sensors");
    while (1) delay(10);
  }

  setup_sensors();
  filter.begin(FILTER_UPDATE_RATE_HZ);
  Wire.setClock(400000); // 400KHz
}

void fxa_fxo_main_calc(){

// Reads the motion sensors, calibrates, and applies fusion filter
sensors_event_t accel, gyro, mag;
accelerometer->getEvent(&accel);
gyroscope->getEvent(&gyro);
magnetometer->getEvent(&mag);

magx = mag.magnetic.x - magx_off;
magy = mag.magnetic.y - magy_off;
magz = mag.magnetic.z - magz_off;

mx = magx * magxx + magy * magxy + magz * magxz;
my = magx * magxy + magy * magyy + magz * magyz;
mz = magx * magxz + magy * magyz + magz * magzz;

ax = (accel.acceleration.x - accelx_off) * accelx_scale;
ay = (accel.acceleration.y - accely_off) * accely_scale;
az = (accel.acceleration.z - accelz_off) * accelz_scale;
  
gx = (gyro.gyro.x - gyrox_off) * gyrox_scale;
gy = (gyro.gyro.y - gyroy_off) * gyroy_scale;
gz = (gyro.gyro.z - gyroz_off) * gyroz_scale;

  // Gyroscope needs to be converted from Rad/s to Degree/s
  // the rest are not unit-important
  gx = gx * SENSORS_RADS_TO_DPS;
  gy = gy * SENSORS_RADS_TO_DPS;
  gz = gz * SENSORS_RADS_TO_DPS;

    // Update the SensorFusion filter
  filter.update(gx, gy, gz, 
                ax, ay, az, 
                mx, my, mz);
                
  filter.getQuaternion(&qw, &qx, &qy, &qz);//gets the quats, used in the bass code to get the roll, pitch, yaw
  filter.getLinearAcceleration(&linx, &liny, &linz);// gets the linear accel

// q3prime = -(sqrt(2)/2) * qw + (sqrt(2)/2) * qz;// maps fxo fxa to qwiic bno with chips mounted same
// q1prime = -(sqrt(2)/2) * qx - (sqrt(2)/2) * qy;//
// q2prime = -(sqrt(2)/2) * qy + (sqrt(2)/2) * qx;
// q0prime = +(sqrt(2)/2) * qz + (sqrt(2)/2) * qw;   

 q0prime = +(sqrt(2)/2) * qw - (sqrt(2)/2) * qz;//maps fxo fxa to non qwiic bno with chips mounted same
 q1prime = -(sqrt(2)/2) * qy + (sqrt(2)/2) * qx;//
 q2prime = +(sqrt(2)/2) * qx + (sqrt(2)/2) * qy;
 q3prime = +(sqrt(2)/2) * qz + (sqrt(2)/2) * qw;  

plucksig = gz;
lin_accelx = 9.81 * liny;    
                
}
