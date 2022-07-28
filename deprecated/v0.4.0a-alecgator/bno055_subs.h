#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define PI 3.141592653
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
float q0prime, q1prime, q2prime, q3prime;
float plucksig;
float lin_accelx;

void bno055_main_calc(){

imu::Quaternion quat = bno.getQuat();

 q0prime = + quat.w();// use these for non qwiic bno055
 q1prime = + quat.x();
 q2prime = + quat.y();
 q3prime = + quat.z(); 

// q3prime = - quat.w();// use these for qwiic bno055
// q2prime = + quat.x();
// q1prime = - quat.y();
// q0prime = + quat.z(); 
 
// q0 =  -  q3prime;//rotate about z by 180
// q1 =  +  q2prime;
// q2 =  -  q1prime;
// q3 =  +  q0prime;   

imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);  
plucksig = gyro.z();

imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
lin_accelx = linearaccel.y();  
  
}

void bno055_setup_subs(void){
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  
}
