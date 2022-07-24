#include <Adafruit_FXOS8700.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
/* Assign a unique ID to this sensor at the same time */
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);

float gyrox_cal = 0;
float gyroy_cal = 0;
float gyroz_cal = 0;

float gyrox = 0;
float gyroy = 0;
float gyroz = 0;

float gyrox_off = +0.0;
float gyroy_off = +0.0;
float gyroz_off = +0.0;

//float gyrox_off = 0.00859;
//float gyroy_off = -0.00221;
//float gyroz_off = 0.00732;

float gyrox_scale = 1.0;
float gyroy_scale = 1.0;
float gyroz_scale = 1.0;

float accelx = 0;
float accely = 0;
float accelz = 0;

float accelx_cal = 0;
float accely_cal = 0;
float accelz_cal = 0;

//float accelx_off = -0.0165;
//float accely_off = -0.0025;
//float accelz_off = -0.058;
//
//float accelx_scale = .9869;
//float accely_scale = 1.0240;
//float accelz_scale = 1.00061;

float accelx_off = 0;
float accely_off = 0;
float accelz_off = 0;

float accelx_scale = 1.00;
float accely_scale = 1.00;
float accelz_scale = 1.00;

float magx;
float magy;
float magz;

//float magx_off = -1.835;
//float magy_off = -21.348;
//float magz_off = -49.511;
//
//float magx_scale = 0.9150;
//float magy_scale = 0.9153;
//float magz_scale = 1.0000;

float magx_off = 0.0;
float magy_off = 0.0;
float magz_off = 0.0;

float magx_scale = 1.0;
float magy_scale = 1.0;
float magz_scale = 1.0;

float alpha = .997;

long int counter = 0;

long int timer = 0;

void setup(void) {
  Serial.begin(115200);

  /* Wait for the Serial Monitor */
  while (!Serial) {
    delay(1);
  }

  Serial.println("FXOS8700 Test");
  Serial.println("");

  /* Initialise the sensor */
  if (!accelmag.begin()) {
    /* There was a problem detecting the FXOS8700 ... check your connections */
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while (1)
      ;
  }
if (!gyro.begin()) {
    /* There was a problem detecting the FXAS21002C ... check your connections
     */
    Serial.println("Ooops, no FXAS21002C detected ... Check your wiring!");
    while (1)
      ;
  }
 /* Set gyro range. (optional, default is 250 dps) */
   gyro.setRange(GYRO_RANGE_500DPS);

  accelmag.setOutputDataRate(ODR_400HZ);
}

void loop(void) {
  timer = micros();
counter++;
sensors_event_t aevent, mevent, gevent;

gyro.getEvent(&gevent);
accelmag.getEvent(&aevent, &mevent);
magx = (mevent.magnetic.x - magx_off) * magx_scale;
magy = (mevent.magnetic.y - magy_off) * magy_scale;
magz = (mevent.magnetic.z - magz_off) * magz_scale;

accelx_cal = (aevent.acceleration.x - accelx_off) * accelx_scale;
accely_cal = (aevent.acceleration.y - accely_off) * accely_scale;
accelz_cal = (aevent.acceleration.z - accelz_off) * accelz_scale;
  
gyrox_cal = (gevent.gyro.x - gyrox_off) * gyrox_scale;
gyroy_cal = (gevent.gyro.y - gyroy_off) * gyroy_scale;
gyroz_cal = (gevent.gyro.z - gyroz_off) * gyroz_scale;

gyrox = alpha * gyrox + (1-alpha) * gyrox_cal;
gyroy = alpha * gyroy + (1-alpha) * gyroy_cal;
gyroz = alpha * gyroz + (1-alpha) * gyroz_cal;

accelx = alpha * accelx + (1-alpha) * accelx_cal;
accely = alpha * accely + (1-alpha) * accely_cal;
accelz = alpha * accelz + (1-alpha) * accelz_cal;

//Serial.print( accelx, 4);
//Serial.print("    ");
//Serial.print(accely, 4);
//Serial.print("    ");
//Serial.print( accelz, 4);

//Serial.print( magx, 5);
//Serial.print("    ");
//Serial.print(magy, 5);
//Serial.print("    ");
//Serial.print( magz, 5);

Serial.print( gyrox, 5);
Serial.print("    ");
Serial.print(gyroy, 5);
Serial.print("    ");
Serial.print( gyroz, 5);


//Serial.print("    ");
//Serial.print( micros());


Serial.println();
while((micros()-timer)<10000){ // this delays for the remainder of the time up to 10ms
  }

}
