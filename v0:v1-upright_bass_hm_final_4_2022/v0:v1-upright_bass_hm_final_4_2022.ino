
/*
 * This version is based off of Konrad's addtions which use an envelope on the bowing action
 * and a different amplitude for the bowing. Also the frequency can change as the bowing is happening
 * There is also a filter acting on the sawtooth waveform which is modified with the theta angle of the bow.
 * There is also a modification to the bowing amplitude with a threshold and power function.
 * 
 * 
 * 
 * 
 */



#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#define PI 3.141592653

#include <Audio.h>
#include <SPI.h>
//#include <SD.h>
//#include <SerialFlash.h>
#include <Bounce.h>
#include "AcousticBs_samples.h"

#include <Adafruit_NeoPixel.h>
#define PIN        3 // neopixel pin

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 8 // 
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// GUItool: begin automatically generated code
AudioSynthWavetable      string1;          //xy=125,106
AudioSynthWaveform       waveform1;      //xy=132,61
AudioEffectEnvelope      envelope1;
AudioMixer4              mixer1;         //xy=353,103
AudioOutputAnalog        dac1;           //xy=506,102
AudioFilterStateVariable filter1;
AudioConnection          patchCord1(string1, 0, mixer1, 1);
AudioConnection          patchCord2(waveform1, 0, filter1, 0);

AudioConnection          patchCord3(filter1, 0, envelope1, 0);

AudioConnection          patchCord4(envelope1, 0, mixer1, 0);
AudioConnection          patchCord5(mixer1, dac1);
// GUItool: end automatically generated code


// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
double long timer0; // variable to store initial time in micros

// the following variables are for the bow velocity
float lin_accelx;
float a_x_board = 0.0; //input
float a_x_board_prev = 0.0;

float a_x_init;

float v_x_board = 0.0; //input
float v_x_board_prev = 0.0;

float delt = 0.01;
//float thresh = 0.03;

//float alpha = .99;
//float beta = 0.99;
//float gammer = 0.98;

float alpha = 1;
float beta = .99;//.997
float gammer = .99;//.995

float out_vx = 0.0; //output
float out_vx_prev = 0.0;

float out_ax = 0.0; //output
float out_ax_prev = 0.0;
// end of bow velocity variables


int bow_angle= 0;
int bow_pos;

float stringfreq;

// the following store and average the analog reads for the two fret positions
float read_raw1;
float read_raw2;
float read_raw1_ave;
float read_raw2_ave;


// the following store the resistances and finger positions
// L_3 is the distance from the finger closest to the bridge to the bridge
// L_1 is the distance from the finger closest to the nut to the nut
// L_2 is the distance between the two fingers
// if there is only one finger, L_2 is the "width" of the finger

float R_1;
float R_2;
float R_3;
float L_1;
float L_2;
float L_3;

int bitnumber = 12;
float resolution = pow(2,bitnumber);
int ave_number = 1;

float string_length;
float base_freq;
float base_freq_discrete;
float low_string_freq = 41.2;//E1

float filter_scale;

  //for the button on bow
Bounce button0 = Bounce(2, 15);
int button0_value;

// the following use the gyro velocity about the z-axis to get the pluck signal
float plucksig;
float plucksig_prev = 0;
float pluck_vel;

int threshL = -150;
int threshH = 40;
int thresh = threshH;

int upflag = 0;
int downflag=0;

int binout = 0;
int binoutprev = 0;

int note_off_flag=0;

int fretnum; //number of half steps up from base note

// these are the euler angles, the quaternions, and the transformed quaternions
float phi;
float theta;
float chi;
float chi0 = 0;
float del_chi;
float q0,q1,q2,q3;
float q0prime, q1prime, q2prime, q3prime;

float scale = 104.14;// this is the length of the open string

bool bowing = false;

void setup(void)
{
  Serial.begin(115200);
  pixels.begin();
  pixels.clear();
  
  AudioMemory(20);

  analogReadResolution(bitnumber);
//  analogReadAveraging(8);

  AudioNoInterrupts(); 
  
  string1.setInstrument(AcousticBs);
  string1.amplitude(1);

  AudioInterrupts();
  mixer1.gain(0, 0.5);//sawtooth waveform for bowing
  mixer1.gain(1, 0.5);//Picked Bs wavetable
 
  delay(500);
  waveform1.begin(WAVEFORM_SAWTOOTH);
  waveform1.amplitude(0.125);
  waveform1.frequency(low_string_freq);

  envelope1.delay(0);
  envelope1.attack(20);  
  envelope1.hold(40);
  envelope1.decay(200);
  envelope1.sustain(1.0);
  envelope1.release(200);

  filter1.resonance(.8);
  filter1.frequency(2000);

//        
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  pinMode(2, INPUT_PULLUP);
//  pinMode(3, OUTPUT); for the four LEDs, not used for hm version
//  pinMode(4, OUTPUT);
//  pinMode(5, OUTPUT);
//  pinMode(6, OUTPUT);
  
  delay(500);
  waveform1.amplitude(0.0);
  imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    a_x_init = linearaccel.y();// get initial value for acceleration
  
}

void loop(void)
{
    timer0 = micros();// starts the timer
//    Serial.println(float(micros())/1000.0); // show initial time
//    Serial.println(" ");


    potcalc(); //determines fretted string length L_3, and also L_1, L_2

    bow_angle_calc();  //determines bow_pos and stringfreq, also produces theta, phi, chi
    
  if (L_3 < (3 + (scale - 75.0))){// zeroes bow angle for standing case
    chi0 = chi; 
      }
      
base_freq = stringfreq * scale/L_3;  // for continuous
fretnum = 5* bow_pos + roundup(12.0*log(scale/L_3)/(log(2)));
base_freq_discrete = low_string_freq * pow(2,float(fretnum)/12.0); // for discrete
bow_vel_calc();//determines a bow velocity out_vx
//Serial.println(L_3);
//Serial.println(base_freq_discrete);
Serial.println(fretnum);

waveform1.frequency(base_freq);
string1.setFrequency(base_freq);

if ((phi < 130) && (phi > 50)){// is bow in plucking position?

if (note_off_flag == 0){
  note_off_flag = 1;  
}
pluckcalc();//calculates when a pluck has occurred and plays a plucked note

}

if ((phi < 40) && (phi > -40)){// is bow in bowing position?
if (note_off_flag == 1){

note_off();
note_off_flag = 0;
    
}
bow_action();// bows the string
}

  while((micros()-timer0)<10000){ // this delays for the remainder of the time up to 10ms
  }

}

void bow_action(){

  button0.update();

  if (button0.fallingEdge()) {
    envelope1.sustain(1.0);
    envelope1.noteOn();
    bowing = true;
  }
  if (button0.risingEdge()){
  
 // waveform1.amplitude(0.0);
  envelope1.noteOff();
  bowing = false;
  }
  
  if(bowing) 
  {
   // float amp = abs(out_vx);
    float amp = out_vx * out_vx;

    Serial.println(amp, 8);
    if(amp <= .0003) amp = 0.0;

    amp = pow(amp, 0.3);

    waveform1.amplitude(amp);
  }

  float temp = constrain(theta, -40, 40);
  filter_scale =  40.f * ((temp + 40.) / 80.f) + 6.0;
  filter1.frequency(base_freq * filter_scale);
  
}

void potcalc(){

    read_raw1_ave = 0.0;// some more averaging on the analog reads
    read_raw2_ave = 0.0;  
    for(int i = 0; i <16; i++){      
    read_raw1 = analogRead(A7);
    read_raw2 = analogRead(A6);
    read_raw1_ave = read_raw1_ave + read_raw1;
    read_raw2_ave = read_raw2_ave + read_raw2;
}
    read_raw1_ave = read_raw1_ave/16.0;
    read_raw2_ave = read_raw2_ave/16.0;


    R_1 =  (float(read_raw2_ave)/(resolution - float(read_raw2_ave)));
    R_3 =  (float(read_raw1_ave)/(resolution - float(read_raw1_ave)));
    
if (read_raw1_ave > 0.9*resolution){// this will be true if string is not fretted
L_1 = 0.0;
L_3 = scale;
L_2 = 0.0;
  
}else{

L_3 = (-4.3035 * R_3*R_3 + 75.806 * R_3 - 0.2821) - 75.0 + scale;
// from newest calibration fits 3-28-2022
L_1 =  5.944 * R_1*R_1 + 66.1 * R_1 - 0.3272;
L_2 = scale - (L_1 + L_3);

L_3 = constrain(L_3, 0.0, scale);
}
//    Serial.print(read_raw1_ave);
 //   Serial.print("   ");   
//    Serial.println(read_raw2_ave);

//    Serial.print(L_1);
//    Serial.print("   ");   
//    Serial.print(L_2);
//    Serial.print("   ");       
//    Serial.println(L_3);

}

void bow_angle_calc(){

 imu::Quaternion quat = bno.getQuat();

 q0prime = quat.w();
 q1prime = quat.x();
 q2prime = quat.y();
 q3prime = quat.z(); 

 q0 = (sqrt(2)/2) * q0prime - (sqrt(2)/2) * q3prime;//rotate about z
 q1 = (sqrt(2)/2) * q1prime + (sqrt(2)/2) * q2prime;
 q2 = (sqrt(2)/2) * q2prime - (sqrt(2)/2) * q1prime;
 q3 = (sqrt(2)/2) * q3prime + (sqrt(2)/2) * q0prime;   

// q0 = (sqrt(2)/2) * q0prime - (sqrt(2)/2) * q1prime;//rotate about x
// q1 = (sqrt(2)/2) * q1prime + (sqrt(2)/2) * q0prime;
// q2 = (sqrt(2)/2) * q2prime + (sqrt(2)/2) * q3prime;
// q3 = (sqrt(2)/2) * q3prime - (sqrt(2)/2) * q2prime;   

// q0 = (sqrt(2)/2) * q0prime - (sqrt(2)/2) * q2prime;//rotate about y
// q1 = (sqrt(2)/2) * q1prime - (sqrt(2)/2) * q3prime;
// q2 = (sqrt(2)/2) * q2prime + (sqrt(2)/2) * q0prime;
// q3 = (sqrt(2)/2) * q3prime + (sqrt(2)/2) * q1prime;  

phi = (360/(2*3.141592653))*atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
theta = (360/(2*3.141592653))*asin(2*(q0*q2-q3*q1));
chi = (360/(2*3.141592653))*atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
  /* Display the floating point data */

    del_chi = chi - chi0;
if (del_chi < -180){// keep del_chi in the correct range from -180 => 180
  del_chi = del_chi +360;
}

if (del_chi > 180){
  del_chi = del_chi -360;
}
    bow_angle = del_chi;
    bow_angle = constrain(bow_angle, -45,45);
    bow_pos = map(int(bow_angle), -45,45,0,3);// gets the different strings
    bow_pos = constrain(bow_pos,0,3);
    stringfreq = low_string_freq * pow(1.3348,bow_pos);// upright tuned in fourths
//    stringfreq = 130.8;
Serial.println(chi);
Serial.println(bow_pos);
 switch (bow_pos) {
    case 0:    // red
    pixels.clear();
    pixels.setPixelColor(0, pixels.Color(50, 0, 0));
    pixels.setPixelColor(7, pixels.Color(50, 0, 0));
    pixels.show();     
      break;
    case 1:    // yellow
    pixels.clear();
    pixels.setPixelColor(1, pixels.Color(50, 50, 0));
    pixels.setPixelColor(6, pixels.Color(50, 50, 0));
    pixels.show();     
      break;
    case 2:    // green
    pixels.clear();
    pixels.setPixelColor(2, pixels.Color(0, 50, 0));
    pixels.setPixelColor(5, pixels.Color(0, 50, 0));
    pixels.show();     
      break;
    case 3:    // blue
    pixels.clear();
    pixels.setPixelColor(3, pixels.Color(0, 0, 50));
    pixels.setPixelColor(4, pixels.Color(0, 0, 50));
    pixels.show();     
      break; 
  }


}

void pluckcalc(){
  
imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);  
plucksig = gyro.z();
pluck_vel = 2* (plucksig - plucksig_prev);// factor of 2 just got things in right range
pluck_vel = constrain(pluck_vel, 50,300.0);
//Serial.println(pluck_vel);

  
   if (plucksig <= thresh) {
binout = 0;
thresh = threshH;    
   }

   if (plucksig > thresh) {
binout = 1;
thresh = threshL;  
   }
   
if ( (binoutprev == 0) && (binout == 1)){

upflag = 1;
downflag = 0;
note_on();
  
}else if ( (binoutprev == 1) && (binout == 0)){
upflag = 0;
downflag = 1;
note_off();
 
}else{
upflag = 0;
downflag = 0; 
}
binoutprev = binout;
plucksig_prev = plucksig;

  button0.update();
  if ( button0.fallingEdge() ) {
  
      upflag = 1;
      downflag = 0;
      note_on();
  
  }
  if ( button0.risingEdge() ) {
      upflag = 0;
      downflag = 0; 
  }

}


void bow_vel_calc(){
    imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    // this takes a bit of time
    lin_accelx = linearaccel.y();    
    a_x_board = lin_accelx - a_x_init;// subtract iff any initial offset
    out_ax = gammer * out_ax_prev + gammer * ( a_x_board - a_x_board_prev);// high pass accel
    
//    v_x_board = alpha * v_x_board_prev + alpha * 0.5 * (out_ax + out_ax_prev) * delt;
    v_x_board =  v_x_board_prev +  out_ax * delt;// integrate accel to get vel
    out_vx = beta * out_vx_prev + beta * ( v_x_board - v_x_board_prev);//high pass vel


 // Serial.print(bow_pos);
 // Serial.print("  ");
 // Serial.println(out_vx*out_vx*10);
    
  a_x_board_prev = a_x_board;
  out_ax_prev = out_ax;
  v_x_board_prev = v_x_board;
  out_vx_prev = out_vx;
  
}


void note_on(){
mixer1.gain(1, pluck_vel/300.0);
string1.playFrequency(base_freq);

}

void note_off(){
string1.stop();
}



int roundup( float a){

if (a - float(int(a)) > 0){
return int(a)+1;
}else {
return a;
}
  
}
