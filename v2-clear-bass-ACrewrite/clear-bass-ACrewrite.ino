/********************************************************

Rewrite of working code from the acrylic electronic
bass with the goal of simplifying subroutines, fixing
bugs, and clarifying variable names for future reference.

*********************************************************/


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

#include <FastLED.h>

#if FASTLED_VERSION < 3001000
#error "Requires FastLED 3.1 or later; check github for latest code."
#endif

// Define universal parameters for both strips
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define BRIGHTNESS  255

// Bow parameters
#define b_DATA_PIN    3
//#define CLK_PIN   4
#define b_NUM_LEDS    10

// Neck parameters
#define n_DATA_PIN    4
//#define CLK_PIN   4
#define n_NUM_LEDS    108

// Initialize led method
CRGBArray<b_NUM_LEDS> b_leds;
CRGBArray<n_NUM_LEDS> n_leds;


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

// Unused test values
/*float alpha = .99;
float beta = 0.99;
float gammer = 0.98; */

float alpha = 1;
float beta = .99;//.997
float gammer = .99;//.995

float out_vx = 0.0; //output
float out_vx_prev = 0.0;

float out_ax = 0.0; //output
float out_ax_prev = 0.0;
// end of bow velocity variables

int bow_angle = 0;
int bow_pos;

float stringfreq;

// the following store and average the analog reads for the two fret positions
float read_raw1;
float read_raw2;
// read average over x reads of the potentiometer
float read_raw1_ave;
float read_raw2_ave;

// Pingpong lightshow variables
int bouncewidth = 20;
int deeelay = 5;
uint8_t hue;
uint8_t trailhue;
int delaychange = 1;



// the following store the resistances and finger positions
// L_bridge is the distance from the finger closest to the bridge to the bridge
// L_nut is the distance from the finger closest to the nut to the nut
// L_between is the distance between the two fingers
// if there is only one finger, L_between is the "width" of the finger

// Softpot reads and purported "lengths"
float R_nut; // Read from nut to top finger (R_nut)
float R_between; // Read between fingers
float R_bridge; // Read from closest to bridge (R_bridge)
float L_nut; // Length from nut to top finger
float L_between; // Length between fingers
float L_bridge; // Length from bridge (L_bridge)

int bitnumber = 12;
float resolution = pow(2,bitnumber);

float string_length;
float base_freq;
float base_freq_discrete;
float low_string_freq = 41.2; // E1 for lowest bass note
//float low_string_freq = 65.4; // C2 for lowest cello note

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

float scale = 104.14; // Bass open string length
//float scale = 75; // Cello open string length

bool bowing = false;

void setup(void) {
  delay(3000); // 3 second delay for recovery

  Serial.begin(115200);

  // Initialize bow LEDs
  FastLED.addLeds<LED_TYPE, b_DATA_PIN, COLOR_ORDER>(b_leds, b_NUM_LEDS)
    .setCorrection(TypicalLEDStrip)
    .setDither(BRIGHTNESS < 255);

  // Initialize neck LEDs
  FastLED.addLeds<LED_TYPE, n_DATA_PIN, COLOR_ORDER>(n_leds, n_NUM_LEDS)
    .setCorrection(TypicalLEDStrip)
    .setDither(BRIGHTNESS < 255);


  // Master brightness
  FastLED.setBrightness(120);
  
  FastLED.clear();
  FastLED.show();
  
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
  waveform1.frequency(low_string_freq);

  envelope1.delay(0);
  envelope1.attack(20);  
  envelope1.hold(40);
  envelope1.decay(200);
  envelope1.sustain(1.0);
  envelope1.release(200);

  filter1.resonance(.8);
  filter1.frequency(2000);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  // Initialize the button
  pinMode(2, INPUT_PULLUP);
  
  delay(500);
  waveform1.amplitude(0.0);
  imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    a_x_init = linearaccel.y();// get initial value for acceleration
}

void loop(void)
{
  button0.update();
  timer0 = micros();// starts the timer
  elapsedMillis timervariable;
  
  potcalc(); //determines fretted string length L_bridge, and also L_nut, L_between

  bow_angle_calc();  //determines bow_pos and stringfreq, also produces theta, phi, chi
  
  if (L_bridge < (3 + (scale - 75.0))) {// zeroes bow angle for standing case
    chi0 = chi; 
  }
      
base_freq = stringfreq * scale/L_bridge;  // for continuous
fretnum = 5* bow_pos + roundup(12.0*log(scale/L_bridge)/(log(2)));
base_freq_discrete = low_string_freq * pow(2,float(fretnum)/12.0); // for discrete
bow_vel_calc(); //determines a bow velocity out_vx

waveform1.frequency(base_freq);
string1.setFrequency(base_freq);

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

if(bowing) {
   // float amp = abs(out_vx);
    float amp = out_vx * out_vx;
   //Serial.println(amp, 8);
    if(amp <= .0003) {
      amp = 0.0;
    }
    amp = pow(amp, 0.3);
    waveform1.amplitude(amp);
  }

if ((phi < 130) && (phi > 50)) {// is bow in plucking position?
  if (note_off_flag == 0){
    note_off_flag = 1;  
  }
pluckcalc();//calculates when a pluck has occurred and plays a plucked note
}

if ((phi < 40) && (phi > -40)) {// is bow in bowing position?
  if (note_off_flag == 1) {
    note_off();
    note_off_flag = 0; 
  }
  bow_action(); // bows the string
}
FastLED.show();
Serial.println(timervariable);
timervariable = 0;

while((micros()-timer0)<10000){ // this delays for the remainder of the time up to 10ms
}
}

void bow_action(){
  float temp = constrain(theta, -40, 40);
  filter_scale =  40.f * ((temp + 40.) / 80.f) + 6.0;
  filter1.frequency(base_freq * filter_scale);
}

void potcalc(){

    read_raw1_ave = 0.0;// some more averaging on the analog reads
    read_raw2_ave = 0.0;  
    for(int i = 0; i <16; i++){      
    read_raw1 = analogRead(A6);
    read_raw2 = analogRead(A7);
    read_raw1_ave = read_raw1_ave + read_raw1;
    read_raw2_ave = read_raw2_ave + read_raw2;
}
    read_raw1_ave = read_raw1_ave/16.0;
    read_raw2_ave = read_raw2_ave/16.0;


    R_nut =  (float(read_raw2_ave)/(resolution - float(read_raw2_ave)));
    R_bridge =  (float(read_raw1_ave)/(resolution - float(read_raw1_ave)));
    
if (read_raw1_ave > 0.9*resolution) { // this will be true if string is not fretted
  L_nut = 0.0;
  L_bridge = scale;
  L_between = 0.0; 
}else{
  // Scaling/calibration
  L_bridge = (2.84 * R_bridge*R_bridge + 76.8 * R_bridge - 0.163) - 75.0 + scale;
  L_nut =  5.944 * R_nut*R_nut + 66.1 * R_nut - 0.3272;
  L_between = scale - (L_nut + L_bridge); 

  L_bridge = constrain(L_bridge, 0.0, scale);
}

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
stringfreq = low_string_freq * pow(1.3348,bow_pos); // upright bass tuned in fourths
//stringfreq = low_string_freq * pow(1.5,bow_pos); // Cello tuned in fifths
//stringfreq = 130.8;

switch (bow_pos) {
    case 0:    // red
    FastLED.clear();
    b_leds[0] = CRGB(50, 0, 0);
    b_leds[9] = CRGB(50, 0, 0);
    for (int i=0; i<n_NUM_LEDS; i++) {
      n_leds[i] = CRGB(50,0,0);
    }
   // FastLED.show();      
      break;
      
    case 1:    // yellow
    FastLED.clear();
    b_leds[1] = CRGB(50, 50, 0);
    b_leds[8] = CRGB(50, 50, 0);
    for (int i=0; i<n_NUM_LEDS; i++) {
      n_leds[i] = CRGB(50,50,0);
    }
  //  FastLED.show();      
      break;
      
    case 2:    // green
    FastLED.clear();
    b_leds[2] = CRGB(0, 50, 0);
    b_leds[7] = CRGB(0, 50, 0);
    for (int i=0; i<n_NUM_LEDS; i++) {
      n_leds[i] = CRGB(0,50,0);
    }
  //  FastLED.show();     
      break;
      
    case 3:    // blue
    FastLED.clear();
    b_leds[3] = CRGB(0, 0, 50);
    b_leds[6] = CRGB(0, 0, 50);
    for (int i=0; i<n_NUM_LEDS; i++) {
      n_leds[i] = CRGB(0,0,50);
    }
  //  FastLED.show();         
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
   
if ((binoutprev == 0) && (binout == 1)){
  upflag = 1;
  downflag = 0;
  note_on();
} else if ((binoutprev == 1) && (binout == 0)) {
  upflag = 0;
  downflag = 1;
  note_off();
} else {
  upflag = 0;
  downflag = 0; 
}

binoutprev = binout;
plucksig_prev = plucksig;
button0.update();

if (button0.fallingEdge()) {
  upflag = 1;
  downflag = 0;
  note_on();
}

if (button0.risingEdge()) {
  upflag = 0;
  downflag = 0; 
}

} // end pluckcalc subroutine


void bow_vel_calc() {
  imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    // this takes a bit of time
  lin_accelx = linearaccel.y();    
  a_x_board = lin_accelx - a_x_init;// subtract iff any initial offset
  out_ax = gammer * out_ax_prev + gammer * ( a_x_board - a_x_board_prev);// high pass accel
    
//  v_x_board = alpha * v_x_board_prev + alpha * 0.5 * (out_ax + out_ax_prev) * delt;
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
  if (a - float(int(a)) > 0) {
    return int(a)+1;
  } else {
    return a;
  } 
}
