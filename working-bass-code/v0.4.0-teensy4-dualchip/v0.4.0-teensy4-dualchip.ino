/********************************************************

Rewrite of working code from the acrylic electronic
bass with the goal of simplifying subroutines, fixing
bugs, and clarifying variable names for future reference.

*********************************************************/

// BF

#include "bno055_subs.h"
//#include "fxa_fxo_subs.h"
#include <Audio.h>
#include <SPI.h>
//#include <SD.h>
//#include <SerialFlash.h>
#include <Bounce.h>
#include "AcousticBs_samples.h"

#include <Adafruit_NeoPixel.h>
#define b_PIN        3 // neopixel pin
#define n_PIN        4 // neopixel pin

// How many NeoPixels are attached to the Arduino?
#define b_NUMPIXELS 28 // 
Adafruit_NeoPixel b_pixels(b_NUMPIXELS, b_PIN, NEO_GRB + NEO_KHZ800);

#define n_NUMPIXELS 101 // 
Adafruit_NeoPixel n_pixels(n_NUMPIXELS, n_PIN, NEO_GRB + NEO_KHZ800);


// GUItool: begin automatically generated code
AudioSynthWavetable      string1;          //xy=125,106
AudioSynthWaveform       waveform1;      //xy=132,61
AudioEffectEnvelope      envelope1;
AudioMixer4              mixer1;         //xy=353,103
AudioOutputPT8211        dac1;           //xy=506,102
AudioFilterStateVariable filter1;
AudioConnection          patchCord1(string1, 0, mixer1, 1);
AudioConnection          patchCord2(waveform1, 0, filter1, 0);

AudioConnection          patchCord3(filter1, 0, envelope1, 0);

AudioConnection          patchCord4(envelope1, 0, mixer1, 0);
AudioConnection          patchCord5(mixer1, dac1);
// GUItool: end automatically generated code


double long timer0; // variable to store initial time in micros

// the following variables are for the bow velocity
// float lin_accelx;
float a_x_board = 0.0; //input
float a_x_board_prev = 0.0;
float a_x_init = 0;
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
int bow_pos_prev;

float stringfreq;

// the following store and average the analog reads for the two fret positions
float read_raw1;
float read_raw2;
// read average over x reads of the potentiometer
float read_raw1_ave;
float read_raw2_ave;


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
// float plucksig;
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
// float q0prime, q1prime, q2prime, q3prime;

float scale = 104.14; // Bass open string length
//float scale = 75; // Cello open string length

bool bowing = false;

void setup() {
  delay(3000); // 3 second delay for recovery

  Serial.begin(115200);

  b_pixels.begin();
  b_pixels.clear();
  n_pixels.begin();
  n_pixels.clear();
  
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

  // BF
  
  bno055_setup_subs(); 
  //fxa_fxo_setup_subs();

  // Initialize the button
  pinMode(2, INPUT_PULLUP);

  
  
  delay(500);
  waveform1.amplitude(0.0);
  // imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  //  a_x_init = linearaccel.y();// get initial value for acceleration
}

void loop(void) { 
  timer0 = micros();// starts the timer

  Serial.println(L_bridge);
  
  // BF 
  
  bno055_main_calc();
  //fxa_fxo_main_calc();
  
  potcalc(); //determines fretted string length L_bridge, and also L_nut, L_between

  bow_angle_calc();  //determines bow_pos and stringfreq, also produces theta, phi, chi
  
  if (L_bridge < (3 + (scale - 75.0))) {// zeroes bow angle for standing case
    chi0 = chi; 
  }
  //Serial.println(filter_scale);
      
base_freq = stringfreq * scale/L_bridge;  // for continuous
fretnum = 5* bow_pos + roundup(12.0*log(scale/L_bridge)/(log(2)));
base_freq_discrete = low_string_freq * pow(2,float(fretnum)/12.0); // for discrete
bow_vel_calc(); //determines a bow velocity out_vx

waveform1.frequency(base_freq);

if (string1.isPlaying()) {
  string1.setFrequency(base_freq);
}

button0.update();

if (button0.fallingEdge()) {
  envelope1.sustain(1.0);
  envelope1.noteOn();
  bowing = true;
  Serial.println("falling");
}

if (button0.risingEdge()){
 // waveform1.amplitude(0.0);
    envelope1.noteOff();
    bowing = false;
      Serial.println("rising");
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
    
    for(int i = 0; i <5; i++){      
      read_raw1 = analogRead(A8);
      read_raw2 = analogRead(A9);
      read_raw1_ave = read_raw1_ave + read_raw1;
      read_raw2_ave = read_raw2_ave + read_raw2;
    }
    
    read_raw1_ave = read_raw1_ave/5.0;
    read_raw2_ave = read_raw2_ave/5.0;

    R_nut =  (float(read_raw2_ave)/(resolution - float(read_raw2_ave)));
    R_bridge =  (float(read_raw1_ave)/(resolution - float(read_raw1_ave)));
    
if (read_raw1_ave > 0.9*resolution) { // this will be true if string is not fretted
  L_nut = 0.0;
  L_bridge = scale;
  L_between = 0.0; 
}else{
  // Scaling/calibration
  L_bridge = (-1.1618 * R_bridge*R_bridge + 74.069 * R_bridge - 0.2172) - 75.0 + scale;
  L_nut =  5.944 * R_nut*R_nut + 66.1 * R_nut - 0.3272;
  L_between = scale - (L_nut + L_bridge); 

  L_bridge = constrain(L_bridge, 0.0, scale);
}

}

void bow_angle_calc(){
//imu::Quaternion quat = bno.getQuat();
//
//q0prime = -quat.w(); // go through the math
//q1prime = quat.x();
//q2prime = quat.y();
//q3prime = -quat.z(); 

q0prime = -q2prime; // real solution for new version of bow
q1prime = -q3prime;
q2prime = q0prime;
q3prime = q1prime;

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
bow_angle = constrain(bow_angle, -45, 60); // DEBUG
bow_pos_prev = bow_pos;
bow_pos = map(int(bow_angle), -45,60,0,3);// gets the different strings
bow_pos = constrain(bow_pos,0,3);
stringfreq = low_string_freq * pow(1.3348,bow_pos); // upright bass tuned in fourths
//stringfreq = low_string_freq * pow(1.5,bow_pos); // Cello tuned in fifths
//stringfreq = 130.8;
if (bow_pos != bow_pos_prev) {
  switch (bow_pos) {
    case 0:    // red
    b_pixels.clear();
    
    b_pixels.setPixelColor(0, b_pixels.Color(50, 0, 0));
    b_pixels.setPixelColor(1, b_pixels.Color(50, 0, 0));
    b_pixels.setPixelColor(14, b_pixels.Color(50, 0, 0));
    b_pixels.setPixelColor(15, b_pixels.Color(50, 0, 0));
    
    b_pixels.show();
    
    n_pixels.clear();
    for (int i=0; i<n_NUMPIXELS; i++) {
      n_pixels.setPixelColor(i, n_pixels.Color(50, 0, 0));
    }
    n_pixels.show();     
      break;
      
    case 1:    // yellow
    b_pixels.clear();
    
    b_pixels.setPixelColor(2, b_pixels.Color(50, 50, 0));
    b_pixels.setPixelColor(3, b_pixels.Color(50, 50, 0));
    b_pixels.setPixelColor(12, b_pixels.Color(50, 50, 0));
    b_pixels.setPixelColor(13, b_pixels.Color(50, 50, 0));

    b_pixels.show();
    
    n_pixels.clear();
    for (int i=0; i<n_NUMPIXELS; i++) {
      n_pixels.setPixelColor(i, n_pixels.Color(50, 50, 0));
    }
    n_pixels.show();     
      break;
      
    case 2:    // green
    b_pixels.clear();
    
    b_pixels.setPixelColor(4, b_pixels.Color(0, 50, 0));
    b_pixels.setPixelColor(5, b_pixels.Color(0, 50, 0));
    b_pixels.setPixelColor(10, b_pixels.Color(0, 50, 0));
    b_pixels.setPixelColor(11, b_pixels.Color(0, 50, 0));
    
    b_pixels.show();
    
    n_pixels.clear();
    for (int i=0; i<n_NUMPIXELS; i++) {
      n_pixels.setPixelColor(i, n_pixels.Color(0, 50, 0));
    }
    n_pixels.show();   
      break;
      
    case 3:    // blue
    b_pixels.clear();
    
    b_pixels.setPixelColor(6, b_pixels.Color(0, 0, 50));
    b_pixels.setPixelColor(7, b_pixels.Color(0, 0, 50));
    b_pixels.setPixelColor(8, b_pixels.Color(0, 0, 50));
    b_pixels.setPixelColor(9, b_pixels.Color(0, 0, 50));

    b_pixels.show();
    
    n_pixels.clear();
    for (int i=0; i<n_NUMPIXELS; i++) {
      n_pixels.setPixelColor(i, n_pixels.Color(0, 0, 50));
    }
    n_pixels.show();  
      break; 
  }
 }
}

void pluckcalc(){

// imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);  
plucksig = -plucksig;
pluck_vel = 2 * (plucksig - plucksig_prev);// factor of 2 just got things in right range
pluck_vel = constrain(pluck_vel, 50,300.0);

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


void bow_vel_calc(){
// imu::Vector<3> linearaccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    // this takes a bit of time
 // lin_accelx = linearaccel.y();    
  a_x_board = lin_accelx - a_x_init;// subtract iff any initial offset
  out_ax = gammer * out_ax_prev + gammer * (a_x_board - a_x_board_prev);// high pass accel
    
//  v_x_board = alpha * v_x_board_prev + alpha * 0.5 * (out_ax + out_ax_prev) * delt;
  v_x_board =  v_x_board_prev +  out_ax * delt;// integrate accel to get vel
  out_vx = beta * out_vx_prev + beta * (v_x_board - v_x_board_prev);//high pass vel
    
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
