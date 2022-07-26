/********************************************************

Complete rewrite of bass code from the ground up.

To find lines of code that must be changed to switch between 
the BNO and FXO-FXA, search for "BF"

To find lines of code that must be changed to switch between 
the Teensy 3.2 and Teensy 4.0 with PT8211 chip, search for "T34"



*********************************************************/

// BF

#include "bno055_subs.h"
//#include "fxa_fxo_subs.h"

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <SerialFlash.h>
#include "AcousticBs_samples.h"

#include <Adafruit_NeoPixel.h>
#define b_PIN        3 // neopixel pin
#define n_PIN        4 // neopixel pin

// How many NeoPixels are attached to the Arduino?
#define b_NUMPIXELS 10 // 
Adafruit_NeoPixel b_pixels(b_NUMPIXELS, b_PIN, NEO_GRB + NEO_KHZ800);

#define n_NUMPIXELS 108 // 
Adafruit_NeoPixel n_pixels(n_NUMPIXELS, n_PIN, NEO_GRB + NEO_KHZ800);


// GUItool: begin automatically generated code
AudioSynthWaveform       waveform1;      //xy=282,462
AudioSynthWavetable      wavetable1;        //xy=326,512
AudioFilterStateVariable filter1;        //xy=451,415
AudioMixer4              mixer1;         //xy=503,504
AudioEffectEnvelope      envelope1;      //xy=615,408

// T34
AudioOutputPT8211        dac1;
//AudioOutputAnalog        dac1;           //xy=695,600

AudioConnection          patchCord1(waveform1, 0, filter1, 0);
AudioConnection          patchCord2(wavetable1, 0, mixer1, 1);
AudioConnection          patchCord3(filter1, 0, envelope1, 0);
AudioConnection          patchCord4(mixer1, dac1);
AudioConnection          patchCord5(envelope1, 0, mixer1, 0);
// GUItool: end automatically generated code


// Declare global variables
uint8_t mode = 1;
uint8_t bow_pos, bow_pos_prev = 0;

uint8_t bitnumber = 12;
float resolution = pow(2,bitnumber);

float bridge_read, nut_read, bridge_ave, nut_ave = 0;
float L_bridge, L_nut, L_between = 0;

// Bass/Cello
float open_string_length = 104.14; // Upright bass
float low_string_freq = 41.2; // E1
float string_freq_ratio = 1.3348; // tuned in fourths

//float open_string_length = 75; // Cello
//float low_string_freq = 65.4; // C2
//float string_freq_ratio = 1.5; // tuned in fifths


float playing_freq = 0;

float phi, theta, chi = 0; // Euler Angles
float chi0, delta_chi = 0; // Calibration offset

uint8_t delta_t = 10; // Integration variable, represents time steps in ms

void setup() {
  delay(500); // 0.5 second delay for recovery


  Serial.begin(115200);

  b_pixels.begin(); // Initialize neopixels
  n_pixels.begin();
  b_pixels.clear();
  n_pixels.clear();
  
  AudioMemory(20);
  
  analogReadResolution(bitnumber);

  // Set audio levels
  mixer1.gain(0, 0.5); // sawtooth waveform for bowing
  mixer1.gain(1, 0.5); // plucked bass wavetable

  AudioNoInterrupts(); // Initialize synthesizers
  wavetable1.setInstrument(AcousticBs);
  wavetable1.amplitude(1); 
  waveform1.begin(WAVEFORM_SAWTOOTH);
  waveform1.amplitude(0.0);
  waveform1.frequency(440); // arbitrary starting frequency
  AudioInterrupts();

  envelope1.delay(0);     // envelope attack/release parameters
  envelope1.attack(20);  
  envelope1.hold(40);
  envelope1.decay(200);
  envelope1.sustain(1.0);
  envelope1.release(200);

  filter1.resonance(.8);  // theta angle filter parameters
  filter1.frequency(2000);


  // BF
  bno055_setup_subs();
  //fxa_fxo_setup_subs();

  pinMode(2, INPUT_PULLUP); // Initialize button
  
  delay(500);
}


void loop() { // low-level implementation of mode selection, more future-proof
  elapsedMillis timer0 = 0;
  
  switch (mode) {
      case 1: // normal bass bowing
      bowingLoop();
        break;
  
      case 2: // softpot touch mode
      touchLoop();
        break;
  }
  
  while (timer0 < delta_t) {} //delay to ensure loop is 10 ms
}

void bowingLoop() {
  calc_euler_angles();
  calc_bow_position();
  calc_pot_position();
  calc_playing_frequency();
  
  bow_lights();
  neck_lights();
}

void touchLoop() {
  
}

// Subroutines needed:
/*  Determine pot position
 *  Determine bow position/which string to play
 *  Determine bow velocity
 *  Turn notes on
 *  Turn notes off
 *  Light up bow and neck separately
 */

void calc_pot_position() {
  /*
   * This function determines the position on the potentiometer and
   * translates it to three different lengths in real space. 
   * 
   * This function also includes code that checks for "functions" on 
   * the softpot ie. calibration of the bow position.
   */
  
  uint8_t potaverage = 1; // How many analog reads are read to
                          // determine pot position
  
  for (int i = 0; i < potaverage; i++) {
    bridge_read = analogRead(A6); // T34 -> A6/A8
    nut_read = analogRead(A7); // T34 -> A7/A9
    bridge_ave = bridge_ave + bridge_read;
    nut_ave = nut_ave + bridge_read;
  }
  bridge_ave = bridge_ave / potaverage;
  nut_ave = nut_ave / potaverage;

  float R_nut =  (float(nut_ave)/(resolution - float(nut_ave)));
  float R_bridge =  (float(bridge_ave)/(resolution - float(bridge_ave)));

  if (bridge_ave > (0.9 * resolution)) { // this will be true if string is not fretted
    L_nut = 0.0;
    L_bridge = open_string_length;
    L_between = 0.0; 
  } else {
    // Scaling/calibration
    L_bridge = (-1.1618 * R_bridge*R_bridge + 74.069 * R_bridge - 0.2172) - 75.0 + open_string_length;
    L_nut =  5.944 * R_nut*R_nut + 66.1 * R_nut - 0.3272;
    L_between = open_string_length - (L_nut + L_bridge); 
  
    L_bridge = constrain(L_bridge, 0.0, open_string_length);
  }

  // Check for calibration condition
  if (L_bridge < (3 + (open_string_length - 75.0))) {// zeroes bow angle for standing case
    chi0 = chi; 
  }
}


void calc_euler_angles() {
  /*  
   *  This portion of the code calculates the three independent Euler angles 
   *  of the system using the quaternions that have been read from either the
   *  FXO-FXA chip or the BNO chip using the code given by Larry.
   *  See files fxa_fxo_subs.h and bno055_subs.h.
   *  
   *  Quaternion operations are done by multiplying an original quaternion, q1, by another rotating quaternion, q2,
   *  in the manner q1*q2. This means rotation is processed from "the sensor's point of view", and not by calculating
   *  absolute rotation, which would be calculated in the order q2*q1.
   */

  bno055_main_calc(); // stores values for quaternions, "plucksig" (gyro.z() value), and "lin_accelx"
  float qw, qx, qy, qz, qwprime, qxprime, qyprime, qzprime;

  qwprime = q0prime; // Use this chunk if using MPU as mounted on original bows
  qxprime = q1prime;
  qyprime = q2prime;
  qzprime = q3prime;

//  qwprime = -q2prime; // Use this chunk if using MPU as mounted on more recent plates (same side as button)
//  qxprime = -q3prime; // Rotates quaternions by 180 degrees about the Y axis
//  qyprime = q0prime;
//  qzprime = q1prime;

  qw = (sqrt(2)/2) * qwprime - (sqrt(2)/2) * qzprime; // Rotates about z-axis 90 degrees
  qx = (sqrt(2)/2) * qxprime + (sqrt(2)/2) * qyprime;
  qy = (sqrt(2)/2) * qyprime - (sqrt(2)/2) * qxprime;
  qz = (sqrt(2)/2) * qzprime + (sqrt(2)/2) * qwprime;

  phi = (360/(2*PI)) * atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy)); // convert to and store phi, theta, and chi
  theta = (360/(2*PI))*asin(2*(qw*qy - qz*qx));
  chi = (360/(2*PI))*atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz));

  delta_chi = chi - chi0;
  
  if (delta_chi < -180) {     // keep del_chi in the correct range from -180 => 180
    delta_chi = delta_chi +360;
  }

  if (delta_chi > 180) {
    delta_chi = delta_chi -360;
  }
}

void calc_bow_position() {  // Maps chi angle to one of the four strings on bass/cello
  float bowing_angle = delta_chi;
  bowing_angle = constrain(bowing_angle, -45, 60);
  bow_pos_prev = bow_pos;
  bow_pos = map(int(bowing_angle), -45, 60, 0, 3);
}

void calc_playing_frequency() { // Determines which frequencies to play based on pot
                                  // position and bow position
  float open_freq = low_string_freq * pow(string_freq_ratio, bow_pos);
  playing_freq = open_freq * (open_string_length / L_bridge);
}

void bow_lights() {
  if (bow_pos != bow_pos_prev) {
    switch (bow_pos) {
      case 0:    // red
      b_pixels.clear();
      
      b_pixels.setPixelColor(1, b_pixels.Color(50, 0, 0));
      b_pixels.setPixelColor(8, b_pixels.Color(50, 0, 0));
  
      b_pixels.show(); 
        break;
        
      case 1:    // yellow
      b_pixels.clear();
      
      b_pixels.setPixelColor(2, b_pixels.Color(50, 50, 0));
      b_pixels.setPixelColor(7, b_pixels.Color(50, 50, 0));
  
      b_pixels.show();  
        break;
        
      case 2:    // green
      b_pixels.clear();
      
      b_pixels.setPixelColor(3, b_pixels.Color(0, 50, 0));
      b_pixels.setPixelColor(6, b_pixels.Color(0, 50, 0));
  
      b_pixels.show();  
        break;
        
      case 3:    // blue
      b_pixels.clear();
      
      b_pixels.setPixelColor(4, b_pixels.Color(0, 0, 50));
      b_pixels.setPixelColor(5, b_pixels.Color(0, 0, 50));
  
      b_pixels.show(); 
        break; 
    }
  }
}

void neck_lights() {
  if (bow_pos != bow_pos_prev) {
    switch (bow_pos) {
      case 0:    // red
      n_pixels.clear();
      for (int i=0; i<n_NUMPIXELS; i++) {
        n_pixels.setPixelColor(i, n_pixels.Color(50, 0, 0));
      }
      n_pixels.show();     
        break;
        
      case 1:    // yellow
      n_pixels.clear();
      for (int i=0; i<n_NUMPIXELS; i++) {
        n_pixels.setPixelColor(i, n_pixels.Color(50, 50, 0));
      }
      n_pixels.show();     
        break;
        
      case 2:    // green
      n_pixels.clear();
      for (int i=0; i<n_NUMPIXELS; i++) {
        n_pixels.setPixelColor(i, n_pixels.Color(0, 50, 0));
      }
      n_pixels.show();   
        break;
        
      case 3:    // blue
      n_pixels.clear();
      for (int i=0; i<n_NUMPIXELS; i++) {
        n_pixels.setPixelColor(i, n_pixels.Color(0, 0, 50));
      }
      n_pixels.show();
        break;
    }
  }
}
