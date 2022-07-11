/*
  AnalogReadSerial

  Reads an analog input on pin 0, prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogReadSerial
*/
float read_raw1;
float read_raw2;
float read_raw1_ave;
float read_raw2_ave;
int flag = 0;
// the setup routine runs once when you press reset:
void setup(void) {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  analogReadResolution(12);
  analogReadAveraging(8);
  delay(500);
}

// the loop routine runs over and over again forever:
void loop(void) {
  // read the input on analog pin 0:
    potcalc();
  delay(10);        // delay in between reads for stability
}


void potcalc(){

      read_raw1 = analogRead(A6);
//      Serial.println(read_raw1);
//Serial.println(flag);
  // print out the value you read:
  if ((read_raw1 < 4000)&&(flag == 0)){
    delay(100);
    read_raw1_ave = 0.0;
    read_raw2_ave = 0.0;  
    for(int i = 0; i <16; i++){      
    read_raw1 = analogRead(A6);
    read_raw2 = analogRead(A7);
//    Serial.print(read_raw1);
//    Serial.print("   ");   
//    Serial.println(read_raw2);
    read_raw1_ave = read_raw1_ave + read_raw1;
    read_raw2_ave = read_raw2_ave + read_raw2;
}
    read_raw1_ave = read_raw1_ave/16.0;
    read_raw2_ave = read_raw2_ave/16.0;
    Serial.println(read_raw1_ave);
    //Serial.print("   ");   
    //Serial.println(read_raw2_ave);
    flag = 1;

  }
if (read_raw1 > 4000){


  flag = 0;
}
  
}
