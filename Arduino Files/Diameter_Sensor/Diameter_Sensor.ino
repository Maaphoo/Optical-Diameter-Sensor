//Diameter sensor UI by Matt Rogge August 25, 2016
//Based on example from http://playground.arduino.cc/Main/TSL1402R


#include <Wire.h>

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


// Define various ADC prescaler:
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
int CLKpin = 3;    // <-- Arduino pin delivering the clock pulses to pin 3 (CLK) of the TSL1402R
int SIpin = 2;     // <-- Arduino pin delivering the SI (serial-input) pulse to pin 2 & 10 of the TSL1402R
int AOpin1 = 1;    // <-- Arduino pin connected to pin 4 (analog output 1)of the TSL1402R
int AOpin2 = 2;    // <-- Arduino pin connected to pin 8 (analog output 2)of the TSL1402R
int laserPin = 12; // <-- the pin that the laser is connected to.


//Data used over IIC

//A map containing the address and length in bytes of each of the regester values
//Each address in broken in to two bytes.
void* regMap[50][2]; 
float diameter;//The diameter measured by the diameter sensor.
boolean laserPower;
int numSamples;
float pixelPitch = 0.0635; //the pitch of the CCD array
float yOffset = 680.0;
int integrationTime = 10;
int threshold = 200;//The intensity difference between adjacent points above which is considered a significant change.
int pixelIndex = 10;//The number of pixels to omit from the sensor. Some times the pixels at the edge of the sensor don't read correctly.
boolean sendRawData;//If true all of the raw sensor data is sent via IIC for each diameter  measurement.
boolean sendIntermediateData;//If true, all intermediate data is sentvia IIC for each diameter measurement
int version;//The version of the diameter sensor's firmware.

//The following are read only values produced by the sensor and the algorithm for determining the diameter
int adcVals[256];// The array where the readout of the photodiodes is stored, as integers
float leftSlope;
float leftInt;
float rightSlope;
float rightInt;
float leftX; //The x position of the left side of the shadow as calculated using the line and the y offset.
float rightX;//The x position of the right side of the shadow as calculated using the line and the y offset.
int maxGapLeft;
int maxGapRight;
int maxGapLeftIndex;
int maxGapRightIndex;
int leftShadowLine[8][2] {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int rightShadowLine[8][2] {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};


void setup()
{
  // Initialize two Arduino pins as digital output:
  pinMode(CLKpin, OUTPUT);
  pinMode(SIpin, OUTPUT);
  pinMode(laserPin, OUTPUT);
  // To set up the ADC, first remove bits set by Arduino library, then choose
  // a prescaler: PS_16, PS_32, PS_64 or PS_128:
  ADCSRA &= ~PS_128;
  ADCSRA |= PS_32; // <-- Using PS_32 makes a single ADC conversion take ~30 us
  //// set prescale to 16
  //sbi(ADCSRA,ADPS2) ;
  //cbi(ADCSRA,ADPS1) ;
  //cbi(ADCSRA,ADPS0) ;

  // Next, assert default setting:
  analogReference(DEFAULT);

  // Set all IO pins low:
  for ( int i = 0; i < 14; i++ )
  {
    digitalWrite(i, LOW);
  }

  // Clock out any existing SI pulse through the ccd register:
  for (int i = 0; i < 260; i++)
  {
    ClockPulse();
  }

  // Create a new SI pulse and clock out that same SI pulse through the sensor register:
  digitalWrite(SIpin, HIGH);
  ClockPulse();
  digitalWrite(SIpin, LOW);
  for (int i = 0; i < 260; i++)
  {
    ClockPulse();
  }
  digitalWrite(laserPin, HIGH);//turn on the laser

  Serial.begin(115200);

  //Set up the register map
  regMap[0][0] = &diameter;
  regMap[0][1] = sizeof(diameter);

//  regMap [1][0] = ;
//  regMap [1][1] = ;
  
//  float diameter;//The diameter measured by the diameter sensor.
//int numSamples;
//int version;
//boolean laserPower;
//float pixelPitch = 0.0635; //the pitch of the CCD array
//float yOffset = 680.0;
//int integrationTime = 10;
//int threshold = 200;//The intensity difference between adjacent points above which is considered a significant change.
//int pixelIndex = 10;//The number of pixels to omit from the sensor. Some times the pixels at the edge of the sensor don't read correctly.
//boolean sendRawData;//If true all of the raw sensor data is sent via IIC for each diameter  measurement.
//boolean sendIntermediateData;//If true, all intermediate data is sentvia IIC for each diameter measurement

//  float slopeAndInt[2];
//  int Vals[2][2] = {1,1,2,3};
//  linReg(slopeAndInt, Vals);
//  Serial.println(slopeAndInt[0]);
}

void loop()
{
  // Stop the ongoing integration of light quanta from each photodiode by clocking in a SI pulse
  // into the sensors register:
  digitalWrite(SIpin, HIGH);
  ClockPulse();
  digitalWrite(SIpin, LOW);

  // Next, read all 256 pixels in parallell. Store the result in the array. Each clock pulse
  // causes a new pixel to expose its value on the two outputs:
  for (int i = 0; i < 128; i++)
  {
    //delayMicroseconds(20);// <-- We add a delay to stabilize the AO output from the sensor
    adcVals[i] = analogRead(AOpin1);
    adcVals[i + 128] = analogRead(AOpin2);
    ClockPulse();
  }

  // Next, stop the ongoing integration of light quanta from each photodiode by clocking in a
  // SI pulse:
  digitalWrite(SIpin, HIGH);
  ClockPulse();
  digitalWrite(SIpin, LOW);
  // Next, send the measurement stored in the array to host computer using serial (rs-232).
  // communication. This takes ~80 ms during whick time no clock pulses reaches the sensor.
  // No integration is taking place during this time from the photodiodes as the integration
  // begins first after the 18th clock pulse after a SI pulse is inserted:
//  for (int i = 0; i < 256; i++)
//  {
//    Serial.print(i);
//    Serial.print(" ");
//    Serial.print(adcVals[i]); //Serial.print(" ");
//    Serial.print('\n');
//  }
  Serial.print("Diameter: ");
  Serial.print(getDiameter());
  
  Serial.println();// <-- Send a linebreak to indicate the measurement is transmitted.
  // Next, a new measuring cycle is starting once 18 clock pulses have passed. At
  // that time, the photodiodes are once again active. We clock out the SI pulse through
  // the 256 bit register in order to be ready to halt the ongoing measurement at our will
  // (by clocking in a new SI pulse):

  for (int i = 0; i < 260; i++)
  {
    if (i == 18)
    {
      // Now the photodiodes goes active..
      // An external trigger can be placed here
      //Turn on laser
    }


    ClockPulse();
  }

  // The integration time of the current program / measurement cycle is ~3ms. If a larger time
  // of integration is wanted, uncomment the next line:
  delayMicroseconds(integrationTime);// <-- Add 20 us integration time
}

// This function generates an outgoing clock pulse from the Arduino digital pin 'CLKpin'. This clock
// pulse is fed into pin 3 of the linear sensor:
void ClockPulse()
{
  //delayMicroseconds(20);
  PORTD = PORTD | B00001000;
  PORTD = PORTD & B11110111;
  //digitalWrite(CLKpin, HIGH);
  //digitalWrite(CLKpin, LOW);
}

float getDiameter() {
  


  //scan through the values from the ccd to find where the edges of the shadows are
  //Omit the first and last few pixels because the edges are un reliable

  for (pixelIndex; pixelIndex < 250; pixelIndex ++) {
    if (adcVals[pixelIndex] - adcVals[pixelIndex + 1] > maxGapLeft) {
      maxGapRight = maxGapLeft;
      maxGapRightIndex = maxGapLeftIndex;
      maxGapLeft = adcVals[pixelIndex] - adcVals[pixelIndex+1] ;
      maxGapLeftIndex = pixelIndex;
    }
    if (adcVals[pixelIndex+1] - adcVals[pixelIndex] > maxGapRight) {
      maxGapRight = adcVals[pixelIndex+1] - adcVals[pixelIndex];
      maxGapRightIndex = pixelIndex;
    }
  }
    
  //Find large gaps around the largest gap
  int leftShadowPoint = 0;//the index of the left Shadow point in the array of significant left shadow points
  for (int i=maxGapLeftIndex-4;i<maxGapLeftIndex+4;i++){
    if (adcVals[i]-adcVals[i+1] > threshold){
      leftShadowLine[leftShadowPoint][0] = i;
      leftShadowLine[leftShadowPoint][1] = adcVals[i];
      leftShadowPoint++;
    } else if (adcVals[i]-adcVals[i+1] < threshold && i>maxGapLeftIndex){//grab the other side of the last significant line segment.
      leftShadowLine[leftShadowPoint][0] = i;
      leftShadowLine[leftShadowPoint][1] = adcVals[i];
      break;
    }
  }
  
  //Find large gaps around the largest gap
  int rightShadowPoint = 0;//the index of the left Shadow point in the array of significant left shadow points  
  for (int i=maxGapRightIndex-4;i<maxGapRightIndex+4;i++){
    if (adcVals[i+1]-adcVals[i] > threshold){
      rightShadowLine[rightShadowPoint][0] = i;
      rightShadowLine[rightShadowPoint][1] = adcVals[i];
      rightShadowPoint++;
    } else if (adcVals[i+1]-adcVals[i] < threshold && i>maxGapRightIndex){
      rightShadowLine[rightShadowPoint][0] = i;
      rightShadowLine[rightShadowPoint][1] = adcVals[i];
      break;
    }
  }

  //find slope of selected transition points
  float leftSlopeAndInt[2];
  float rightSlopeAndInt[2];

  linReg(&leftSlope,&leftInt,leftShadowLine);
  linReg(&rightSlope, &rightInt,rightShadowLine);

  //Determine left x value
  leftX = (yOffset-leftSlopeAndInt[1])/leftSlopeAndInt[0];
  rightX = (yOffset-rightSlopeAndInt[1])/rightSlopeAndInt[0];
  diameter = (rightX-leftX)*pixelPitch;
  
//  Serial.print("maxGapLeft: ");
//  Serial.print(maxGapLeftIndex);
//  Serial.print("  ");
//  Serial.println(maxGapLeft);
//  Serial.print("maxGapRight: ");
//  Serial.print(maxGapRightIndex);
//  Serial.print("  ");
//  Serial.println(maxGapRight);
//  
//  for (int i = 0; i<8;i++) {
//    Serial.print("Left Significant Point: ");
//    Serial.print(leftShadowLine[i][0]);
//    Serial.print(" ");
//    Serial.println(leftShadowLine[i][1]);
//  }
//  for (int i = 0; i<8;i++) {
//    Serial.print("Right Significant Point: ");
//    Serial.print(rightShadowLine[i][0]);
//    Serial.print(" ");
//    Serial.println(rightShadowLine[i][1]);
//  }
//  
//  Serial.println("\n\n\n");
  return diameter;
}

void  linReg(float* slope, float* intercept, int Vals[][2]) {
  //Variables
  int n;
  float sumX = 0.0, sumY = 0.0, sumXY = 0.0, sumXX = 0.0, sumYY = 0.0;
  float sXY, sXX;

  //Determine n
  for (int i=0;i<8;i++){
    n=i;
    if (Vals[i][0] == 0){
      break;
    }
  }
//  Serial.print("n: ");
//  Serial.println(n);
//  
  //Find Sums
  for (int i = 0; i < n; i++) {
    sumX = sumX + float(Vals[i][0]);
    sumY = sumY + float(Vals[i][1]);
    sumXY = sumXY + float(Vals[i][0]) * float(Vals[i][1]);
    sumXX = sumXX + float(Vals[i][0]) * float(Vals[i][0]);
    sumYY = sumYY + float(Vals[i][1]) * float(Vals[i][1]);
  }

  //Compute slope and intercept
  sXY = sumXY - sumX * sumY / float(n);
  sXX = sumXX - sumX * sumX / float(n);
  *slope = sXY / sXX;
  *intercept = sumY / float(n) - *slope * (sumX / float(n));

//  //Debug
//  Serial.println("Data (x,y):");
//  for (int i = 0; i < n; i++) {
//    Serial.print(Vals[i][0]);
//    Serial.print(", ");
//    Serial.println(Vals[i][1]);
//  }
//  Serial.println();
//
//  Serial.print("sumX: ");
//  Serial.println(sumX);
//  Serial.print("sumY: ");
//  Serial.println(sumY);
//  Serial.print("sumXY: ");
//  Serial.println(sumXY);
//  Serial.print("sumXX: ");
//  Serial.println(sumXX);
//  Serial.print("sumYY: ");
//  Serial.println(sumYY);
//  Serial.print("sXY: ");
//  Serial.println(sXY);
//  Serial.print("sXX: ");
//  Serial.println(sXX);
//  Serial.print("slope: ");
//  Serial.println(slopeAndInt[0], 6);
//  Serial.print("Intercept: ");
//  Serial.println(slopeAndInt[1], 6);
//  Serial.println();
}

void receiveEvent (int bytesRecieved){
  byte* bytePtr;
  byte command = Wire.read();//the address of the register to be read
  boolean readOrWrite = command & B00000001;//If the zeroith place is 0 it is read, 1 is write.
  byte address = command >> 1; //Shifting the address one to the right removes the read/write command and yields the address of the regester

  
  void* itemPtr = regMap[address][0];
  int  itemLength = int(regMap[address][1]);
  
  for (byte i = 0;i<itemLength;i++){
//    Wire.write(itemPtr);
    itemPtr+=1;
  }
}

