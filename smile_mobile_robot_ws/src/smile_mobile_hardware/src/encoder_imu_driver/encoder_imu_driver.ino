/*
 * Authors: David Pierce Walker-Howell <piercedhowell@gmail.com>
 *          Ramiz Hanan <ramizhanan@gmail.com>
 * Date Created: 07/12/2020
 * Description: This is a driver code for the arduino nano that collects the encoder 
 *              and IMU data. It communicates to the main computer via serial.
 * 
 */

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Set up IMU 
// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);


//The Nano Interrupt pins the encoder is connected to
#define ENCODER_A_1 2
#define ENCODER_A_2 3
#define ENCODER_B_1 4
#define ENCODER_B_2 5

//Gear ratio of internal motor shaft to external shaft.
const float GEAR_RATIO = 3.0;

typedef union{
  float f;
  char c[4];
} floatBytes;

typedef struct{
  float shaft1Freq;
  float shaft2Freq;
} shaftFrequencies;


//Keep timing for frequency estimation.
unsigned long prevTime = 0, currTime = 0;

//The HIGH or LOW state of each encoder channel pulse.
volatile bool encoderA1State, encoderA2State;
volatile bool encoderB1State, encoderB2State;

//Counts for each time the interrupt occurs for a reading on encoder A1.
volatile long currEncoder1Count = 0.0, currEncoder2Count = 0.0;
long lastEncoder1Count = 0.0, lastEncoder2Count = 0.0;

//Start and end bytes for sending data to computer
char START_BYTE = 0xDA;
char END_BYTE = 0xAD;

void setup() {
  // put your setup code here, to run once:

  //Encoder setup (pin connections)
  pinMode(ENCODER_A_1, INPUT_PULLUP);
  pinMode(ENCODER_A_2, INPUT_PULLUP);
  pinMode(ENCODER_B_1, INPUT_PULLUP);
  pinMode(ENCODER_B_2, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_1), interruptEncoderA1, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_2), interruptEncoderA2, CHANGE);
  Serial.begin(115200);
  
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);
}

void loop() {
  // put your main code here, to run repeatedly:

  //Get the Encoder data --> Convert it from float to bytes
  shaftFrequencies shaftFreqs;
  shaftFreqs = getMotorShaftFrequencies();
  floatBytes shaft1, shaft2;
  shaft1.f = shaftFreqs.shaft1Freq;
  shaft2.f = shaftFreqs.shaft2Freq;
  
  // Get IMU data
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  imu::Vector<3> acc = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  // convert IMU data to bytes (using a union struct)
  floatBytes accX, accY, accZ, gyroX, gyroY, gyroZ, roll, pitch, heading;
  accX.f = acc.x(); // X Acceleration m/s^2
  accY.f = acc.y(); // Y Acceleration m/s^2
  accZ.f = acc.z(); // Z Acceleration m/s^2
  gyroX.f = gyro.x(); // X Gyro rad/s
  gyroY.f = gyro.y(); // Y Gyro rad/s
  gyroZ.f = gyro.z(); // Z Gyro rad/s
  roll.f = euler.z();
  pitch.f = euler.y();
  heading.f = euler.x(); // Absoluate Heading Degrees (sensor fused) Points West?

  //Send the encoder and IMU data
  Serial.write(START_BYTE);
  
  Serial.write(shaft1.c, 4); //4bytes
  Serial.write(shaft2.c, 4); //4bytes

  Serial.write(accX.c, 4);
  Serial.write(accY.c, 4);
  Serial.write(accZ.c, 4);

  Serial.write(gyroX.c, 4);
  Serial.write(gyroY.c, 4);
  Serial.write(gyroZ.c, 4);

  Serial.write(roll.c, 4);
  Serial.write(pitch.c, 4);
  Serial.write(heading.c, 4);

  Serial.write(END_BYTE);
  
  delay(10);
}

//void transmitShaftFrequencies(){
//  //Transmit the shaft frequecies of each motor..
//  floatToIntBytes fToi1, fToi2;
//  shaftFrequencies shaftFreqs;
//  shaftFreqs = getMotorShaftFrequencies();
//  fToi1.f = shaftFreqs.shaft1Freq;
//  fToi2.f = shaftFreqs.shaft2Freq;
//  Wire.write(fToi1.c[0]);
//  Wire.write(fToi1.c[1]);
//  Wire.write(fToi1.c[2]);
//  Wire.write(fToi1.c[3]);
//
//  Wire.write(fToi2.c[0]);
//  Wire.write(fToi2.c[1]);
//  Wire.write(fToi2.c[2]);
//  Wire.write(fToi2.c[3]);
//}

shaftFrequencies getMotorShaftFrequencies(){

  //Frequencies of encoder and shaft
  //shaftFreq = encoderFreq / (16 * GEAR_RATIO);
  float encoder1Freq, encoder2Freq;
  shaftFrequencies shaftFreqs;
  float dt;
  
  currTime = micros();
  
  dt = currTime - prevTime;
  encoder1Freq = ((float)(currEncoder1Count - lastEncoder1Count) / (dt/1000000.0));
  encoder2Freq = ((float)(currEncoder2Count - lastEncoder2Count) / (dt/1000000.0));
  shaftFreqs.shaft1Freq = (encoder1Freq) / (16 * GEAR_RATIO);
  shaftFreqs.shaft2Freq = (encoder2Freq) / (16 * GEAR_RATIO);
  lastEncoder1Count = currEncoder1Count;
  lastEncoder2Count = currEncoder2Count;

  prevTime = micros();
  return(shaftFreqs);
  
}

void interruptEncoderA1(){
  encoderA1State = digitalRead(ENCODER_A_1);
  encoderB1State = digitalRead(ENCODER_B_1);
  
  if((encoderA1State == 1)){
    if(encoderB1State == 1){
      currEncoder1Count++;
    }
    else{
      currEncoder1Count--;
    }
  }

  else{
    if(encoderB1State == 1){
      currEncoder1Count--;
    }
    else{
      currEncoder1Count++;
    }
  }

}

void interruptEncoderA2(){
  encoderA2State = digitalRead(ENCODER_A_2);
  encoderB2State = digitalRead(ENCODER_B_2);
  
  if((encoderA2State == 1)){
    if(encoderB2State == 1){
      currEncoder2Count++;
    }
    else{
      currEncoder2Count--;
    }
  }

  else{
    if(encoderB2State == 1){
      currEncoder2Count--;
    }
    else{
      currEncoder2Count++;
    }
  }

}
