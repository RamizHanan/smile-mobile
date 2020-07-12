
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>  
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


// Set up IMU 
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

//Address set for the I2C slave arduino reading the encoders
#define SLAVE_ADDRESS_1 4

#define FR_PWM_PIN 6
#define FL_PWM_PIN 5
#define BR_PWM_PIN 10
#define BL_PWM_PIN 9

Servo FR_motor, FL_motor, BR_motor, BL_motor;  

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);


typedef union
{
    float f;
    int i;
    char c[4];
} floatIntBytes;

typedef struct
{
    float shaft1Freq;
    float shaft2Freq;
} shaftFrequencies;



void writePWM(int pwm_1,int pwm_2,int pwm_3,int pwm_4){
  //Remap the PWM signals from -255, 255 to 0, 180 for the servo library
  //Sum multiplied by a negative to account for motor directions
  pwm_1 = (int)map(pwm_1, -255, 255, 0, 180);
  pwm_2 = (int)map(pwm_2, -255, 255, 0, 180);
  pwm_3 = (int)map(pwm_3, -255, 255, 0, 180);
  pwm_4 = (int)map(-1 * pwm_4, -255, 255, 0, 180); //4th wheel is flipped physically

  FL_motor.write(pwm_1);
  FR_motor.write(pwm_2);
  BR_motor.write(pwm_3);
  BL_motor.write(pwm_4);
}

void readWritePWM(){
  /* Reads PWM values from TX2 and writes them to each motor using writePWM() */
  floatIntBytes pwm1, pwm2, pwm3, pwm4;
  char startRead = 0xDB;
  char endRead= 0xBD;
  char checkStart;
  char checkEnd;
 
  if(Serial.available() > 0){
    checkStart = Serial.read();
    if(checkStart == startRead){
      pwm1.c[0] = Serial.read();
      pwm1.c[1] = Serial.read();
      pwm1.c[2] = Serial.read();
      pwm1.c[3] = Serial.read();   

      pwm2.c[0] = Serial.read();
      pwm2.c[1] = Serial.read();
      pwm2.c[2] = Serial.read();
      pwm2.c[3] = Serial.read();      
      
      pwm3.c[0] = Serial.read();
      pwm3.c[1] = Serial.read();
      pwm3.c[2] = Serial.read();
      pwm3.c[3] = Serial.read();      
      
      pwm4.c[0] = Serial.read();
      pwm4.c[1] = Serial.read();
      pwm4.c[2] = Serial.read();
      pwm4.c[3] = Serial.read();      
      checkEnd = Serial.read();
      if(checkEnd == endRead){
        writePWM(pwm1.i,pwm2.i,pwm3.i,pwm4.i);
        }
      }
    } 
}

shaftFrequencies getShaftFrequencies(int slaveAddress){
  /*
   * Get the shaft frequencies from the two motors.
   */
  shaftFrequencies shaftFreqs;

  Wire.requestFrom(slaveAddress, 8); //Request for bytes of data from encoder.

  int counter = 0;
  floatIntBytes fToi1, fToi2;
  fToi1.i = 0; fToi2.i = 0;
  
  while(Wire.available()){
    if(counter < 4)
      fToi1.c[counter] = Wire.read();
    else
      fToi2.c[counter - 4] = Wire.read();
    counter++;
    if(counter >= 8) break; 
  }

  shaftFreqs.shaft1Freq = fToi1.f;
  shaftFreqs.shaft2Freq = fToi2.f;
  return(shaftFreqs);
}

floatIntBytes loadFloat(float val){
  floatIntBytes temp;
  temp.f = val;
  return temp;
}

void setup() {
  
  //Initialize Master I2C.
  Wire.begin(); //No address needed for master.

  Serial.begin(115200);
  //Serial.begin(9600);

  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  bno.setExtCrystalUse(true);

   
  //Attach the PWM pins to control each motor using Serial Write.
  FR_motor.attach(FR_PWM_PIN);
  FL_motor.attach(FL_PWM_PIN);
  BR_motor.attach(BR_PWM_PIN);
  BL_motor.attach(BL_PWM_PIN);
  
}

void loop() {
  
  char startDataWrite = 0xDA;
  char endDataWrite = 0xAD;
  readWritePWM();
  
  // Get encoder info
  shaftFrequencies motors12 = getShaftFrequencies(SLAVE_ADDRESS_1);
  
  // convert Encoder Data to bytes
  floatIntBytes motors12Shaft1Freq = loadFloat(motors12.shaft1Freq);
  floatIntBytes motors12Shaft2Freq = loadFloat(motors12.shaft2Freq);
  
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
  
  // convert IMU data to bytes
  floatIntBytes accX = loadFloat(acc.x()); // X Acceleration m/s^2
  floatIntBytes accY = loadFloat(acc.y()); // Y Acceleration m/s^2
  floatIntBytes accZ = loadFloat(acc.z()); // Z Acceleration m/s^2
  floatIntBytes gyroX = loadFloat(gyro.x()); // X Gyro rad/s
  floatIntBytes gyroY = loadFloat(gyro.y()); // Y Gyro rad/s
  floatIntBytes gyroZ = loadFloat(gyro.z()); // Z Gyro rad/s
  floatIntBytes heading = loadFloat(euler.x()); // Absoluate Heading Degrees (sensor fused) Points West?


  // Transmit serial package
  Serial.write(startDataWrite); //1 byte
  
  Serial.write(motors12Shaft1Freq.c[0]); //4 bytes
  Serial.write(motors12Shaft1Freq.c[1]); 
  Serial.write(motors12Shaft1Freq.c[2]); 
  Serial.write(motors12Shaft1Freq.c[3]);

  Serial.write(motors12Shaft2Freq.c[0]); //4 bytes
  Serial.write(motors12Shaft2Freq.c[1]); 
  Serial.write(motors12Shaft2Freq.c[2]); 
  Serial.write(motors12Shaft2Freq.c[3]); 

  Serial.write(accX.c[0]); //4 bytes
  Serial.write(accX.c[1]); 
  Serial.write(accX.c[2]); 
  Serial.write(accX.c[3]); 

  Serial.write(accY.c[0]); //4 bytes
  Serial.write(accY.c[1]); 
  Serial.write(accY.c[2]); 
  Serial.write(accY.c[3]); 

  Serial.write(accZ.c[0]); //4 bytes
  Serial.write(accZ.c[1]); 
  Serial.write(accZ.c[2]); 
  Serial.write(accZ.c[3]);
  
  Serial.write(heading.c[0]); //  4 bytes
  Serial.write(heading.c[1]); //  
  Serial.write(heading.c[2]); //  
  Serial.write(heading.c[3]); //
  
  Serial.write(gyroX.c[0]); //4 bytes
  Serial.write(gyroX.c[1]); 
  Serial.write(gyroX.c[2]); 
  Serial.write(gyroX.c[3]); 

  Serial.write(gyroY.c[0]); //4 bytes
  Serial.write(gyroY.c[1]); 
  Serial.write(gyroY.c[2]); 
  Serial.write(gyroY.c[3]); 

  Serial.write(gyroZ.c[0]); //4 bytes
  Serial.write(gyroZ.c[1]); 
  Serial.write(gyroZ.c[2]); 
  Serial.write(gyroZ.c[3]); 
  
  Serial.write(endDataWrite); // 1 byte
  
  delay(BNO055_SAMPLERATE_DELAY_MS);
}