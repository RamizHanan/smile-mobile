/*
 * Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
 * Date Created: 07/10/2020
 * Description: Driver for the arduino to receive serial PWM commands and 
 */
#include <Servo.h>

//Motor controller PWM pins
#define FR_PWM_PIN 6
#define FL_PWM_PIN 5
#define BR_PWM_PIN 10
#define BL_PWM_PIN 9

//Servo Library is used to control the motor controllers
Servo FR_motor, FL_motor, BR_motor, BL_motor;  

//The state of receiving serial data
char receivingState = 0;
bool debugPinState = 0; //FOR DEBUG PURPOSES.

//Union between 16-bit integer and char array
typedef union
{
    int i;
    char c[2];
} intCharUnion;

/*
 * Write the PWM values received to the motor controller. Remap the PWM values from -255, 255 --> 0 180
 */
void writePWM(int pwm1,int pwm2,int pwm3,int pwm4){
  Serial.write(pwm1);
  //Remap the PWM signals from -255, 255 to 0, 180 for the servo library
  //Sum multiplied by a negative to account for motor directions
  pwm1 = (int)map(pwm1, -255, 255, 0, 180);
  pwm2 = (int)map(pwm2, -255, 255, 0, 180);
  pwm3 = (int)map(pwm3, -255, 255, 0, 180);
  pwm4 = (int)map(-1 * pwm4, -255, 255, 0, 180); //4th wheel is flipped physically

  FL_motor.write(pwm1);
  FR_motor.write(pwm2);
  BR_motor.write(pwm3);
  BL_motor.write(pwm4);
}

void setup() {

  Serial.begin(115200);
  
  //Attach the PWM pins to control each motor using Serial Write.
  FR_motor.attach(FR_PWM_PIN);
  FL_motor.attach(FL_PWM_PIN);
  BR_motor.attach(BR_PWM_PIN);
  BL_motor.attach(BL_PWM_PIN);

  //DEBUGGING PURPOSES --> Change pin state everytime a valid frame is received.
  pinMode(11, OUTPUT);
  
}

void loop() {
  /* Reads PWM values from TX2 and writes them to each motor using writePWM() */
  intCharUnion pwm1, pwm2, pwm3, pwm4;
  char startRead = 0xDB;
  char endRead = 0xBD;
  char checkStart;
  char checkEnd;
  char readByte[0];

  //Wait for all 10 bytes to be available before reading.
  //This proved to give more consistent results
  if(Serial.available() > 0){
    
    switch (receivingState) {
      case 0:
        Serial.readBytes(readByte, 1);
        if(readByte[0] == startRead){receivingState = 1;}
        else {receivingState = 0;}
        break;
      case 1:
        //Read 2 bytes each for the 4 PWM values
        //MUST use readBytes instead of read.
        Serial.readBytes(pwm1.c, 2);
        Serial.readBytes(pwm2.c, 2);
        Serial.readBytes(pwm3.c, 2);
        Serial.readBytes(pwm4.c, 2);

        receivingState = 2;
        break;
     case 2:
        Serial.readBytes(readByte, 1); 
        if(readByte[0] == endRead) //Successful receive of a datapacket.
        {
          writePWM(pwm1.i, pwm2.i, pwm3.i, pwm4.i);
          receivingState = 0;
        
          //FOR DEBUG PURPOSES (COMMENT OUT IN IMPLEMENTATION)
//          if(pwm3.i == -152){
//          
//          digitalWrite(11, debugPinState);
//          debugPinState = !debugPinState;
//          
//          }
        }
        else
          receivingState = 0;
        break;
    }
  }

  //Having a delay is SUPER necessay! The arduino needs a little rest time from execution for buffers to fill properly
  delay(1); 
  
}
