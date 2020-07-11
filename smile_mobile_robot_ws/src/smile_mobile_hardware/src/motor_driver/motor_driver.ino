/*
 * Author: David Pierce Walker-Howell<piercedhowell@gmail.com>
 * Date Created: 07/10/2020
 * Description: Driver for the arduino to receive serial PWM commands and 
 */
#include <Servo.h>

#define FR_PWM_PIN 6
#define FL_PWM_PIN 5
#define BR_PWM_PIN 10
#define BL_PWM_PIN 9

Servo FR_motor, FL_motor, BR_motor, BL_motor;  

//The state of receiving serial data
char receivingState = 0;


//Union between 16-bit integer and char array
typedef union
{
    int i;
    char c[2];
} intCharUnion;

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



floatIntBytes loadFloat(float val){
  floatIntBytes temp;
  temp.f = val;
  return temp;
}

void setup() {

  Serial.begin(115200);
  
  //Attach the PWM pins to control each motor using Serial Write.
  FR_motor.attach(FR_PWM_PIN);
  FL_motor.attach(FL_PWM_PIN);
  BR_motor.attach(BR_PWM_PIN);
  BL_motor.attach(BL_PWM_PIN);

  //DEBUGGING PURPOSES --> Change pin state everytime a valid frame is received.
  pinMode(
  
}

void loop() {
  /* Reads PWM values from TX2 and writes them to each motor using writePWM() */
  intCharUnion pwm1, pwm2, pwm3, pwm4;
  char startRead = 0xDB;
  char endRead= 0xBD;
  char checkStart;
  char checkEnd;
  char readByte;

  if(Serial.available() > 0){

    switch (receivingState) {
      case 0:
        readByte = Serial.read();
        if(readByte == startRead){receivingState = 1;}
        break;
      case 1:
        //Read 8 bytes for all the PWM values
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

        state = 2;
        break;
     case 2:
        readByte = Serial.read();
        if(readByte == endRead)
        {
          receivingState = 0;
          writePWM(pwm1, pwm2, pwm3, pwm4);
        }
    }
  }
  
  
}
