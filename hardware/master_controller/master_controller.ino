#include <Wire.h>

#define SLAVE_ADDRESS_1 4
#define SLAVE_ADDRESS_2 5

typedef union{
  float f;
  int i;
  char c[4];
} floatToIntBytes;

typedef struct{
  float shaft1Freq;
  float shaft2Freq;
} shaftFrequencies;

void setup() {
  //Initialize Master I2C.
  Wire.begin(); //No address needed for master.

  //Initialize serial
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  shaftFrequencies motors12, motors34;

  motors12 = getShaftFrequencies(SLAVE_ADDRESS_1);
  motors34 = getShaftFrequencies(SLAVE_ADDRESS_2);
  
  Serial.print("Motor 1: ");
  Serial.println(motors12.shaft1Freq);
  Serial.print("Motor 2: ");
  Serial.println(motors12.shaft2Freq);
  Serial.print("Motor 3: ");
  Serial.println(motors34.shaft1Freq);
  Serial.print("Motor 4: ");
  Serial.println(motors24.shaft2Freq);
  delay(10);
}

shaftFrequencies getShaftFrequencies(int slaveAddress){
  /*
   * Get the shaft frequencies from the two motors.
   */
  shaftFrequencies shaftFreqs;

  Wire.requestFrom(slaveAddress, 8); //Request for bytes of data from encoder.

  int counter = 0;
  floatToIntBytes fToi1, fToi2;
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
