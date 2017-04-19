#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#include <LiquidCrystal.h>
LiquidCrystal lcd(22, 24, 52, 50, 48, 46);

int data[8];
int i;
int counter = 0;
uint8_t input[16];
int res[8];

int readStuff(){
  byte MSB = 0;  // to build  2 byte integer from serial in byte
  byte LSB = 0;  // to build  2 byte integer from serial in byte
  short   MSBLSB = 0;  //to build  2 byte integer from serial in byt
  MSB = Serial.read();
  LSB = Serial.read();
  MSBLSB=word(MSB, LSB);  
  return MSBLSB;
}

void setup(){
   Serial.begin(115200); 
  pwm.begin();
  pwm.setPWMFreq(100);
  lcd.begin(16, 2);
  lcd.print("ready for action!");
}

void loop(){
    if (Serial.available() > 0){  
        input[counter] = Serial.read();
        counter++;
    }
    
    if( counter == 16){
//        lcd.clear();
        for(i = 0; i < 8; i++){
             res[i] = (input[2*i] << 8 ) | (input[2*i+1] & 0xff);
             pwm.setPWM(i, 0, res[i]);
        }
//        lcd.clear();
//        lcd.setCursor(0,0);
//        lcd.print(res[2]);
//        pwm.setPWM(6,0,res[6]);
        counter = 0;
    }
}


