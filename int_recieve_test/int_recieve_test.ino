#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#include <LiquidCrystal.h>
LiquidCrystal lcd(22, 24, 52, 50, 48, 46);

int data[8];
int i;
int counter = 0;
int res[8];
byte input[2];
int thing;

union Sharedblock
{
    char part[4];
    float data;
} my_block;

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
  lcd.begin(16, 2);
  lcd.print("fgt!");
  float temp = 5.5;
  my_block.data = temp;
}

void loop(){
    
    if (Serial.available() > 0){  
        input[counter] = Serial.read();
        counter++;
    }

    if(counter == 2){
        lcd.clear();
        thing =(input[0] & 0xff) | (input[1] << 8 );
        lcd.print(thing);
        counter = 0;

        my_block.data = (float)3.14;
        for(int i=0;i<4;++i)
        {
            Serial.write(my_block.part[i]);
        }
        
        }

}
