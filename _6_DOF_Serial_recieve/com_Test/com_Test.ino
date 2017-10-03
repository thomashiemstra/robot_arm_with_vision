uint8_t check;
int16_t ticks[8];
uint8_t bytes[16];

void setup() {
    Serial.begin(115200); 
    pinMode(LED_BUILTIN, OUTPUT);
    check = 0;
    for(int i = 0; i<8; i++){
            ticks[i] = 255+ i*i;
        }
}

void loop() {
    digitalWrite(LED_BUILTIN, LOW);  
    if (Serial.available() > 0){  
        
        check = Serial.read();
     
       if (check == 1){
                for(int i =0; i < 8; i++){
                    bytes[2*i] = (ticks[i] >> 8) &0xff;
                    bytes[2*i+1] = ticks[i] & 0xff;
                }    
                Serial.write(bytes,16);
            }
    }

    
    

    

}
