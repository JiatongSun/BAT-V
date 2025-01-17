/*
 * ESP32-UART
 *  test routine for SERIAL2 (using pin 16 and 17)
 * output should look something like
 *  ESP32 write 1
 *  teensy: 1teensy teensy
 *  ESP32 write 2
 *  teensy: 2teensy teensy
 *  ...
 */
#define RXD2 16
#define TXD2 17

char send_buffer[3] = {0,127,255};

void setup() {
  Serial.begin(115200);
  //  Serial2.begin(baud-rate, protocol, RX pin, TX pin);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
}
 
void loop() { 
  while (Serial2.available()) { // read any incoming and write it to the monitor
      Serial.print("\nESP32 read: ");
      Serial.print(byte(Serial2.read()));
      Serial.print(" ");
  }
  if (millis()%1000==1){  
    Serial.print("\nESP32 write: ");
    for(int i=0; i<3; ++i){
        Serial2.print(send_buffer[i]); 
        Serial.print(byte(send_buffer[i]));
        Serial.print(" ");
        send_buffer[i]++;
    }
    delay(1);
  }
}
