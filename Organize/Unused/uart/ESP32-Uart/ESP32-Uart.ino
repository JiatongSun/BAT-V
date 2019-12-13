#define RXD2 16
#define TXD2 17

char send_buffer[3] = {0,127,255};                    // this is an example of communicating three one-byte numbers with Teensy

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);      // use Serial2 for uart communication
}
 
void loop() { 
  while (Serial2.available()) {                       // read any incoming and write it to the monitor
      Serial.print("\nESP32 read: ");
      Serial.print(byte(Serial2.read()));             // convert to byte while receiving
      Serial.print(" ");
  }
  if (millis()%1000==1){  
    Serial.print("\nESP32 write: ");
    for(int i=0; i<3; ++i){                           // receive data one at a time  
        Serial2.print(send_buffer[i]); 
        Serial.print(byte(send_buffer[i]));           // convert to byte while sending  
        Serial.print(" ");
        send_buffer[i]++;                             // changing the number for communucating
    }
    delay(1);
  }
}
