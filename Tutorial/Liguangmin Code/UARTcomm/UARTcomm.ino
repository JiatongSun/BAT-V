#define RXD2 16
#define TXD2 17

const int UARTBUFFERSIZE = 100;
char UARTBUFFER[UARTBUFFERSIZE];
int HeadX;
int HeadY;
int TailX;
int TailY;
void sendPacket();
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(115200,SERIAL_8N1,RXD2,TXD2);

}

int i;
void loop() {
  while (Seiral2.available())
  {
    Serial.print(char(Serial2.read()));
  }
  // put your main code here, to run repeatedly:
  if (millis()%1000==1)
  {
    Serial2.println(i++);
    Serial.printf("ESP32 Write %d\n",i);
    delay(1);
  }

}

void sendPacket()
{

}
