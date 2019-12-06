#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "HZYYYYY";
WiFiUDP udp;
const int UDP_PACKET_SIZE = 100;
char udpBuffer[UDP_PACKET_SIZE];
IPAddress ipTarget(192,169,1,124);
IPAddress myIP(192,168,1,142);


void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
pinMode(2,OUTPUT);
Serial.print("Connecting to ");
Serial.println(ssid);
WiFi.config(myIP,IPAddress(192,168,1,1),IPAddress(255,255,255,0));
WiFi.begin(ssid);
udp.begin(208);

while (WiFi.status() != WL_CONNECTED)
{
  delay(500);
  Serial.print(".");
}
Serial.println("WiFi connected");
Serial.print(WiFi.localIP());
}

void fncUdpSend()
{
  strcpy(udpBuffer,"hello testing message");
  udp.beginPacket(ipTarget, 208);
  udp.printf("%s",udpBuffer);
  udp.endPacket();
  Serial.println(udpBuffer);
}

void fncUdpSendint(int i)
{
  udpBuffer[0] = i & 0xff;
  udpBuffer[1] = i >> 8 ;
  udpBuffer[2] = 0;
  udp.beginPacket(ipTarget, 208);
      udp.printf("%s",udpBuffer);
  udp.endPacket();
}

void loop() {
  // put your main code here, to run repeatedly:
delay(4000);
digitalWrite(2,LOW);
delay(1000);
digitalWrite(2,HIGH);
fncUdpSend();

}
