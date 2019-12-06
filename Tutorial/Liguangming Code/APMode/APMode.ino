#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "Liguangming";


WiFiUDP UDPTestServer;
IPAddress myIPaddress(192,168,1,142);
IPAddress NMask(255,255,255,0);
const int UDP_PACKET_SIZE = 100;
byte packetBuffer[UDP_PACKET_SIZE+1];
WiFiServer server(80);
void setup() {
Serial.begin(115200);
WiFi.softAP(ssid);
delay(100);
WiFi.softAPConfig(myIPaddress,IPAddress(192,168,1,1),IPAddress(255,255,255,0));
IPAddress IP= WiFi.softAPIP();
server.begin();
UDPTestServer.begin(233);



//while (WiFi.status() != WL_CONNECTED)
// {
//   delay(500);
//   Serial.print(".");
//
// }

Serial.println("WiFi connected");
 packetBuffer[UDP_PACKET_SIZE] = 0;

}

void handleUDPServer()
{
  int cb = UDPTestServer.parsePacket();
  if (cb)
  {
    UDPTestServer.read(packetBuffer, UDP_PACKET_SIZE);
    Serial.printf("%s\n",packetBuffer);
  }
  
}



void loop() {

handleUDPServer();
delay(1);
}
