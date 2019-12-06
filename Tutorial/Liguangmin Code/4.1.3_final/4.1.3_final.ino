#include <WiFi.h> //include the wifi file
#include <WiFiUdp.h>//include the UDP protocal file

const char* ssid = "NANI";//connect to a access point named NANI
//const char* password = "19970524";
const int POTPin = 32;//set P32 as the pin of the potentiometer
const int LED = 0;//set P0 as the OUTPUT LED pin
const int Switch = 33;//set P33 as the pin for the switch
int val = 0;//initialize the value to store the analog read data
int j = 0 ;//initialize a counter
int cb = 0;//initialize a flag which indicates if you have received a packet
int largervalue = 0;//initialize a value to store the smaller value
int NumberReceived = 0;//initialize a value to store the number received
//void handleUDPServer();
//void fncUdpSend();
void fncUdpSendint(int i);//function prototype for sending a packet of integer
void handleUDPServerint();//function prototype to parse the integer received
WiFiUDP UDPTestServer;//initialize the UDP server
const int UDP_PACKET_SIZE = 100;//set the packet size to be 100
char udpBuffer[UDP_PACKET_SIZE];//initialize the packet sent to be size of Packetsize
byte packetBuffer[UDP_PACKET_SIZE+1];//initialize the packet received to be size of packet+1
IPAddress ipTarget(192,168,1,125);//IP address sending message to
IPAddress myIP(192,168,1,142);//My own IP address
//WiFiServer server(80);


void setup() {
//setup function which only runs once
pinMode(Switch,INPUT); //set the Switch Pin as an Input pin
pinMode(2,OUTPUT);//Set the onboard LED pin to be output pin
pinMode(LED,OUTPUT);//Set the external LED pin as the outpin pin
Serial.begin(115200);//begin serial communication at 115200 baud
Serial.print("Connecting to ");//Print a message that is connecting to target network
Serial.println(ssid);//Print a message that is connecting to target network
WiFi.mode(WIFI_STA);//set the WiFi mode to be in station mode
WiFi.config(myIP,IPAddress(192,168,1,1),IPAddress(255,255,255,0));//configure my ip,the gateway ip and mask
WiFi.begin(ssid);//begin WiFi on the designated network
//server.begin();
UDPTestServer.begin(1609);//initialize the UDP receving on port 1609

while (WiFi.status() != WL_CONNECTED)//runs forever until WiFi is connected
{
  delay(500);//wait for 0.5 seconds
  Serial.print(".");//print a .
}
Serial.println("WiFi connected");//Print WiFi is connected
packetBuffer[UDP_PACKET_SIZE] = 0 ;//null terminator on 100th of Packet
//Serial.print(WiFi.localIP());
}

//


void loop()//a loop runs forever
{

while (j<= 49)//while the counter is smaller than 49
{
val = map(analogRead(POTPin),0,4095,100,10000);//read the value from the  potentiometer
digitalWrite(LED,HIGH);//turn on the External_LED
delay(val/100);//delay for 1/100 of the time read from the analog pin
if (!digitalRead(Switch) ==1)//if the switch is pressed
{
  fncUdpSendint(val/100);//send 1/100 of the value read to the opponent
  while (cb ==0)//when no packet has been received
  {
  handleUDPServerint();//try to receive a packet from the opponent
  }
  cb = 0;//reset the packet flag
  largervalue = max(val/100,NumberReceived);//store the larger value of these two
  if ((abs(val/100-NumberReceived)*100/largervalue) < 10)//when you press the button and the difference is smaller than 10%
  {
    while(1)
    {
      digitalWrite(2,HIGH);//turn on the onboard LED marks you are winning
    }
  }
  else if ((abs(val/100-NumberReceived)*100/largervalue) >= 10)//when you press the button and the digerence is bigger than 10%
  {
    while(1)
    {
      digitalWrite(2,LOW);//turn off the onboard LED to show you lost
    }

  }
}
handleUDPServerint();//try to receive a packet from opponent each loop
if (cb)//if a package has been received
{
  fncUdpSendint(val/100);//send val/100 to the opponent
  largervalue = max(val/100,NumberReceived);//return the larger value of two
  if ((abs(val/100-NumberReceived)*100/largervalue) < 10)//if the button is not pressed and the difference is smaller than 10%
  {
    while(1)
    {
      digitalWrite(2,LOW);//Turn the onboard LED off to show you have lost
    }
  }
  else if ((abs(val/100-NumberReceived)*100/largervalue) >= 10)//if the button is not pressed and the difference is greater than 10%
  {
    while(1)
    {
      digitalWrite(2,HIGH);//Turn on the onboard LED to show you have won
    }
  }
}
j=j+1;//add the counter
}

j=0;//set counter back to zero after loop


/*The following loop follows the exact same routine except it happens in the LED off state */
while ( j <=49)
{
val = map(analogRead(POTPin),0,4095,100,10000);
Serial.println("val");
Serial.println(val);

digitalWrite(LED,LOW);
delay(val/100);
if (!digitalRead(Switch) ==1)
{
  fncUdpSendint(val/100);
  Serial.println(val/100);
  while(cb ==0){
//  NumberReceived = handleUDPServerint();
  handleUDPServerint();
  }
  cb  = 0;
  Serial.println(NumberReceived);
  largervalue = max(val/100,NumberReceived);
   Serial.println("largervalue");
  Serial.println(largervalue);
  if ((abs(val/100-NumberReceived)*100/largervalue) < 10)
  {
    while(1)
    {
      digitalWrite(2,HIGH);
    }
  }
  else if (abs((val/100-NumberReceived)*100/largervalue) >= 10)
  {
    while(1)
    {
      digitalWrite(2,LOW);
    }

  }
}
handleUDPServerint();
if (cb)
{
   Serial.println(NumberReceived);
  fncUdpSendint(val/100);
  Serial.println(val/100);
  largervalue = max(val/100,NumberReceived);
    Serial.println("largervalue");
  Serial.println(largervalue);
  if ((abs(val/100-NumberReceived)*100/largervalue) < 10)
  {
    while(1)
    {
      digitalWrite(2,LOW);
    }
  }
  else if ((abs(val/100-NumberReceived)*100/largervalue) >= 10)
  {
    while(1)
    {
      digitalWrite(2,HIGH);
    }
  }
}
j=j+1;
}
j=0;
Serial.println();
}


/* The following subroutine packs the int packet and send it */
void fncUdpSendint(int i)//void function which takes in an integer as input
{
  udpBuffer[0] = i & 0xff; //Put the first byte of i in the first byte of udpBuffer
  udpBuffer[1] = i >> 8 ;//Put the second byte of i in the second byte of udpBuffer
  udpBuffer[2] = 0;//write a 0 to the third byte of udpbuffer to terminate udpBuffer
  UDPTestServer.beginPacket(ipTarget, 1609);//begin sending packet to the oppoent at their opening port
  UDPTestServer.printf("%s",udpBuffer);//send udpBuffer to the opponent
  UDPTestServer.endPacket();//end sending the packet

}
/*The following subrountine receives opponent's packet and parse it*/
void handleUDPServerint()//void function which doesn't take in anything
{
  cb = UDPTestServer.parsePacket();//set cb to the length of packet received
  int i;
  if (cb)//if cb is not 0
  UDPTestServer.read(packetBuffer,UDP_PACKET_SIZE);//read the packetreceived
  NumberReceived =  packetBuffer[0] | (packetBuffer[1] << 8);//number received is the first byte received | second packetreceived 
//    i =   packetBuffer[0] | (packetBuffer[1] << 8) ;
//    return i;

  }
}
