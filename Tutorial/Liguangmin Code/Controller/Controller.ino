/*This code is running on the sending side
Note that the controller is in AP which can be accessed by the running car*/

#include <WiFi.h> //libraries
#include <WiFiUdp.h>

#define trigger 4
const int POT = 32; // Setting up the pin to control remote DC motors
const int SERVOPOT = 33;//Setting up the pot to control remote servo motor
void sendPacket();//Function prototye to send integer package
int buttonRead;

int potread; // initializing the reading from the pot that has been mapped
int cb = 0; // the size of the packet received. 0 if no packet received
int port = 1609; // my local port
int targetPort = 1609; // The port on the cart
int MessageSent = 0;//Declare the variable to store the message sent out for DC motors
int SERVOMessageSent = 0;//Declare the variable to store message sent out for servo motor
int servopotread = 0;//Declare the reading from servo motor
//both for send and receive
const char* ssid = "SmartGuangming"; // my wifi name since I am in AP mode
//const char* password = "GDI";

//WiFiServer server(port);

//for receiver
WiFiUDP UDPTestServer; // setting up UDP communication for receiver
IPAddress myIPaddress (192, 168, 1, 125); //my IP address

const int UDP_PACKET_SIZE = 100; // can be up to 65535
byte packetBuffer [UDP_PACKET_SIZE + 1]; // this will hold the other player's packet

//for sender
WiFiUDP udp; // settinng up UDP communication for sending to the other player
//const int UDP_PACKET_SIZE= 100;
char udpBuffer[UDP_PACKET_SIZE]; // this will hold the packet that I am sending
IPAddress ipTarget(192, 168, 1, 142); //IP address of the car

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); // baud rate
  // Serial.println();
  // Serial.print("Connecting to ");
  // Serial.println(ssid);
  pinMode(POT, INPUT); // setting up the pot input for speed of DC motors
  pinMode(SERVOPOT,INPUT);// setting up the pot input for speed of servo motor
  pinMode(trigger, INPUT); //setting up the button for the attack/defend mechanism
  WiFi.setSleep(false);//Set WiFi to never fall into sleep mode ;
//
//  //station mode
 // WiFi.mode(WIFI_STA);
 // WiFi.config(myIPaddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));
 // WiFi.begin(ssid);

    //AP mode
    WiFi.mode(WIFI_AP);
    WiFi.softAP(ssid);//initialize the name of WiFi in AP mode
    delay(100);
    WiFi.softAPConfig(myIPaddress, IPAddress(192, 168, 1, 1), IPAddress(255, 255, 255, 0));//Configure WiFi addresses

  udp.begin(port); // starting the udp communications
  UDPTestServer.begin(port);//Starting port communication
//
 // while (WiFi.status() != WL_CONNECTED) {
 //   delay(500);
 //   Serial.print(".");
 // }
 // Serial.println("WiFi connected");
  packetBuffer[UDP_PACKET_SIZE] = 0; // null terminate

}

void loop() {//a loop runs forever
  potread = map(analogRead(POT),0,4095,1000,3000);//read POT value and map it between 3000 and 1000
  servopotread = map(analogRead(SERVOPOT),0,4095,4951,4000);//read servopot value and map it between 4095-5000
  Serial.println(servopotread);
  buttonRead = digitalRead(trigger)*1000+1000;
  Serial.println("ButtonRead");
  Serial.println(buttonRead);
  sendPacket();//Run the subroutine to send out packet
  }




//
//void receivePacket() { // FOR RECEIVING
//  cb = UDPTestServer.parsePacket(); // parse packet. Sees if I get something
//  if(cb){ // If I did
//    UDPTestServer.read(packetBuffer, UDP_PACKET_SIZE); // read the packet, though it is in bytes
////    Serial.printf("%s\n", packetBuffer);
////    int first = (int)(packetBuffer[0]);
////    int second = (int)(packetBuffer[1]<<8);
//    val = (packetBuffer[1]<<8) | packetBuffer[0]; // convert it to an integer
//    Serial.println(got);
//    cb = 0;
//  }
//}

void sendPacket() {
//  char buf[8];
//  itoa(tenth, buf, 10);
//  strcpy(udpBuffer, buf);// send what ever you want upto buffer size

  udpBuffer[0] = potread & 0xff; // send LSB for first integer
  udpBuffer[1] = potread >> 8; // send MSB for first integer
  udpBuffer[2] = servopotread  & 0xff;//send LSB for first integer
  udpBuffer[3] = servopotread >> 8;//send MSB for second integer
  udpBuffer[4] = buttonRead & 0xff;
  udpBuffer[5] = buttonRead >> 8;
//   udpBuffer[4] = buttonRead & 0xff; // send LSB for first integer
//   udpBuffer[5] = buttonRead >> 8; // send MSB for first integer
  udpBuffer[6] = 0; // null terminate

  udp.beginPacket(ipTarget, targetPort);  // send to car port
  udp.printf("%s", udpBuffer);//Send the message
  udp.endPacket(); // end msg


}
