#include <WiFi.h>
#include <WiFiUDP.h>

#define VRX_pin A0  // joystick vertical
#define VRY_pin A7  // joystick horizontal
#define SW_pin_1 23 // switch 1
#define SW_pin_2 22 // switch 2
#define PTM_pin A5  // potentiometer

// WiFi name
const char* ssid = "BAT_V";                // this is the network name
const char* pass = "BATATABANANA";         // this is the password

// IP Addresses
IPAddress IPlocal(192,168,1,171);          // initialize local IP address
IPAddress IPtarget(192,168,1,179);         // initialize target IP address

// variables for UDP
WiFiUDP udp;
unsigned int UDPlocalPort = 2918;   // UDP port number for local ESP
unsigned int UDPtargetPort = 2816;  // UDP port number on the target ESP
const int packetSize = 100;         // define packetSize (length of message)
byte sendBuffer[packetSize];        // create the sendBuffer array
byte receiveBuffer[packetSize];     // create the receiveBuffer array

// send five data
int VRX = 0;      // joystick horizontal data
int VRY = 0;      // joystick vertical data
int SW_1 = 0;     // switch 1 data
int SW_2 = 0;     // switch 2 data
int PTM = 0;      // potentiometer data

// display parameter
int display_time = 1000, display_begin = millis();

void setup() {
    Serial.begin(115200);
    pinMode(SW_pin_1, INPUT);
    pinMode(SW_pin_2, INPUT);
    // setup WiFi and UDP
    AP_UDP_Set();

}

void loop() {
    // avoid sending zero since zero is the null terminate string
    VRX = map(analogRead(VRX_pin),0,4095,1,255);  // joystick x data map to [1,255]
    VRY = map(analogRead(VRY_pin),0,4095,1,255);  // joystick y data map to [1,255]
    SW_1 = digitalRead(SW_pin_1) + 1;             // switch 1 data transfromed to {1,2}
    SW_2 = digitalRead(SW_pin_2) + 1;             // switch 2 data transfromed to {1,2}
    PTM = map(analogRead(PTM_pin),0,4095,1,255);  // potentiometer data map to[1,255]

    UDPsendData();        // send data
    printData();          // print data
    
    delay(100);      
}

void AP_UDP_Set(){          // UDP AP set up
    // Wifi indication
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);
       
    WiFi.mode(WIFI_AP);         // sets the WiFi mode to be AP mode
    WiFi.softAP(ssid, pass); // configures the ESP in AP mode with network name and password
    delay(100);                 // hack need to wait 100ms for AP_START...
    Serial.print("Set softAPConfig "); Serial.println(ssid);    // debug statement
    IPAddress gateway(192,168,1,1);                 // initializes gateway IP
    IPAddress subnet(255,255,255,0);                // initializes subnet mask
    WiFi.softAPConfig(IPlocal, gateway, subnet);    // sets the IP addr of ESP to IPlocal
    udp.begin(UDPlocalPort);    // configure a port for UDP comms
    IPAddress myIP = WiFi.softAPIP();   // demo the usage of the softAPIP command
    Serial.print("AP IP address: "); Serial.println(myIP);      // debug statement

    //Turn OFF to indicate WiFi setup successfully
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
}


void UDPsendData(){         // send data
    // split the integer
    sendBuffer[0] = VRX;              // send VRX
    sendBuffer[1] = VRY;              // send VRY
    sendBuffer[2] = PTM;              // send PTM
    sendBuffer[3] = SW_1;             // send SW_1
    sendBuffer[4] = SW_2;             // send SW_2
    sendBuffer[5] = 0;                // null terminate string

    // send the message
    udp.beginPacket(IPtarget, UDPtargetPort);   // target IP and target port num to send info to
    udp.printf("%s", sendBuffer);               // send the contents of sendBuffer over WiFiUDP
    udp.endPacket();
    // end message
}

void printData(){           // print data
    Serial.print("VRX: ");Serial.print(analogRead(VRX_pin));
    Serial.print("    VRY: ");Serial.print(analogRead(VRY_pin));
    Serial.print("    PTM: ");Serial.print(PTM);
    Serial.print("    SW_1: ");Serial.print(SW_1);
    Serial.print("    SW_2: ");Serial.println(SW_2);
}
