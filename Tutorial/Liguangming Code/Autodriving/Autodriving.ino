/*This is the code running on the Receiver Side
The car is running on station mode, which connects to the access point on the controller
Author: Yupeng Li */
//GPIO 4 USED for the WEAPON SWITCH
const int LEDC_CHANNEL = 0;//interrupt channel 0
const int LEDC_CHANNEL1 = 1;//interrupt channel 1
const int LEDC_CHANNEL_SERVO = 2;//interrupt channel 2
const int LEDC_RESOLUTION_BITS=13;//bits of resolution is 13
const int LEDC_RESOLUTION = ((1<<LEDC_RESOLUTION_BITS)-1);//resolution is 2^13
const int LEDC_FREQ_HZ = 5000;//interrupt frequency set to be 5000
const int LEDC_FREQ_HZ_SERVO = 50;//interrupt frequency for servo set to be 50
const int A1= 21;//1A used to control the direction on H bridge
const int N_A1 =22;//4A used to control the NOT direction on H bridge
const int Enable=4;//Enable pin used to control the PWM on H bridge 1
const int ServoControl = 33;//Servo control pin used to control the PWM for the servo
const int PhotoDiode = 27;
uint32_t ServoAngleDuty;
//--------------------------------------------------------------------------------------
//The following are the constants used for the VIVE sensing
int HeadX = 0;
int HeadY = 0;
int tailX = 0;
int tailY = 0;
float DirectionX = HeadX-TailX;
float DirectionY = HeadY-TailY;
float NormalX = DirectionX / sqrt(DirectionX ^ 2 + DirectionY ^ 2);
float NormalY = DirectionY / sqrt(DirectionX ^ 2 + DirectionY ^ 2)
int regionOneLeft;
int regionOneRight;
int regionOneTop;
int regionOneBot;
int regionTwoLeft;
int regionTwoRight;
int regionTwoTop;
int regionTwoBot;
int regionThreeLeft;
int regionThreeRight;
int regionThreeTop;
int regionThreeBot;
int regionFourLeft;
int regionFourRight;
int regionFourTop;
int regionFourBot;

void GoStraight(int Xway, int Yway, float NormalX, float NormalY);
void TurnLeft();
void TurnRight();
//The end for constants used for VIVE sensing
//--------------------------------------------------------------------------------------


//The following is the setup function which only runs once
void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);//Setup Serial Communication at 115200(For debug)
/* The following lines turns the WiFi mode into AP mode */
// WiFi.mode(WIFI_AP);
// WiFi.softAP(ssid);
// delay(100);
// WiFi.softAPConfig(myIPaddress,IPAddress(192,168,1,1),IPAddress(255,255,255,0));
ledcSetup(LEDC_CHANNEL,LEDC_FREQ_HZ,LEDC_RESOLUTION_BITS);//Set up LEDC channel 0 at 5000HZ, 2^13 resolution
ledcSetup(LEDC_CHANNEL1,LEDC_FREQ_HZ,LEDC_RESOLUTION_BITS);//Set up LEDC Channel 1 at 5000Hz, 2^13 resolution
ledcSetup(LEDC_CHANNEL_SERVO,LEDC_FREQ_HZ_SERVO,LEDC_RESOLUTION_BITS);//Setup LEDC Channel 2 at 50Hz, 2^13 resolution
ledcAttachPin(Enable,LEDC_CHANNEL);//Attach ledc at LEDC_CHANNEL to PWM1
ledcAttachPin(ServoControl,LEDC_CHANNEL_SERVO);//Attach ledc at LEDC_CHANNEL_SERVO to servo control pin
pinMode(A1,OUTPUT);//Set 1A as output
pinMode(N_A1,OUTPUT);//Set 4A as output
// pinMode(EnableControl,INPUT);
pinMode(ServoControl,OUTPUT);//Set ServoControl as output
pinMode(PhotoDiode,INPUT);//Set the PhotoDiode pin as an input pin

}

//The end of the setup function
//--------------------------------------------------------------------------------------

void loop() {
  // put your main code here, to run repeatedly:
  uint_32 halfduty = 127;
  uint_32 fullduty = 255;
  uint_32 servoLeft = 450*LEDC_RESOLUTION/10000;
  uint_32 servoRight = 1050*LEDC_RESOLUTION/10000;
  uint_32 servoCenter = 750*LEDC_RESOLUTION/10000;
  while(HeadX<BoundryFourX && tailX<BoundryFourX && HeadY < BoundryFourY && tailY < BoundryFourY )
  {
    while (Direction!=0)//while the head is not in line with horizontal
    {
      if (DirectionX > 0)
      {
        if (DirectionY>0)
        {

          digitalWrite(A1,HIGH);
          digitalWrite(N_A1,LOW);
          ledcWrite(LEDC_CHANNEL,halfduty);
          ledcWrite(LEDC_CHANNEL_SERVO,servoRight);
        }
        else if (DirectionY < 0)
        {
          digitalWrite(A1,HIGH);
          digitalWrite(N_A1,LOW);
          ledcWrite(LEDC_CHANNEL,halfduty);
          ledcWrite(LEDC_CHANNEL_SERVO,servoLeft);
        }

      }
    }
    ledcWrite(LEDC_CHANNEL,fullduty);
    ledcWrite(LEDC_CHANNEL_SERVO,halfduty)

  }

  //If the car is in region 6

//When psudoduty is greater than 0, turn the wheels forward by setting both direction pins to HIGH
  if ((psudoduty) >= 0){
  digitalWrite(A1,HIGH);
  digitalWrite(N_A1,LOW);
  Serial.println("Rotating forward");
  }

  //When the psudoduty is smaller than 0, turn the wheels backward by setting both direction pins to LOW
  else if ((psudoduty)<0)
  {
  Serial.println("Rotating backward");
  digitalWrite(A1,LOW);
  digitalWrite(N_A1,HIGH);
  }

  ledcWrite(LEDC_CHANNEL,duty);//Run the motors at given duty cycle to control there speed
  ledcWrite(LEDC_CHANNEL1,duty);//Run the motors at given duty cycle to control there speed
    uint32_t servoduty = map(servoread,4000,5000,450,1050)*LEDC_RESOLUTION/10000;//map the servo duty to 450-1050 which correspond to full left and full right
    Serial.println("Servo duty");
    Serial.println(map(servoread,4000,5000,450,1050));//Print out the servo duty(For debugging)
    ledcWrite(LEDC_CHANNEL_SERVO,servoduty);//PWM to servo motor based on the received duty cycle
  cb = 0;//Set the receive pack back to 0
}


/*Subroutine used to receive and parse packet*/
void receivePacket(){
  cb = UDPTestServer.parsePacket();//Set cb to the length of the packet received
  if (cb){//When cb is not 0
    UDPTestServer.read(packetBuffer,UDP_PACKET_SIZE);//Read packetbuffer with UDP_PACKET_SIZE from the UDP packet received
    val = (packetBuffer[1]<<8 |packetBuffer[0]);//Save the first two bytes of the packet received as the PWM for DC motors
    servoread = (packetBuffer[3]<<8|packetBuffer[2]);//Save the second two bytes of the packet received as the PWM for servo motors

  }
}

void GoStraight(int Xway, int Yway, float NormalX, float NormalY)
{
  digitalWrite(A1,HIGH);
  digitalWrite(N_A1,LOW);
  ledcWrite(LEDC_CHANNEL,fullduty);
   if (Xway == 1 && Yway == 0 )
   {
    ServoAngleDuty = map((NormalY-Yway)*1000,-1000,1000,450*LEDC_RESOLUTION/10000,1050*LEDC_RESOLUTION/10000);
   }
   else if (Xway == -1 && Yway == 0)
  {
    ServoAngleDuty = map ((NormalY - Yway)*1000,1000,-1000,450*LEDC_RESOLUTION/10000,1050*LEDC_RESOLUTION/10000);
  }
  else if (Xway == 0 && Yway == 1 )
  {
    ServoAngleDuty = map((NormalX - Xway) * 1000,-1000,1000, 450*LEDC_RESOLUTION/10000,1050*LEDC_RESOLUTION/10000);
  }
  else if (Xway == 0 && Yway == -1)
  {
    ServoAngleDuty = map((NormalX - Xway) * 1000, 1000,-1000, 450*LEDC_RESOLUTION/10000,1050*LEDC_RESOLUTION/10000);
  }
  ledcWrite(LEDC_CHANNEL_SERVO,ServoAngleDuty);
}

void TurnLeft()
{
  digitalWrite(A1,HIGH);
  digitalWrite(N_A1,LOW);
  ledcWrite(LEDC_CHANNEL,halfduty);
  ledcWrite(LEDC_CHANNEL,servoLeft);
}
void TurnRight()
{
  digitalWrite(A1,HI)
}
