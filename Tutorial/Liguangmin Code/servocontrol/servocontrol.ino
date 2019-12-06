const int servobutton = 0;
const int servomotor= 18;
const int LEDC_CHANNEL = 1;
const int LEDC_RESOLUTION_BITS=13;//bits of resolution is 13
const int LEDC_RESOLUTION = ((1<<LEDC_RESOLUTION_BITS)-1);//resolution is 2^13
const int LEDC_FREQ_HZ = 50;//interrupt frequency for servo set to be 50
int servoread = 0;
int val = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  ledcSetup(LEDC_CHANNEL,LEDC_FREQ_HZ,LEDC_RESOLUTION_BITS);//Set up LEDC channel 0 at 5000HZ, 2^13 resolution
  ledcAttachPin(servomotor,LEDC_CHANNEL);//Attach ledc at LEDC_CHANNEL_SERVO to servo control pin
  pinMode(servobutton,INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint32_t servoduty = map(analogRead(servopot),0,4095,450,1050)*LEDC_RESOLUTION/10000;
  ledcWrite(LEDC_CHANNEL,servoduty);//PWM to servo motor based on the received duty cycle
  Serial.println(servoduty);
}
