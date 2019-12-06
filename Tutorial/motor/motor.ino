#define LEDC_CHANNEL 0 // use first channel of 16  
#define LEDC_RESOLUTION_BITS 8 
#define LEDC_RESOLUTION ((1<<LEDC_RESOLUTION_BITS)-1) 
#define LEDC_FREQ_HZ  1000 
#define LEDC_PIN      32
#define DIR_PIN_1     23
#define DIR_PIN_2     22

// receive four data
int motor_speed = 0;
bool dir = 0;

void setup(){
    Serial.begin(115200); 
    
    pinMode(DIR_PIN_1,OUTPUT); // motor
    pinMode(DIR_PIN_2,OUTPUT); // motor
    ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ,  LEDC_RESOLUTION_BITS);
    ledcAttachPin(LEDC_PIN, LEDC_CHANNEL );
}

void loop(){
    motor_speed = 255;
    
    if(dir){ 
        digitalWrite(DIR_PIN_1,HIGH);
        digitalWrite(DIR_PIN_2,LOW);
    } else {
        digitalWrite(DIR_PIN_1,LOW);
        digitalWrite(DIR_PIN_2,HIGH);
    }
    ledcWrite(LEDC_CHANNEL,motor_speed);    
}
