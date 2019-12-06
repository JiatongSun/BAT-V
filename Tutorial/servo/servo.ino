#define LEDC_CHANNEL 0 // use first channel of 16  
#define LEDC_RESOLUTION_BITS 13 
#define LEDC_RESOLUTION ((1<<LEDC_RESOLUTION_BITS)-1) 
#define LEDC_FREQ_HZ          50
#define SERVO_PIN             16
#define MAX_ANGLE             1000
#define MIN_ANGLE             200
#define CLOCKWISE             1
#define COUNTERCLOCKWISE      -1

int pos = MIN_ANGLE;
int dir = CLOCKWISE;

void setup(){
    Serial.begin(115200); 
    ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ,  LEDC_RESOLUTION_BITS);
    ledcAttachPin(SERVO_PIN, LEDC_CHANNEL );
}

void loop(){
    ledcWrite(LEDC_CHANNEL,pos);
    pos += dir;
    if(pos == MAX_ANGLE || pos == MIN_ANGLE)dir = -dir;
    Serial.println(pos);
    delay(3); 
}
