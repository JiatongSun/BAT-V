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

hw_timer_t* timer = NULL;
volatile uint32_t isrCounter = 0;

void IRAM_ATTR onTimer(){
    ledcWrite(LEDC_CHANNEL,pos);
    pos += dir;
    if(pos == MAX_ANGLE || pos == MIN_ANGLE)dir = -dir;    
}

void timerSetup(){
    timer = timerBegin(1, 80, true);
    timerAttachInterrupt(timer, &onTimer, true); // Attach onTimer() to our timer.
    // Set alarm to call onTimer after 1 second (value in microseconds).
    timerAlarmWrite(timer, 3000, true);      // Repeat the alarm (third parameter)
    timerAlarmEnable(timer);                    // Start an alarm
}

void setup(){
    Serial.begin(115200); 
    ledcSetup(LEDC_CHANNEL, LEDC_FREQ_HZ,  LEDC_RESOLUTION_BITS);
    ledcAttachPin(SERVO_PIN, LEDC_CHANNEL );
    timerSetup();
}

void loop(){
    
}
