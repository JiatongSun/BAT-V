#define    TRIGGER_PIN             18
#define    ECHO_PIN                19

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t* timer = NULL;
long echo_start = 0;
float distance = 0;
bool is_trigger = false;
volatile long duration = 0; 
volatile bool is_rise = false, is_fall = false;

void setup(){
    Serial.begin(115200); 
    pinMode(TRIGGER_PIN,OUTPUT);
    pinMode(ECHO_PIN,INPUT);
    ISRSetup();
}  

void loop(){
    ultraDetect();
    show();
}

void IRAM_ATTR triggerStart() {
    portENTER_CRITICAL_ISR(&mux);
    is_trigger = true;
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR echoRise() {
    portENTER_CRITICAL_ISR(&mux);
    is_rise = true;
    echo_start = micros();
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR echoFall() {
    portENTER_CRITICAL_ISR(&mux);
    is_fall = true;
    duration = micros() - echo_start;
    distance = (duration/2)*0.0343;
    portEXIT_CRITICAL_ISR(&mux);
}

void ISRSetup(){
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, triggerStart, true); 
    timerAlarmWrite(timer, 200000, true);
    timerAlarmEnable(timer);     
    
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoRise, RISING);
}

void ultraDetect(){
    if(is_trigger){
        digitalWrite(TRIGGER_PIN,LOW);
        delayMicroseconds(2);
        digitalWrite(TRIGGER_PIN,HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN,LOW);
        if(is_rise){
            is_rise = false;
            detachInterrupt(digitalPinToInterrupt(ECHO_PIN));
            attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoFall, FALLING);
        } else if(is_fall){
            is_fall = false;
            detachInterrupt(digitalPinToInterrupt(ECHO_PIN));
            attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoRise, RISING);
        }
    }
}

void show(){
    if(millis() % 500 == 0){
        Serial.print("Distance: ");
        if(distance < 400 &&  distance > 2){
            Serial.print(distance);
            Serial.println(" cm");
        } 
        else {
            Serial.println("Out of range!");
        }
    }
}
