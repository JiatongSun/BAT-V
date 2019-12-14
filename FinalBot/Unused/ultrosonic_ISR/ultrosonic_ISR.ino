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

void IRAM_ATTR triggerStart() {                                                       // timer interrupt to receive distance every 200 ms
    portENTER_CRITICAL_ISR(&mux);
    is_trigger = true;                                                                // set flag to true
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR echoRise() {
    portENTER_CRITICAL_ISR(&mux);
    is_rise = true;                                                                   // set flag to true
    echo_start = micros();                                                            // save time of rising edge
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR echoFall() {
    portENTER_CRITICAL_ISR(&mux);
    is_fall = true;                                                                   // set flag to true
    duration = micros() - echo_start;                                                 // save duration time of the echoed pulse
    distance = (duration/2)*0.0343;                                                   // calculate distance based on sonic speed 343m/s
    portEXIT_CRITICAL_ISR(&mux);  
}

void ISRSetup(){
    timer = timerBegin(0, 80, true);                                                  // 80MHz / 80 = 1MHz
    timerAttachInterrupt(timer, triggerStart, true); 
    timerAlarmWrite(timer, 200000, true);                                             // trigger a pulse every 200000 us
    timerAlarmEnable(timer);     
    
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoRise, RISING);               // attach interrupt to rising edge first
}

void ultraDetect(){
    if(is_trigger){
        digitalWrite(TRIGGER_PIN,LOW);                                                // start to trigger a pulse
        delayMicroseconds(2); 
        digitalWrite(TRIGGER_PIN,HIGH);                                               // creating a pulse (high voltage level)
        delayMicroseconds(10);                                                        // trigger a 10 us pulse
        digitalWrite(TRIGGER_PIN,LOW);                                                // finish triggering
        if(is_rise){
            is_rise = false;                                                          // reset interrupt flag
            detachInterrupt(digitalPinToInterrupt(ECHO_PIN));                         // detach interrupt  
            attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoFall, FALLING);      // attach interrupt to falling edge
        } else if(is_fall){
            is_fall = false;                                                          // reset interrupt flag
            detachInterrupt(digitalPinToInterrupt(ECHO_PIN));                         // detach interrupt
            attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoRise, RISING);       // attach interrupt to rising edge
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
