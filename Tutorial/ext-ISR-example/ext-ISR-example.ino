/*
 * External Interrupt 
 * MEAM510 2019 example
 * Code debounces a switch at pin BUTTONPIN
 */
#define BUTTONPIN    0
#define DEBOUNCETIME 20 // in ms 
volatile int numberOfButtonInterrupts = 0;
volatile bool lastState;
volatile uint32_t debounceTimeout = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Interrupt Service Routine - Keep it short!       
void IRAM_ATTR handleButtonInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
   numberOfButtonInterrupts++;
   lastState = digitalRead(BUTTONPIN);
   debounceTimeout = xTaskGetTickCount();   //faster version of millis() 
  portEXIT_CRITICAL_ISR(&mux);
}

void setup(){
  Serial.begin(115200);
  pinMode(BUTTONPIN, INPUT);  // Pull up to 3.3V on input - GPIO 0 has external pullup    
  attachInterrupt(digitalPinToInterrupt(BUTTONPIN), handleButtonInterrupt, CHANGE);
}

void loop() {

  bool currentState = digitalRead(BUTTONPIN);

  if ((numberOfButtonInterrupts != 0) //interrupt has triggered                                                                   
    && (currentState == lastState) // pin state same as last interrupt                                  
    && (millis() - debounceTimeout > DEBOUNCETIME )) 
  { //  low for at least DEBOUNCETIME,            
    if (currentState == LOW)  Serial.printf("Button is pressed\n");
    else   Serial.printf("Button is RELEASED\n");
    Serial.printf("Triggered %d times\n", numberOfButtonInterrupts);
    portENTER_CRITICAL_ISR(&mux); // can't change it unless, atomic - Critical section                       
     numberOfButtonInterrupts = 0; // acknowledge keypress and reset interrupt counter                       
    portEXIT_CRITICAL_ISR(&mux);
    delay(1);
  }
  delay(1);
}

