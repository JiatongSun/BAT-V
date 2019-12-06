#define CircuitInput 0;
volatile bool lastState;
volatile bool currentState;
volatile int currentTime;
volatile int oldTime;
volatile int pulseWidth
portMUX_TYPE mux = portMUX_INITIALIIZER_UNLOCKED;

void IRAM_ATTR handleSignalReceived(){
  portENTER_CRITICAL_ISR(&mux);
  lastState = currentState;
  currentState = digitalRead(CircuitInput);
  lastTime = currentTime;
  currentTime = xTaskGetTickCount();
  portEXIT_CRITICAL_ISR(&mux);

}
void setup() {
  // put your setup code here, to run once:
attachInterrupt(digitalPinToInterrupt(CircuitInput),handleSignalReceived,CHANGE);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (currentState == 0 && lastState ==1)
  {
    pulseWidth = currentTime - lastTime;
  }
}
