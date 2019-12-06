/*
 * Timer Interrupt example
 * MEAM510 2019 example
 * ISR every second 
 * author: MHY
 */

hw_timer_t* timer = NULL;
volatile uint32_t isrCounter = 0;

void IRAM_ATTR onTimer(){
  // Increment the counter and set the ISR time
  isrCounter++;
}

void setup() {
  Serial.begin(115200);
  // Use timer 0 [0:3],  Set prescaler = 80.
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true); // Attach onTimer() to our timer.
  // Set alarm to call onTimer after 1 second (value in microseconds).
  timerAlarmWrite(timer, 1000000, true);      // Repeat the alarm (third parameter)
  timerAlarmEnable(timer);                    // Start an alarm
}

void loop() {
  static uint32_t oldCounter = 0;
  
  if (oldCounter != isrCounter) { // If Timer has fired 
    Serial.printf("%d\n", isrCounter);
    oldCounter = isrCounter;
  }
}
