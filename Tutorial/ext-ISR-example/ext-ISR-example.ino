/*
 * External Interrupt 
 * MEAM510 2019 example
 * Code debounces a switch at pin BUTTONPIN
 */
#define BUTTONPIN    0
#define DEBOUNCETIME 20 // in ms 

#define CircuitInput 0

volatile int numberOfButtonInterrupts = 0;
volatile bool lastState;
volatile uint32_t debounceTimeout = 0;

volatile bool currentState;
volatile int currentTime;
volatile int lastTime;
volatile int pulseWidth;
int sync_cnt = 0;
bool is_x_found = false, is_y_found = false;
long x_coor = 0, y_coor = 0;
int dis;


portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;



// Interrupt Service Routine - Keep it short! 
void IRAM_ATTR handleSignalReceived(){
  portENTER_CRITICAL_ISR(&mux);
  lastState = currentState;
  currentState = digitalRead(CircuitInput);
  lastTime = currentTime;
  currentTime = xTaskGetTickCount();
  portEXIT_CRITICAL_ISR(&mux);

}      
//void IRAM_ATTR handleButtonInterrupt() {
//  portENTER_CRITICAL_ISR(&mux);
//   numberOfButtonInterrupts++;
//   lastState = digitalRead(BUTTONPIN);
//   debounceTimeout = xTaskGetTickCount();   //faster version of millis() 
//  portEXIT_CRITICAL_ISR(&mux);
//}

void setup(){
  Serial.begin(115200);
  pinMode(BUTTONPIN, INPUT);  // Pull up to 3.3V on input - GPIO 0 has external pullup    
  //attachInterrupt(digitalPinToInterrupt(BUTTONPIN), handleSignalReceived, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CircuitInput),handleSignalReceived,CHANGE);
}

void loop() {

//  bool currentState = digitalRead(BUTTONPIN);
//
//  if ((numberOfButtonInterrupts != 0) //interrupt has triggered                                                                   
//    && (currentState == lastState) // pin state same as last interrupt                                  
//    && (millis() - debounceTimeout > DEBOUNCETIME )) 
//  { //  low for at least DEBOUNCETIME,            
//    if (currentState == LOW)  Serial.printf("Button is pressed\n");
//    else   Serial.printf("Button is RELEASED\n");
//    Serial.printf("Triggered %d times\n", numberOfButtonInterrupts);
//    portENTER_CRITICAL_ISR(&mux); // can't change it unless, atomic - Critical section                       
//     numberOfButtonInterrupts = 0; // acknowledge keypress and reset interrupt counter                       
//    portEXIT_CRITICAL_ISR(&mux);
//    delay(1);
//  }
//  delay(1);

if (currentState == 0 && lastState ==1){
     pulseWidth = currentTime - lastTime;
  }
  else if (currentState == 1 && lastState == 0) dis = currentTime - lastTime;
   if(pulseWidth > 100){
            sync_cnt ++;
        } else if (pulseWidth < 100 && pulseWidth > 0){
            if(sync_cnt == 3){
                is_x_found = true;
                x_coor = dis;
            } else if(sync_cnt == 1){
                is_y_found = true;
                y_coor = dis;
            }
            sync_cnt = 0;
        }

        if(is_x_found && is_y_found){
        Serial.print("x = ");
        Serial.print(x_coor);
        Serial.print("    y = ");
        Serial.println(y_coor);
        is_x_found = false;
        is_y_found = false;
    }
 }
