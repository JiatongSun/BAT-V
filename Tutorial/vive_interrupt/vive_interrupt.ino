#define VIVE_PIN 21

volatile bool currentflag = false;
bool currentstate = false;
int signal_start = 0;
int signal_end = 0;
int bandwidth = 0;
int syn = 0;
bool is_x_found, is_y_found;
int x_coor, y_coor;


portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Interrupt Service Routine - Keep it short! 
void IRAM_ATTR ViveSignalReceived(){
  portENTER_CRITICAL_ISR(&mux);
  currentflag = true;
  currentstate = digitalRead(VIVE_PIN);
  if(currentstate) {
    signal_start = micros();
  }
  else {
    bandwidth = micros() - signal_start;
  }
  portEXIT_CRITICAL_ISR(&mux);

}      

void setup() {
    Serial.begin(115200);
    pinMode(VIVE_PIN,INPUT); 
    attachInterrupt(digitalPinToInterrupt(VIVE_PIN),ViveSignalReceived,CHANGE);
}

void loop() {
      if(currentflag){
        currentflag = false;
//        currentstate = digitalRead(VIVE_PIN);
//        if(currentstate) {
//          signal_start = micros();
//          //Serial.print("start ");
//          //Serial.print(signal_start);
//        }
//        else {
//          
//          //Serial.print("     end  ");
//         // Serial.print(signal_end);
//          bandwidth = micros() - signal_start;
//          //Serial.print("     bandwidth  ");
//          Serial.println(bandwidth);
//        }
      }

      if(bandwidth > 60) {
          syn++;
          signal_end = micros();
      }
      else if (bandwidth < 60 && bandwidth > 0){
            if(syn == 3){
                is_x_found = true;
                x_coor = micros() - signal_end;
            } else if(syn == 1){
                is_y_found = true;
                y_coor = micros() - signal_end;
            }
            syn = 0;
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
