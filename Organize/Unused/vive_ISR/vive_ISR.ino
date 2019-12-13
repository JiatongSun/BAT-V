#define VIVE_PIN 21

volatile bool rise_flag = false;                // flag indicating rising edge
volatile bool fall_flag = false;                // flag indicating falling edge
volatile long signal_start = 0;                 // time of rising edge      
long sync_end = 0;                              // time of sync signal falling edge
long bandwidth = 0;                             // signal bandwidth  
int sync_cnt = 0;                               // cumulative sync signal count
bool is_x_found = false, is_y_found = false;    // flag indicating if x or y is found
int x_coor = 0, y_coor = 0;                     // coordinate

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR riseReceived() {                 // capture rising edge
  portENTER_CRITICAL_ISR(&mux);
  rise_flag = true;                             // set a flag to true 
  signal_start = micros();                      // document time of rising edge
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR fallReceived() {
  portENTER_CRITICAL_ISR(&mux);
  fall_flag = true;                             // set a flag to true
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(115200);
  pinMode(VIVE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(VIVE_PIN), riseReceived, RISING);           // attach interrupt to rising edge first
}

void loop() {
    if (rise_flag) {                                                                // a rising edge is found
        rise_flag = false;
        detachInterrupt(digitalPinToInterrupt(VIVE_PIN));                           // detach interrupt
        attachInterrupt(digitalPinToInterrupt(VIVE_PIN), fallReceived, FALLING);    // attach interrupt to falling edge
    } else if (fall_flag){                                                          // a falling edge is found
        fall_flag = false;
        detachInterrupt(digitalPinToInterrupt(VIVE_PIN));                           // detach interrupt
        attachInterrupt(digitalPinToInterrupt(VIVE_PIN), riseReceived, RISING);     // attach interrupt to rising edge
        
        bandwidth = micros() - signal_start;                                        // calculate bandwidth

        if (bandwidth > 60 && bandwidth < 2000) {                                   // found a sync signal
            sync_cnt++;
            sync_end = micros();
        }
        else if (bandwidth < 60 && bandwidth > 10) {                                // found an x or y signal
            if (sync_cnt == 3) {
                is_x_found = true;
                x_coor = micros() - sync_end;                                       // calculate x coordinate
            } else if (sync_cnt == 1) {
                is_y_found = true;
                y_coor = micros() - sync_end;                                       // calculate y coordinate
            }
            sync_cnt = 0;                                                           // reset sync signal count
        }
    }
  
    if (is_x_found && is_y_found) {
        Serial.print("x = ");
        Serial.print(x_coor);
        Serial.print("    y = ");
        Serial.println(y_coor);
        is_x_found = false;                                                         // reset flag 
        is_y_found = false;                                                         // reset flag
    }
}
