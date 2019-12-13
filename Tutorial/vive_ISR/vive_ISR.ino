#define VIVE_PIN 21

volatile bool rise_flag = false;
volatile bool fall_flag = false;
volatile long signal_start = 0;
long sync_end = 0;
long bandwidth = 0;
int sync_cnt = 0;
bool is_x_found = false, is_y_found = false;
int x_coor = 0, y_coor = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR riseReceived() {
  portENTER_CRITICAL_ISR(&mux);
  rise_flag = true;
  signal_start = micros();
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR fallReceived() {
  portENTER_CRITICAL_ISR(&mux);
  fall_flag = true;
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(115200);
  pinMode(VIVE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(VIVE_PIN), riseReceived, RISING);
}

void loop() {
    if (rise_flag) {
        rise_flag = false;
        detachInterrupt(digitalPinToInterrupt(VIVE_PIN));
        attachInterrupt(digitalPinToInterrupt(VIVE_PIN), fallReceived, FALLING);
    } else if (fall_flag){
        fall_flag = false;
        detachInterrupt(digitalPinToInterrupt(VIVE_PIN));
        attachInterrupt(digitalPinToInterrupt(VIVE_PIN), riseReceived, RISING);
        
        bandwidth = micros() - signal_start;
        Serial.println(bandwidth);

        if (bandwidth > 60 && bandwidth < 2000) {
            sync_cnt++;
            sync_end = micros();
        }
        else if (bandwidth < 60 && bandwidth > 10) {
            if (sync_cnt == 3) {
                is_x_found = true;
                x_coor = micros() - sync_end;
            } else if (sync_cnt == 1) {
                is_y_found = true;
                y_coor = micros() - sync_end;
            }
            sync_cnt = 0;
        }
    }
  
    if (is_x_found && is_y_found) {
        Serial.print("x = ");
        Serial.print(x_coor);
        Serial.print("    y = ");
        Serial.println(y_coor);
        is_x_found = false;
        is_y_found = false;
    }
}
