#define VIVE_PIN   21

bool currentflag = false;
bool currentstate = false;
int signal_start = 0;
//int time_end = 0;
int bandwidth = 0;
int syn = 0;
bool is_x_found, is_y_found;
int x_coor, y_coor;


//#define VIVE_PIN    35

bool vive_state = false, last_vive_state = false;
int sync_cnt = 0;
long signal_start = 0;
long sync_end = 0;
//long x_coor = 0, y_coor = 0;
//bool is_x_found = false, is_y_found = false;


void setup(){
    Serial.begin(115200);    
    pinMode(VIVE_PIN,INPUT); 
}

void loop(){
    if(digitalRead(VIVE_PIN)==HIGH) {
      vive_state = true;
      Serial.println("true");
    }
    else {
      vive_state = false;
      Serial.println("false");
    }

    if(vive_state && !last_vive_state) {
      signal_start = micros();
      Serial.println(signal_start);
    }
    else if(!vive_state && last_vive_state){
        long bandwidth = micros() - signal_start;
        
        if(bandwidth > 100){
            sync_cnt ++;
            sync_end = micros();
        } else if (bandwidth < 100 && bandwidth > 0){
            if(sync_cnt == 3){
                is_x_found = true;
                x_coor = micros() - sync_end;
            } else if(sync_cnt == 1){
                is_y_found = true;
                y_coor = micros() - sync_end;
            }
            Serial.println(sync_cnt);
            sync_cnt = 0;
        }
    }
    last_vive_state = vive_state;
    if(is_x_found && is_y_found){
        Serial.print("x = ");
        Serial.print(x_coor);
        Serial.print("    y = ");
        Serial.println(y_coor);
        is_x_found = false;
        is_y_found = false;
    }
}
