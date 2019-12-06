#define VIVE_PIN_1    21
#define VIVE_PIN_2    39

bool vive_state_1 = false, last_vive_state_1 = false;
int sync_cnt_1 = 0;
long signal_start_1 = 0;
long sync_end_1 = 0;
long x_coor_1 = 0, y_coor_1 = 0;
bool is_x_found_1 = false, is_y_found_1 = false;

bool vive_state_2 = false, last_vive_state_2 = false;
int sync_cnt_2 = 0;
long signal_start_2 = 0;
long sync_end_2 = 0;
long x_coor_2 = 0, y_coor_2 = 0;
bool is_x_found_2 = false, is_y_found_2 = false;

void setup(){
    Serial.begin(115200);    
    pinMode(VIVE_PIN_1,INPUT); 
    pinMode(VIVE_PIN_2,INPUT); 
}

void loop(){
    if(digitalRead(VIVE_PIN_1)==HIGH) vive_state_1 = true;
    else vive_state_1 = false;

    if(vive_state_1 && !last_vive_state_1) signal_start_1 = micros();
    else if(!vive_state_1 && last_vive_state_1){
        long bandwidth_1 = micros() - signal_start_1;
        
        if(bandwidth_1 > 60){
            sync_cnt_1 ++;
            sync_end_1 = micros();
        } else if (bandwidth_1 < 60 && bandwidth_1 > 10){
            if(sync_cnt_1 == 3){
                is_x_found_1 = true;
                x_coor_1 = micros() - sync_end_1;
            } else if(sync_cnt_1 == 1){
                is_y_found_1 = true;
                y_coor_1 = micros() - sync_end_1;
            }
            sync_cnt_1 = 0;
        }
    }
    last_vive_state_1 = vive_state_1;
    if(is_x_found_1 && is_y_found_1){
        Serial.print("x_1 = ");
        Serial.print(x_coor_1);
        Serial.print("    y_1 = ");
        Serial.println(y_coor_1);
        is_x_found_1 = false;
        is_y_found_1 = false;
    }

    if(digitalRead(VIVE_PIN_2)==HIGH) vive_state_2 = true;
    else vive_state_2 = false;

    if(vive_state_2 && !last_vive_state_2) signal_start_2 = micros();
    else if(!vive_state_2 && last_vive_state_2){
        long bandwidth_2 = micros() - signal_start_2;
        
        if(bandwidth_2 > 60){
            sync_cnt_2 ++;
            sync_end_2 = micros();
        } else if (bandwidth_2 < 60 && bandwidth_2 > 10){
            if(sync_cnt_2 == 3){
                is_x_found_2 = true;
                x_coor_2 = micros() - sync_end_2;
            } else if(sync_cnt_2 == 1){
                is_y_found_2 = true;
                y_coor_2 = micros() - sync_end_2;
            }
            sync_cnt_2 = 0;
        }
    }
    last_vive_state_2 = vive_state_2;
    if(is_x_found_2 && is_y_found_2){
        Serial.print("x_2 = ");
        Serial.print(x_coor_2);
        Serial.print("    y_2 = ");
        Serial.println(y_coor_2);
        is_x_found_2 = false;
        is_y_found_2 = false;
    }
}
