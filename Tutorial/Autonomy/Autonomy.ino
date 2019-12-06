void setup(){
    Serial.begin(115200);    
    pinMode(VIVE_PIN,INPUT); 
}

void loop(){
    if(digitalRead(VIVE_PIN)==HIGH) vive_state = true;
    else vive_state = false;

    if(vive_state && !last_vive_state) signal_start = micros();
    else if(!vive_state && last_vive_state){
        long bandwidth = micros() - signal_start;
        
        if(bandwidth > 60){
            sync_cnt ++;
            sync_end = micros();
        } else if (bandwidth < 60 && bandwidth > 0){
            if(sync_cnt == 3){
                is_x_found = true;
                x_coor = micros() - sync_end;
            } else if(sync_cnt == 1){
                is_y_found = true;
                y_coor = micros() - sync_end;
            }
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
