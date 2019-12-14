#define    LEFT_ENCODER_PIN        36
#define    RIGHT_ENCODER_PIN       39

#define    PERIOD_THRESH           10000                              // thresh to determine if the period is unreasonable
#define    ERR_NUM_THRESH          180                                // maximum unreasonable number out of 360 to judge if the wheel has stopped
#define    PAUSE_THRESH            20000                              // thresh to determine if the wheel has stopped

bool left_encoder_state = false, last_left_encoder_state = false;     // left encoder voltage level
bool right_encoder_state = false, last_right_encoder_state = false;   // right encoder voltage level
long left_start = 0, right_start = 0;                                 // time of rising edge
long left_period = 0, right_period = 0;                               // duration time of the pulse
int  left_data_cnt = 0, right_data_cnt = 0;                           // data count
long left_data[360] = {0}, right_data[360] = {0};                     // array to save data
double left_rps = 0, right_rps = 0;                                   // round per second
long left_fall = 0, right_fall = 0;                                   // time of falling edge

void setup() {
    Serial.begin(115200);
    pinMode(LEFT_ENCODER_PIN,INPUT);
    pinMode(RIGHT_ENCODER_PIN,INPUT);
}

void loop() {
    encoderCalc();
}



void encoderCalc(){
//***************************** left encoder *****************************
    // read voltage level  
    if(digitalRead(LEFT_ENCODER_PIN)==HIGH) left_encoder_state = true;
    else left_encoder_state = false;

    // capture rising edge or falling edge
    if(left_encoder_state && !last_left_encoder_state) left_start = micros();
    else if(!left_encoder_state && last_left_encoder_state){
        left_fall = micros();
        left_period = 2 * (micros() - left_start);                              // calculate each period
        if(left_period > PERIOD_THRESH)left_period = 0;                         // judge if the period is reasonable
        left_data[left_data_cnt] = left_period;                                 // save data into an array
        float round_period = 0;                                                 
        int err_num_cnt = 0;
        for (int i=0; i<360; ++i) {
            if(!left_data[i]) err_num_cnt++;                                    // calculate unreasonable period
            round_period += left_data[i];                                       // calculate the sum of reasonable period
        }
        if(err_num_cnt < ERR_NUM_THRESH){                                       // if most data are reasonable  
            round_period = round_period * 360 / (360 - err_num_cnt);            // calculate speed  
            left_rps = 1000000.0 / round_period;
        }
        else left_rps = 0;                                            
        left_data_cnt = (left_data_cnt + 1) % 360;            
    }

    if(micros() - left_fall > PAUSE_THRESH) left_rps = 0;                       // stop judgement
    
    last_left_encoder_state = left_encoder_state;    
//**************************** right encoder ***************************** 
    if(digitalRead(RIGHT_ENCODER_PIN)==HIGH) right_encoder_state = true;
    else right_encoder_state = false;
    
    if(right_encoder_state && !last_right_encoder_state) right_start = micros();
    else if(!right_encoder_state && last_right_encoder_state){
        right_fall = micros();
        right_period = 2 * (micros() - right_start);                            // calculate each period
        if(right_period > PERIOD_THRESH)right_period = 0;                       // judge if the period is reasonable
        right_data[right_data_cnt] = right_period;                              // save data into an array
        float round_period = 0;
        int err_num_cnt = 0;
        for (int i=0; i<360; ++i) {
            if(!right_data[i]) err_num_cnt++;                                   // calculate unreasonable period
            round_period += right_data[i];                                      // calculate the sum of reasonable period
        }
        if(err_num_cnt < ERR_NUM_THRESH){                                       // if most data are reasonable  
            round_period = round_period * 360 / (360 - err_num_cnt);            // calculate speed  
            right_rps = 1000000.0 / round_period;       
        }
        else right_rps = 0;
        right_data_cnt = (right_data_cnt + 1) % 360;
    }

    if(micros() - right_fall > PAUSE_THRESH) right_rps = 0;                     // stop judgement

    last_right_encoder_state = right_encoder_state;
}
