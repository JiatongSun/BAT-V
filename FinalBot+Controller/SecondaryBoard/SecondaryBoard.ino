//========================================================================
//==================== MEAM 510 Final Secondary Board ====================
//========================================================================
//*****************************Team 04: BAT_V*****************************
//**************Members: Jiatong Sun, Xinyue Wei, Haojiong Lu*************
//========================================================================
//==================== MEAM 510 Final Secondary Board ====================
//========================================================================





//========================================================================
//====================== constant definition start =======================
//=============================== ========================================
#define    CLOCKWISE                1  
#define    COUNTERCLOCKWISE         -1  
#define    STANDBY                  0
#define    AUTONOMOUS               0
#define    MANUAL                   1
//********************************* servo *******************************
#define    SERVO_RESOLUTION_BITS    13
#define    SERVO_RESOLUTION         8191
#define    SERVO_FREQ_HZ            50
#define    ORIENT_CHANNEL           0
#define    ORIENT_MIN_ANGLE         300
#define    ORIENT_MAX_ANGLE         450
//********************************* motor ********************************
#define    MOTOR_RESOLUTION_BITS    8
#define    MOTOR_RESOLUTION         255
#define    MOTOR_FREQ_HZ            1000    
#define    BACK_LEFT_CHANNEL        2
#define    BACK_RIGHT_CHANNEL       3
//========================================================================
//======================= constant definition end ========================
//========================================================================





//========================================================================
//========================= pin definition start =========================
//========================================================================
//****************************** ultrosonic ******************************
#define    TRIGGER_PIN             19
#define    ECHO_PIN                18
//********************************* servo ********************************
#define    ORIENT_SERVO_PIN        23
//********************************* motor ********************************
#define    BACK_LEFT_EN_PIN        25
#define    BACK_RIGHT_EN_PIN       26
#define    BACK_DIR_PIN            32
#define    BACK_IDIR_PIN           33
//********************************* vive *********************************
#define    VIVE_PIN_1              21
#define    VIVE_PIN_2              39
//********************************** LED *********************************
#define    LED_PIN_1               14
#define    LED_PIN_2               14
//========================================================================
//========================== pin definition end ==========================
//========================================================================





//========================================================================
//================== global variables definition start ===================
//========================================================================
//********************************* mode *********************************
bool cur_mode = MANUAL;
//****************************** ultrosonic ******************************
long distance = 0;
long duration = 0;
//********************************* servo ********************************
int orient_pos = ORIENT_MIN_ANGLE;
int orient_dir = CLOCKWISE;
//********************************* motor ********************************
double back_left_speed = 0, back_right_speed = 0;
int back_dir = 0;
bool back_standby = true;
//********************************* vive *********************************
bool vive_state_1 = false, last_vive_state_1 = false;
int sync_cnt_1 = 0;
long signal_start_1 = 0;
long sync_end_1 = 0;
long x_coor_1 = 0, y_coor_1 = 0;
bool is_x_found_1 = false, is_y_found_1 = false;
bool vive_display_1 = false;

bool vive_state_2 = false, last_vive_state_2 = false;
int sync_cnt_2 = 0;
long signal_start_2 = 0;
long sync_end_2 = 0;
long x_coor_2 = 0, y_coor_2 = 0;
bool is_x_found_2 = false, is_y_found_2 = false;
bool vive_display_2 = false;
//========================================================================
//=================== global variables definition end ====================
//========================================================================





//========================================================================
//========================= main function start ==========================
//========================================================================
void setup(){
    Serial.begin(115200); 
    pinSetup();
    PWMSetup();
}  

void loop(){    
    viveReceive(); 

    backMotorControl();
    
//    ultraDectect();
    show();
}
//========================================================================
//========================== main function end ===========================
//========================================================================





//========================================================================
//=========================== pin setup start ============================
//========================================================================
void pinSetup(){
//****************************** ultrosonic ******************************
    pinMode(TRIGGER_PIN,OUTPUT);
    pinMode(ECHO_PIN,INPUT);
//********************************* motor ********************************
    pinMode(BACK_DIR_PIN,OUTPUT);
    pinMode(BACK_IDIR_PIN,OUTPUT);
//********************************* vive *********************************
    pinMode(VIVE_PIN_1,INPUT); 
    pinMode(VIVE_PIN_2,INPUT); 
}
//========================================================================
//============================ pin setup end =============================
//========================================================================





//========================================================================
//=========================== PWM setup start ============================
//========================================================================
void PWMSetup(){
//********************************* servo ********************************
    ledcSetup(ORIENT_CHANNEL,SERVO_FREQ_HZ,SERVO_RESOLUTION_BITS);
    ledcAttachPin(ORIENT_SERVO_PIN,ORIENT_CHANNEL); 
//********************************* motor ********************************
    ledcSetup(BACK_LEFT_CHANNEL,MOTOR_FREQ_HZ,MOTOR_RESOLUTION_BITS);
    ledcSetup(BACK_RIGHT_CHANNEL,MOTOR_FREQ_HZ,MOTOR_RESOLUTION_BITS);
    ledcAttachPin(BACK_LEFT_EN_PIN,BACK_LEFT_CHANNEL); 
    ledcAttachPin(BACK_RIGHT_EN_PIN,BACK_RIGHT_CHANNEL);
}
//========================================================================
//============================ PWM setup end =============================
//========================================================================





//========================================================================
//======================== distance detect start =========================
//========================================================================
void ultraDectect(){
    digitalWrite(TRIGGER_PIN,LOW);
    delayMicroseconds(2);
    
    digitalWrite(TRIGGER_PIN,HIGH);
    delayMicroseconds(10);

    digitalWrite(TRIGGER_PIN,LOW);
    duration = pulseIn(ECHO_PIN,HIGH);
    distance = (duration/2)*0.0343;
}
//========================================================================
//========================= distance detect end ==========================
//========================================================================





//========================================================================
//====================== back motor control start ========================
//========================================================================
void backMotorControl(){
    if(back_dir == CLOCKWISE){
        digitalWrite(BACK_DIR_PIN,HIGH);
        digitalWrite(BACK_IDIR_PIN,LOW); 
    }
    else if(back_dir == COUNTERCLOCKWISE){
        digitalWrite(BACK_DIR_PIN,LOW);
        digitalWrite(BACK_IDIR_PIN,HIGH); 
    } else{
        digitalWrite(BACK_DIR_PIN,LOW);
        digitalWrite(BACK_IDIR_PIN,LOW);      
    }
//    ledcWrite(BACK_LEFT_CHANNEL,back_left_speed);
//    ledcWrite(BACK_RIGHT_CHANNEL,back_right_speed);  
}
//========================================================================
//======================= back motor control end =========================
//========================================================================





//========================================================================
//========================= vive receive start ===========================
//========================================================================
void viveReceive(){
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
//========================================================================
//========================== vive receive end ============================
//========================================================================





//========================================================================
//========================== data print start ============================
//========================================================================
void show(){
    if(micros() % 500000 == 0){ // print every half second
//****************************** ultrosonic ******************************
//        Serial.print("Distance: ");
//        if(distance>=400 || distance<=2){
//            Serial.println("Out of range!");
//        } else {
//            Serial.print(distance);
//            Serial.println(" cm");
//        }
//********************************* servo ********************************
        Serial.print("Orient: ");
        Serial.print(orient_pos);
//********************************* motor ********************************
        if(back_dir == CLOCKWISE) Serial.println("CLOCKWISE");
        else if(back_dir == COUNTERCLOCKWISE) Serial.println("COUNTERCLOCKWISE");
        else Serial.println("STANDBY");
        
        Serial.print("back left PWM: ");
        Serial.print(back_left_speed);
        Serial.print("    back right PWM: ");
        Serial.println(back_right_speed);
//********************************* vive *********************************
        if(vive_display_1){    
//            Serial.print("x_1 = ");
//            Serial.print(x_coor_1);
//            Serial.print("    y_1 = ");
//            Serial.println(y_coor_1);
            vive_display_1 = false;
        }
        if(vive_display_2){
//            Serial.print("x_2 = ");
//            Serial.print(x_coor_2);
//            Serial.print("    y_2 = ");
//            Serial.println(y_coor_2);
            vive_display_2 = false;
        } 
    }
}
//========================================================================
//=========================== data print end =============================
//========================================================================
