//========================================================================
//====================== constant definition start =======================
//========================================================================
#define    CLOCKWISE                1  
#define    COUNTERCLOCKWISE         -1  
#define    LEFT                     -1
#define    MIDDLE                   0
#define    RIGHT                    1
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
#define    MOTOR_ZERO_SPEED         119
//========================================================================
//======================= constant definition end ========================
//========================================================================





//========================================================================
//========================= pin definition start =========================
//========================================================================
//********************************* servo ********************************
#define    ORIENT_SERVO_PIN        23
//********************************* motor ********************************
#define    BACK_LEFT_EN_PIN        25
#define    BACK_RIGHT_EN_PIN       26
#define    BACK_DIR_PIN            32
#define    BACK_IDIR_PIN           33
//========================================================================
//========================== pin definition end ==========================
//========================================================================





//========================================================================
//================== global variables definition start ===================
//========================================================================
//********************************* servo ********************************
int orient_pos = ORIENT_MIN_ANGLE;
int orient_dir = CLOCKWISE;
//********************************* motor ********************************
double back_left_speed = 0, back_right_speed = 0;
int back_dir = 0;
bool front_standby = true, back_standby = true;
//******************************* autonomy *******************************
int spin_dir = MIDDLE, last_spin_dir = MIDDLE;
long spin_start = 0;
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
    backMotorControl();
    show();
}
//========================================================================
//========================== main function end ===========================
//========================================================================





//========================================================================
//=========================== pin setup start ============================
//========================================================================
void pinSetup(){
//********************************* motor ********************************
    pinMode(BACK_DIR_PIN,OUTPUT);
    pinMode(BACK_IDIR_PIN,OUTPUT);
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
//============================== spin start ==============================
//========================================================================
void spin(){    
    if(spin_dir != MIDDLE && last_spin_dir == MIDDLE){
        
    }
    if(left_spin){
        
    }
}
//========================================================================
//=============================== spin end ===============================
//========================================================================





//========================================================================
//=========================== turn left start ============================
//========================================================================
void turnLeft(){    
    digitalWrite(BACK_DIR_PIN,HIGH);
    digitalWrite(BACK_IDIR_PIN,LOW); 
    ledcWrite(ORIENT_CHANNEL,ORIENT_MAX_ANGLE);
}
//========================================================================
//============================ turn left end =============================
//========================================================================





//========================================================================
//========================== turn right start ============================
//========================================================================
void turnRight(){    
    digitalWrite(BACK_DIR_PIN,HIGH);
    digitalWrite(BACK_IDIR_PIN,LOW); 
    ledcWrite(ORIENT_CHANNEL,ORIENT_MIN_ANGLE);
}
//========================================================================
//=========================== turn right end =============================
//========================================================================





//========================================================================
//=========================== backwards start ============================
//========================================================================
void backwards(){    
    digitalWrite(BACK_DIR_PIN,LOW);
    digitalWrite(BACK_IDIR_PIN,HIGH); 
    ledcWrite(BACK_LEFT_CHANNEL,MOTOR_RESOLUTION);
    ledcWrite(BACK_RIGHT_CHANNEL,MOTOR_RESOLUTION); 
}
//========================================================================
//============================ backwards end =============================
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
    ledcWrite(BACK_LEFT_CHANNEL,back_left_speed);
    ledcWrite(BACK_RIGHT_CHANNEL,back_right_speed);  
}
//========================================================================
//======================= back motor control end =========================
//========================================================================





//========================================================================
//========================== data print start ============================
//========================================================================
void show(){
    if(micros() % 500000 == 0){ // print every half second
//********************************* motor ********************************
        if(back_dir == CLOCKWISE) Serial.println("CLOCKWISE");
        else if(back_dir == COUNTERCLOCKWISE) Serial.println("COUNTERCLOCKWISE");
        else Serial.println("STANDBY");
        
        Serial.print("back left PWM: ");
        Serial.print(back_left_speed);
        Serial.print("    back right PWM: ");
        Serial.println(back_right_speed);
    }
}
//========================================================================
//=========================== data print end =============================
//========================================================================
