#define DIFF(a,b,thresh)  (abs(a-b) - thresh)


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
//******************************** autonomy ******************************
#define    AIM_X                    3500
#define    AIM_Y                    5000
#define    X_ADJUST_THRESH          200
#define    Y_ADJUST_THRESH          200
#define    X_APPROACH_THRESH        200
#define    Y_APPROACH_THRESH        200
#define    BACK_TIME                300000
#define    TURN_TIME                300000
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
int spin_mode = 0;
long spin_start = 0;

long front_vive_x = 0, front_vive_y = 0, back_vive_x = 0, back_vive_y = 0;
long dx = front_vive_x - back_vive_x;
long dy = front_vive_y - back_vive_y;
long x_car = 0.5 * (front_vive_x + back_vive_x);
long y_car = 0.5 * (front_vive_y + back_vive_y);
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
    //backMotorControl();
//    show();
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
//============================ forwards start ============================
//========================================================================
void forwards(){    
    digitalWrite(BACK_DIR_PIN,HIGH);
    digitalWrite(BACK_IDIR_PIN,LOW); 
    ledcWrite(BACK_LEFT_CHANNEL,MOTOR_RESOLUTION);
    ledcWrite(BACK_RIGHT_CHANNEL,MOTOR_RESOLUTION); 
}
//========================================================================
//============================= forwards end =============================
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
//======================= adjust X position start ========================
//========================================================================
void adjustXPosition(){
    if(((x_car<AIM_X)&&(dx>0)&&(dy<0)) || ((x_car>AIM_X)&&(dx<0)&&(dy>0))){
      turnRight();
      forwards();
    }else if(((x_car<AIM_X)&&(dx>0)&&(dy>0)) || ((x_car>AIM_X)&&(dx<0)&&(dy<0))){
      turnLeft();
      forwards();
    }else if(((x_car<AIM_X)&&(dx<0)&&(dy<0)) || ((x_car>AIM_X)&&(dx>0)&&(dy>0))){
      if(abs(dx) > X_ADJUST_THRESH){
        turnLeft();
        backwards();
      }else{
        turnRight();
        forwards();
      }
    }else if(((x_car<AIM_X)&&(dx<0)&&(dy>0)) || ((x_car>AIM_X)&&(dx>0)&&(dy<0))){
      if(abs(dx) > X_ADJUST_THRESH){
        turnRight();
        backwards();
      }else{
        turnLeft();
        forwards();
      }
    }
}
//========================================================================
//======================== adjust X position end =========================
//========================================================================




//========================================================================
//======================= adjust Y position start ========================
//========================================================================
void adjustYPosition(){
    if(((y_car<AIM_Y)&&(dy>0)&&(dx<0)) || ((y_car>AIM_Y)&&(dy<0)&&(dx>0))){
      turnRight();
      forwards();
    }else if(((y_car<AIM_Y)&&(dy>0)&&(dx>0)) || ((y_car>AIM_Y)&&(dy<0)&&(dx<0))){
      turnLeft();
      forwards();
    }else if(((y_car<AIM_Y)&&(dy<0)&&(dx<0)) || ((y_car>AIM_Y)&&(dy>0)&&(dx>0))){
      if(abs(dy) > Y_ADJUST_THRESH){
        turnLeft();
        backwards();
      }else{
        turnRight();
        forwards();
      }
    }else if(((y_car<AIM_Y)&&(dy<0)&&(dx>0)) || ((y_car>AIM_Y)&&(dy>0)&&(dx<0))){
      if(abs(dy) > Y_ADJUST_THRESH){
        turnRight();
        backwards();
      }else{
        turnLeft();
        forwards();
      }
    }
}
//========================================================================
//======================== adjust Y position end =========================
//========================================================================



//========================================================================
//========================== to destination start=========================
//========================================================================
void toDestination(){
    if((abs(x_car - AIM_X) > X_APPROACH_THRESH) || (abs(y_car - AIM_Y) > Y_APPROACH_THRESH) ) {
      adjustXPosition();
    }
    if((abs(dx) < X_ADJUST_THRESH) && abs(x_car - AIM_X)> X_APPROACH_THRESH) forwards();
    if((abs(dy) > Y_ADJUST_THRESH)){
      if(((AIM_Y -y_car) * dx)> 0) turnLeft();
      else if (((AIM_Y -y_car) * dx)< 0) turnRight();
    }
    if((abs(dy) < Y_ADJUST_THRESH) && abs(y_car - AIM_Y)> Y_APPROACH_THRESH) forwards();
}
//========================================================================
//========================== to destination end=========================
//========================================================================



//========================================================================
//============================== spin start ==============================
//========================================================================
void spin(){ 
    if(spin_dir != MIDDLE && last_spin_dir == MIDDLE){
        spin_start = micros();
        spin_mode = 1;
        backwards();
    }
    if(micros() - spin_start > BACK_TIME && spin_mode == 1){
        spin_start = micros();
        spin_mode = 2;
        if(spin_dir == LEFT) turnLeft();
        else if(spin_dir == RIGHT) turnRight();
        spin_mode == 2;
    }
    if(micros() - spin_start > TURN_TIME && spin_mode == 2){
        forwards();
        spin_mode = 0;
    }
}
//========================================================================
//=============================== spin end ===============================
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
