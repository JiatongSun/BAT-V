//========================================================================
//==================== MEAM 510 Final Auxiliary Board ====================
//========================================================================
//*****************************Team 04: BAT_V*****************************
//**************Members: Jiatong Sun, Xinyue Wei, Haojiong Lu*************
//========================================================================
//==================== MEAM 510 Final Auxiliary Board ====================
//========================================================================





//========================================================================
//====================== constant definition start =======================
//========================================================================
#define    CLOCKWISE                1  
#define    COUNTERCLOCKWISE         -1  
#define    STANDBY                  0
#define    AUTONOMOUS               1
#define    MANUAL                   0
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
//********************************* vive ********************************
#define    AIM_X                    3500
#define    AIM_Y                    5000
#define    X_ADJUST_THRESH          200
#define    Y_ADJUST_THRESH          200
#define    X_APPROACH_THRESH        500
#define    Y_APPROACH_THRESH        500
#define    BACK_TIME                300000
#define    TURN_TIME                300000
//========================================================================
//======================= constant definition end ========================
//========================================================================





//========================================================================
//========================= pin definition start =========================
//========================================================================
//****************************** connection ******************************
#define    CONNECT_PIN             35
//****************************** ultrosonic ******************************
#define    TRIGGER_PIN             19
#define    ECHO_PIN                18
//********************************* comm *********************************
#define    COMM_PIN_X_1            32
#define    COMM_PIN_Y_1            33
//********************************* vive *********************************
#define    VIVE_PIN_1              21
#define    VIVE_PIN_2              39
//********************************** LED *********************************
#define    LED_PIN_1               4
#define    LED_PIN_2               14
//========================================================================
//========================== pin definition end ==========================
//========================================================================





//========================================================================
//================== global variables definition start ===================
//========================================================================
//********************************* mode *********************************
bool cur_mode = MANUAL;
//****************************** interrupt *******************************
hw_timer_t* timer = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
//****************************** ultrosonic ******************************
volatile long duration = 0; 
volatile bool echo_rise_flag = false, echo_fall_flag = false;
long echo_start = 0;
float distance = 0, last_distance = 0;
float filter_weight = 3;
bool is_trigger = false;
//********************************* servo ********************************
int orient_pos = ORIENT_MIN_ANGLE;
int orient_dir = CLOCKWISE;
//********************************* motor ********************************
double back_left_speed = 0, back_right_speed = 0;
int back_dir = 0;
bool back_standby = true;
//********************************* vive *********************************
volatile bool vive_rise_flag_1 = false;
volatile bool vive_fall_flag_1 = false;
volatile long signal_start_1 = 0;
long sync_end_1 = 0;
long bandwidth_1 = 0;
int sync_cnt_1 = 0;
bool is_x_found_1 = false, is_y_found_1 = false;
int x_coor_1 = 0, y_coor_1 = 0;
bool vive_display_1 = false;

volatile bool vive_rise_flag_2 = false;
volatile bool vive_fall_flag_2 = false;
volatile long signal_start_2 = 0;
long sync_end_2 = 0;
long bandwidth_2 = 0;
int sync_cnt_2 = 0;
bool is_x_found_2 = false, is_y_found_2 = false;
int x_coor_2 = 0, y_coor_2 = 0;
bool vive_display_2 = false;
//******************************* autonomy *******************************
bool auxboard_state = true, last_auxboard_state = true;
int spin_dir = MIDDLE, last_spin_dir = MIDDLE;
int spin_mode = 0;
long spin_start = 0;
long dx = x_coor_1 - x_coor_2;
long dy = y_coor_1 - y_coor_2;
long x_car = 0.5 * (x_coor_1 + x_coor_2);
long y_car = 0.5 * (y_coor_1 + y_coor_2);
//========================================================================
//=================== global variables definition end ====================
//========================================================================





//========================================================================
//========================= main function start ==========================
//========================================================================
void setup(){
    Serial.begin(115200); 
    pinSetup();
    ISRSetup();
}  

void loop(){   
    checkPinSetup(); 
    viveReceive(); 
    ultraDetect();

    if(auxboard_state == AUTONOMOUS){
        ledcWrite(1, 120);
//        if(millis()%2000 == 0)spin();
//        toDestination();
//        rush();
    }
    
    show();
}
//========================================================================
//========================== main function end ===========================
//========================================================================




//========================================================================
//======================== check pin setup start  ========================
//========================================================================
void checkPinSetup(){
      auxboard_state = digitalRead(CONNECT_PIN);
      if(auxboard_state == AUTONOMOUS)digitalWrite(LED_BUILTIN, HIGH);
      else digitalWrite(LED_BUILTIN, LOW);
}
//========================================================================
//========================= check pin setup end  =========================
//========================================================================



//========================================================================
//============================ pin setup start ===========================
//========================================================================
void pinSetup(){
//****************************** connection ******************************
    pinMode(CONNECT_PIN, INPUT);
//****************************** ultrosonic ******************************
    pinMode(TRIGGER_PIN,OUTPUT);
    pinMode(ECHO_PIN,INPUT);
//    pinMode(COMM_PIN_X_1,OUTPUT);
//    pinMode(COMM_PIN_Y_1,OUTPUT);
    ledcAttachPin(COMM_PIN_X_1, 1);
    ledcSetup(1, 500, 8);
//********************************* vive *********************************
    pinMode(VIVE_PIN_1,INPUT); 
    pinMode(VIVE_PIN_2,INPUT); 
}
//========================================================================
//============================= pin setup end ============================
//========================================================================





//========================================================================
//=========================== Interrupt start ============================
//========================================================================
//****************************** ultrosonic ******************************
void IRAM_ATTR triggerStart() {
    portENTER_CRITICAL_ISR(&mux);
    is_trigger = true;
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR echoRise() {
    portENTER_CRITICAL_ISR(&mux);
    echo_rise_flag = true;
    echo_start = micros();
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR echoFall() {
    portENTER_CRITICAL_ISR(&mux);
    echo_fall_flag = true;
    duration = micros() - echo_start;
    distance = (duration/2)*0.0343;
    distance = (distance * filter_weight + last_distance) / (filter_weight + 1);
    last_distance = distance;
    portEXIT_CRITICAL_ISR(&mux);
}
//********************************* vive *********************************
void IRAM_ATTR viveRise_1() {
    portENTER_CRITICAL_ISR(&mux);
    vive_rise_flag_1 = true;
    signal_start_1 = micros();
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR viveFall_1() {
    portENTER_CRITICAL_ISR(&mux);
    vive_fall_flag_1 = true;
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR viveRise_2() {
    portENTER_CRITICAL_ISR(&mux);
    vive_rise_flag_2 = true;
    signal_start_2 = micros();
    portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR viveFall_2() {
    portENTER_CRITICAL_ISR(&mux);
    vive_fall_flag_2 = true;
    portEXIT_CRITICAL_ISR(&mux);
}
//========================================================================
//============================ Interrupt end =============================
//========================================================================





//========================================================================
//=========================== ISR setup start ============================
//========================================================================
void ISRSetup(){
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, triggerStart, true); 
    timerAlarmWrite(timer, 200000, true);
    timerAlarmEnable(timer);
    attachInterrupt(digitalPinToInterrupt(VIVE_PIN_1), viveRise_1, RISING);
    attachInterrupt(digitalPinToInterrupt(VIVE_PIN_2), viveRise_2, RISING);
    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoRise, RISING);
}
//========================================================================
//============================ ISR setup end =============================
//========================================================================





//========================================================================
//======================== distance detect start =========================
//========================================================================
void ultraDetect(){
    if(is_trigger){
        digitalWrite(TRIGGER_PIN,LOW);
        delayMicroseconds(2);
        digitalWrite(TRIGGER_PIN,HIGH);
        delayMicroseconds(10);
        digitalWrite(TRIGGER_PIN,LOW);
        if(echo_rise_flag){
            echo_rise_flag = false;
            detachInterrupt(digitalPinToInterrupt(ECHO_PIN));
            attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoFall, FALLING);
        } else if(echo_fall_flag){
            echo_fall_flag = false;
            detachInterrupt(digitalPinToInterrupt(ECHO_PIN));
            attachInterrupt(digitalPinToInterrupt(ECHO_PIN), echoRise, RISING);
        }
    }
}
//========================================================================
//========================= distance detect end ==========================
//========================================================================





//========================================================================
//========================= vive receive start ===========================
//========================================================================
void viveReceive(){
//********************************* vive 1 *******************************  
    if (vive_rise_flag_1) {
        vive_rise_flag_1 = false;
        detachInterrupt(digitalPinToInterrupt(VIVE_PIN_1));
        attachInterrupt(digitalPinToInterrupt(VIVE_PIN_1), viveFall_1, FALLING);
    } else if (vive_fall_flag_1){
        vive_fall_flag_1 = false;
        detachInterrupt(digitalPinToInterrupt(VIVE_PIN_1));
        attachInterrupt(digitalPinToInterrupt(VIVE_PIN_1), viveRise_1, RISING);
        
        bandwidth_1 = micros() - signal_start_1;

        if (bandwidth_1 > 60 && bandwidth_1 < 2000) {
            sync_cnt_1++;
            sync_end_1 = micros();
        }
        else if (bandwidth_1 < 60 && bandwidth_1 > 10) {
            if (sync_cnt_1 == 3) {
                is_x_found_1 = true;
                x_coor_1 = micros() - sync_end_1;
            } else if (sync_cnt_1 == 1) {
                is_y_found_1 = true;
                y_coor_1 = micros() - sync_end_1;
            }
            sync_cnt_1 = 0;
        }
    }
  
    if (is_x_found_1 && is_y_found_1) {
        vive_display_1 = true;
        is_x_found_1 = false;
        is_y_found_1 = false;
    }
//********************************* vive 2 *******************************  
    if (vive_rise_flag_2) {
        vive_rise_flag_2 = false;
        detachInterrupt(digitalPinToInterrupt(VIVE_PIN_2));
        attachInterrupt(digitalPinToInterrupt(VIVE_PIN_2), viveFall_2, FALLING);
    } else if (vive_fall_flag_2){
        vive_fall_flag_2 = false;
        detachInterrupt(digitalPinToInterrupt(VIVE_PIN_2));
        attachInterrupt(digitalPinToInterrupt(VIVE_PIN_2), viveRise_2, RISING);
        
        bandwidth_2 = micros() - signal_start_2;

        if (bandwidth_2 > 60 && bandwidth_2 < 2000) {
            sync_cnt_2++;
            sync_end_2 = micros();
        }
        else if (bandwidth_2 < 60 && bandwidth_2 > 10) {
            if (sync_cnt_2 == 3) {
                is_x_found_2 = true;
                x_coor_2 = micros() - sync_end_2;
            } else if (sync_cnt_2 == 1) {
                is_y_found_2 = true;
                y_coor_2 = micros() - sync_end_2;
            }
            sync_cnt_2 = 0;
        }
    }
  
    if (is_x_found_2 && is_y_found_2) {
        vive_display_2 = true;
        is_x_found_2 = false;
        is_y_found_2 = false;
    }
}
//========================================================================
//========================== vive receive end ============================
//========================================================================


void processData(){
    
}










////========================================================================
////=========================== turn left start ============================
////========================================================================
//void turnLeft(){    
//    digitalWrite(BACK_DIR_PIN,HIGH);
//    digitalWrite(BACK_IDIR_PIN,LOW); 
//    ledcWrite(ORIENT_CHANNEL,ORIENT_MAX_ANGLE);
//}
////========================================================================
////============================ turn left end =============================
////========================================================================
//
//
//
//
//
////========================================================================
////========================== turn right start ============================
////========================================================================
//void turnRight(){    
//    digitalWrite(BACK_DIR_PIN,HIGH);
//    digitalWrite(BACK_IDIR_PIN,LOW); 
//    ledcWrite(ORIENT_CHANNEL,ORIENT_MIN_ANGLE);
//}
////========================================================================
////=========================== turn right end =============================
////========================================================================
//
//
//
//
////========================================================================
////============================ forwards start ============================
////========================================================================
//void forwards(){    
//    digitalWrite(BACK_DIR_PIN,HIGH);
//    digitalWrite(BACK_IDIR_PIN,LOW); 
//    ledcWrite(BACK_LEFT_CHANNEL,MOTOR_RESOLUTION);
//    ledcWrite(BACK_RIGHT_CHANNEL,MOTOR_RESOLUTION); 
//}
////========================================================================
////============================= forwards end =============================
////========================================================================
//
//
//
//
//
////========================================================================
////=========================== backwards start ============================
////========================================================================
//void backwards(){    
//    digitalWrite(BACK_DIR_PIN,LOW);
//    digitalWrite(BACK_IDIR_PIN,HIGH); 
//    ledcWrite(BACK_LEFT_CHANNEL,MOTOR_RESOLUTION);
//    ledcWrite(BACK_RIGHT_CHANNEL,MOTOR_RESOLUTION); 
//}
////========================================================================
////============================ backwards end =============================
////========================================================================
//
//
//
//

////========================================================================
////======================= adjust X position start ========================
////========================================================================
//void adjustXPosition(){
//    if(((x_car<AIM_X)&&(dx>0)&&(dy<0)) || ((x_car>AIM_X)&&(dx<0)&&(dy>0))){
//      turnRight();
//      forwards();
//    }else if(((x_car<AIM_X)&&(dx>0)&&(dy>0)) || ((x_car>AIM_X)&&(dx<0)&&(dy<0))){
//      turnLeft();
//      forwards();
//    }else if(((x_car<AIM_X)&&(dx<0)&&(dy<0)) || ((x_car>AIM_X)&&(dx>0)&&(dy>0))){
//      if(abs(dx) > X_ADJUST_THRESH){
//        turnLeft();
//        backwards();
//      }else{
//        turnRight();
//        forwards();
//      }
//    }else if(((x_car<AIM_X)&&(dx<0)&&(dy>0)) || ((x_car>AIM_X)&&(dx>0)&&(dy<0))){
//      if(abs(dx) > X_ADJUST_THRESH){
//        turnRight();
//        backwards();
//      }else{
//        turnLeft();
//        forwards();
//      }
//    }
//}
////========================================================================
////======================== adjust X position end =========================
////========================================================================
//
//
//
//
////========================================================================
////======================= adjust Y position start ========================
////========================================================================
//void adjustYPosition(){
//    if(((y_car<AIM_Y)&&(dy>0)&&(dx<0)) || ((y_car>AIM_Y)&&(dy<0)&&(dx>0))){
//      turnRight();
//      forwards();
//    }else if(((y_car<AIM_Y)&&(dy>0)&&(dx>0)) || ((y_car>AIM_Y)&&(dy<0)&&(dx<0))){
//      turnLeft();
//      forwards();
//    }else if(((y_car<AIM_Y)&&(dy<0)&&(dx<0)) || ((y_car>AIM_Y)&&(dy>0)&&(dx>0))){
//      if(abs(dy) > Y_ADJUST_THRESH){
//        turnLeft();
//        backwards();
//      }else{
//        turnRight();
//        forwards();
//      }
//    }else if(((y_car<AIM_Y)&&(dy<0)&&(dx>0)) || ((y_car>AIM_Y)&&(dy>0)&&(dx<0))){
//      if(abs(dy) > Y_ADJUST_THRESH){
//        turnRight();
//        backwards();
//      }else{
//        turnLeft();
//        forwards();
//      }
//    }
//}
////========================================================================
////======================== adjust Y position end =========================
////========================================================================
//
//
//
////========================================================================
////========================== to destination start=========================
////========================================================================
//void toDestination(){
//    if((abs(x_car - AIM_X) > X_APPROACH_THRESH) || (abs(y_car - AIM_Y) > Y_APPROACH_THRESH) ) {
//      adjustXPosition();
//    }else if((abs(dx) < X_ADJUST_THRESH) && abs(x_car - AIM_X)> X_APPROACH_THRESH){
//      forwards();
//    }else if((abs(dy) > Y_ADJUST_THRESH)){
//      if(((AIM_Y -y_car) * dx)> 0) turnLeft();
//      else if (((AIM_Y -y_car) * dx)< 0) turnRight();
//    }else if((abs(dy) < Y_ADJUST_THRESH) && abs(y_car - AIM_Y)> Y_APPROACH_THRESH){
//      forwards();
//    }
//}
////========================================================================
////========================== to destination end=========================
////========================================================================
//
//

////========================================================================
////============================== spin start ==============================
////========================================================================
//void spin(){ 
//    if(spin_dir != MIDDLE && last_spin_dir == MIDDLE){
//        spin_start = micros();
//        spin_mode = 1;
//        backwards();
//    }
//    if(micros() - spin_start > BACK_TIME && spin_mode == 1){
//        spin_start = micros();
//        spin_mode = 2;
//        if(spin_dir == LEFT) turnLeft();
//        else if(spin_dir == RIGHT) turnRight();
//        spin_mode == 2;
//    }
//    if(micros() - spin_start > TURN_TIME && spin_mode == 2){
//        forwards();
//        spin_mode = 0;
//    }
//}
//
////========================================================================
////=============================== spin end ===============================
////========================================================================
//
//
//
//
//
////========================================================================
////============================== rush start ==============================
////========================================================================
//void rush(){
//
//    if((abs(x_car - AIM_X) < X_APPROACH_THRESH) && (abs(y_car - AIM_Y) < Y_APPROACH_THRESH) && abs(dx < X_ADJUST_THRESH)){
//      forwards();
//    }
//}
////========================================================================
////============================= rush end =================================
////========================================================================




//========================================================================
//========================== data print start ============================
//========================================================================
void show(){
    if(millis() % 500 == 0){ // print every half second
        if(auxboard_state == AUTONOMOUS)Serial.println("AUTONOMOUS");
        else Serial.println("MANUAL");
//****************************** ultrosonic ******************************
        Serial.print("Distance: ");
        if(distance>=400 || distance<=2){
            Serial.println("Out of range!");
        } else {
            Serial.print(distance);
            Serial.println(" cm");
        }
//********************************* vive *********************************
        if(vive_display_1){    
            Serial.print("x_1 = ");
            Serial.print(x_coor_1);
            Serial.print("    y_1 = ");
            Serial.println(y_coor_1);
            vive_display_1 = false;
        }
        if(vive_display_2){
            Serial.print("x_2 = ");
            Serial.print(x_coor_2);
            Serial.print("    y_2 = ");
            Serial.println(y_coor_2);
            vive_display_2 = false;
        } 
    }
}
//========================================================================
//=========================== data print end =============================
//========================================================================
