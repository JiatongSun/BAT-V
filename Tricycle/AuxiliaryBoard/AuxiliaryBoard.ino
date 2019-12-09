//========================================================================
//==================== MEAM 510 Final Auxiliary Board ====================
//========================================================================
//*****************************Team 04: BAT_V*****************************
//**************Members: Jiatong Sun, Xinyue Wei, Haojiong Lu*************
//========================================================================
//==================== MEAM 510 Final Auxiliary Board ====================
//========================================================================





//========================================================================
//============================ library start =============================
//========================================================================
#include "driver/i2s.h"
#include "freertos/queue.h"
#include <pgmspace.h>

#include "audio_example_file.h"
//========================================================================
//============================= library end ==============================
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
#define    AIM_X                    3600
#define    AIM_Y                    5760
#define    X_ADJUST_THRESH          200
#define    Y_ADJUST_THRESH          200
#define    X_APPROACH_THRESH        500
#define    Y_APPROACH_THRESH        500
#define    BACK_TIME                300000
#define    TURN_TIME                300000
//****************************** ultrosonic ******************************
#define    MIN_DIST                 6
#define    MAX_DIST                 20
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
//********************************* servo ********************************
#define    ORIENT_SERVO_PIN        27
//********************************* motor ********************************
#define    BACK_LEFT_EN_PIN        17
#define    BACK_RIGHT_EN_PIN       16
#define    BACK_DIR_PIN            32
#define    BACK_IDIR_PIN           33
//********************************* vive *********************************
#define    VIVE_PIN_1              21
#define    VIVE_PIN_2              39
//********************************** LED *********************************
#define    LED_PIN_1               4
#define    LED_PIN_2               14
//********************************** I2S *********************************
#define    BCK_PIN                 26
#define    LRCK_PIN                25
#define    DATA_OUT_PIN            22
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
float dist_thresh = 10;
//********************************* i2s **********************************
int i2s_num = 0;
i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 641
};

i2s_pin_config_t pin_config = {
    .bck_io_num = BCK_PIN,
    .ws_io_num = LRCK_PIN,
    .data_out_num = DATA_OUT_PIN, 
    .data_in_num = -1
};
//========================================================================
//=================== global variables definition end ====================
//========================================================================





//========================================================================
//========================= main function start ==========================
//========================================================================
void setup(){
    Serial.begin(115200); 
    pinEnable();
    PWMSetup();
    ISRSetup();
}  

void loop(){   
    checkPinSetup(); 
    if(auxboard_state == AUTONOMOUS){
//        avoidBarrier()
        viveReceive(); 
//        ultraDetect();
        toDestination();
//        rush();
//        show();
    } 
    else if(auxboard_state == MANUAL){
        if (millis() % 30000 == 0){
            playPROGMEMsample(audio_table);
        }
    }
    
}
//========================================================================
//========================== main function end ===========================
//========================================================================




//========================================================================
//======================== check pin setup start  ========================
//========================================================================
void checkPinSetup(){
      auxboard_state = digitalRead(CONNECT_PIN);
      if((auxboard_state == MANUAL) && (last_auxboard_state != auxboard_state)){
        pinDisable();
      }else if((auxboard_state == AUTONOMOUS) && (last_auxboard_state != auxboard_state)){
        pinEnable();
      }
      last_auxboard_state = auxboard_state;
}
//========================================================================
//========================= check pin setup end  =========================
//========================================================================



//========================================================================
//=========================== pin enable start ===========================
//========================================================================
void pinEnable(){
//****************************** connection ******************************
    pinMode(CONNECT_PIN, INPUT);
//****************************** ultrosonic ******************************
    pinMode(TRIGGER_PIN,OUTPUT);
    pinMode(ECHO_PIN,INPUT);
//********************************* motor ********************************
    pinMode(BACK_DIR_PIN,OUTPUT);
    pinMode(BACK_IDIR_PIN,OUTPUT);
//********************************* vive *********************************
    pinMode(VIVE_PIN_1,INPUT); 
    pinMode(VIVE_PIN_2,INPUT); 
//********************************* servo *********************************
    PWMSetup(); 
}
//========================================================================
//============================ pin enable end ============================
//========================================================================





//========================================================================
//=========================== pin disable start ==========================
//========================================================================
void pinDisable(){
//********************************* motor ********************************
    pinMode(BACK_DIR_PIN,INPUT);
    pinMode(BACK_IDIR_PIN,INPUT);
    pinMode(BACK_LEFT_EN_PIN,INPUT);
    pinMode(BACK_RIGHT_EN_PIN,INPUT);
//********************************* servo *********************************
    pinMode(ORIENT_SERVO_PIN, INPUT);
}
//========================================================================
//============================ pin disable end ===========================
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
//========================= avoid barrier start ==========================
//========================================================================
void avoidBarrier(){
    if(distance < MIN_DIST || distance > MAX_DIST) return;
    if(distance < dist_thresh) {
        return;
    }
}
//========================================================================
//========================= avoid barrier end ============================
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
    }else if((abs(dx) < X_ADJUST_THRESH) && abs(x_car - AIM_X)> X_APPROACH_THRESH){
      forwards();
    }else if((abs(dy) > Y_ADJUST_THRESH)){
      if(((AIM_Y -y_car) * dx)> 0) turnLeft();
      else if (((AIM_Y -y_car) * dx)< 0) turnRight();
    }else if((abs(dy) < Y_ADJUST_THRESH) && abs(y_car - AIM_Y)> Y_APPROACH_THRESH){
      forwards();
    }
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
//============================== rush start ==============================
//========================================================================
void rush(){

    if((abs(x_car - AIM_X) < X_APPROACH_THRESH) && (abs(y_car - AIM_Y) < Y_APPROACH_THRESH) && abs(dx < X_ADJUST_THRESH)){
      forwards();
    }
}
//========================================================================
//============================= rush end =================================
//========================================================================






//========================================================================
//============================= I2S start ================================
//========================================================================
int i2s_write_sample_nb(uint16_t sample) {
    return i2s_write_bytes((i2s_port_t)i2s_num, (const char *)&sample, sizeof(uint32_t), 100);
}

//Main function to play samples from PROGMEM
void playPROGMEMsample(const uint16_t* audioSample) {
    uint32_t sampleSize = sizeof(audioSample) * 4;
    uint32_t counter = 0;
    i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);
    i2s_set_pin((i2s_port_t)i2s_num, &pin_config);
    i2s_set_sample_rates((i2s_port_t)i2s_num, 16000);
    uint32_t readData;
    while (audioSample)  {
        readData = pgm_read_dword(&audioSample[counter++]);
        if (readData == NULL) break;
        i2s_write_sample_nb(readData);
    }
    i2s_driver_uninstall((i2s_port_t)i2s_num); //stop & destroy i2s driver
}
//========================================================================
//============================== I2S end =================================
//========================================================================






//========================================================================
//========================== data print start ============================
//========================================================================
void show(){
    if(millis() % 500 == 0){ // print every half second
//****************************** ultrosonic ******************************
        Serial.print("Distance: ");
        if(distance>=400 || distance<=2){
            Serial.println("Out of range!");
        } else {
            Serial.print(distance);
            Serial.println(" cm");
        }
//********************************* servo ********************************
        Serial.print("Orient: ");
        Serial.println(orient_pos);
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
