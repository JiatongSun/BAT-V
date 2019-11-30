//========================================================================
//============================ MEAM 510 Final ============================
//========================================================================
//*****************************Team 04: BAT_V*****************************
//**************Members: Jiatong Sun, Xinyue Wei, Haojiong Lu*************
//========================================================================
//============================ MEAM 510 Final ============================
//========================================================================





//========================================================================
//============================ library start =============================
//========================================================================
#include <WiFi.h>
#include <WiFiUDP.h>
//========================================================================
//============================= library end ==============================
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
#define    WEAPON_CHANNEL           1
#define    ORIENT_MIN_ANGLE         300
#define    ORIENT_MAX_ANGLE         450
#define    WEAPON_MIN_ANGLE         200
#define    WEAPON_MID_ANGLE         600
#define    WEAPON_MAX_ANGLE         1000
//********************************* motor ********************************
#define    MOTOR_RESOLUTION_BITS    8
#define    MOTOR_RESOLUTION         255
#define    MOTOR_FREQ_HZ            1000    
#define    BACK_LEFT_CHANNEL        2
#define    BACK_RIGHT_CHANNEL       3
#define    MOTOR_ZERO_SPEED         119
//******************************** encoder *******************************
#define    PERIOD_THRESH            10000
#define    ERR_NUM_THRESH           180
#define    PAUSE_THRESH             20000
//========================================================================
//======================= constant definition end ========================
//========================================================================





//========================================================================
//========================= pin definition start =========================
//========================================================================
//****************************** ultrosonic ******************************
#define    TRIGGER_PIN             23
#define    ECHO_PIN                22
//********************************* servo ********************************
#define    ORIENT_SERVO_PIN        17
#define    WEAPON_SERVO_PIN        16
//********************************* motor ********************************
#define    BACK_LEFT_EN_PIN        32
#define    BACK_RIGHT_EN_PIN       27
#define    BACK_DIR_PIN            26
#define    BACK_STANDBY_PIN        21
#define    FRONT_DIR_PIN           19
#define    FRONT_EN_PIN            14 
//******************************** encoder *******************************
#define    LEFT_ENCODER_PIN        36
#define    RIGHT_ENCODER_PIN       39
//********************************* vive *********************************
#define    VIVE_PIN                13
//******************************** top hat *******************************
#define    I2C_CLK_PIN             33
#define    I2C_DAT_PIN             25
#define    NEOPIXEL_PIN            12
//========================================================================
//========================== pin definition end ==========================
//========================================================================





//========================================================================
//================== global variables definition start ===================
//========================================================================
//********************************* mode *********************************
bool cur_mode = MANUAL;
//********************************* timer ********************************
hw_timer_t* timer = NULL;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
//********************************* WiFi *********************************
const char* ssid = "BAT_V", * pass = "BATATABANANA";  
IPAddress IPlocal(192,168,1,179), IPtarget(192,168,1,171);  
WiFiUDP udp;
unsigned int UDPlocalPort = 2816, UDPtargetPort = 2918; 
const int packetSize = 100; 
byte receive_buffer[packetSize];
int VRX = 0, VRY = 0, PTM = 0, SW_1 = 0, SW_2 = 0;
//****************************** ultrosonic ******************************
long distance = 0;
long duration = 0;
//********************************* servo ********************************
int orient_pos = ORIENT_MIN_ANGLE;
int orient_dir = CLOCKWISE;
int weapon_pos = WEAPON_MIN_ANGLE;
int weapon_dir = CLOCKWISE;
bool weapon_mode = MANUAL;
bool weapon_auto = false;
//********************************* motor ********************************
double back_left_speed = 0, back_right_speed = 0;
int back_dir = 0;
bool front_standby = true, back_standby = true;
//******************************** encoder *******************************
bool left_encoder_state = false, last_left_encoder_state = false;
bool right_encoder_state = false, last_right_encoder_state = false;
long left_start = 0, right_start = 0;
long left_period = 0, right_period = 0;
int  left_data_cnt = 0, right_data_cnt = 0;
long left_data[360] = {0}, right_data[360] = {0};
double left_rps = 0, right_rps = 0;
long left_fall = 0, right_fall = 0;
//********************************* PID **********************************
bool is_pid = false;
double diff_rps = 0;
double last_diff_rps = 0;
double all_diff_rps = 0;
//********************************* vive *********************************
bool vive_state = false, last_vive_state = false;
int sync_cnt = 0;
long signal_start = 0, sync_end = 0;
long x_coor = 0, y_coor = 0;
bool is_x_found = false, is_y_found = false;
//========================================================================
//=================== global variables definition end ====================
//========================================================================





//========================================================================
//========================= main function start ==========================
//========================================================================
void setup(){
    Serial.begin(115200); 
    STA_UDP_Set();
    timerSetup();
    pinSetup();
    PWMSetup();
}  

void loop(){
    WiFi_Reconnect();
    UDPreceiveData();
    orientServoControl();
    weaponServoControl();
    backMotorControl();
    frontMotorControl();
    encoderCalc();
    viveReceive();  
//    pidControl();
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
    pinMode(BACK_STANDBY_PIN,OUTPUT);
    pinMode(FRONT_EN_PIN,OUTPUT);
    pinMode(FRONT_DIR_PIN,OUTPUT);
//******************************** encoder *******************************
    pinMode(LEFT_ENCODER_PIN,INPUT);
    pinMode(RIGHT_ENCODER_PIN,INPUT);
//********************************* vive *********************************
    pinMode(VIVE_PIN,INPUT); 
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
    ledcSetup(WEAPON_CHANNEL,SERVO_FREQ_HZ,SERVO_RESOLUTION_BITS);
    ledcAttachPin(ORIENT_SERVO_PIN,ORIENT_CHANNEL); 
    ledcAttachPin(WEAPON_SERVO_PIN,WEAPON_CHANNEL); 
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
//============================ interrupt start ===========================
//========================================================================
void IRAM_ATTR onTimer(){
    portENTER_CRITICAL_ISR(&mux);
    weapon_auto = true;
    portEXIT_CRITICAL_ISR(&mux);
}
//========================================================================
//============================= interrupt end ============================
//========================================================================





//========================================================================
//========================== timer setup start ===========================
//========================================================================
void timerSetup(){
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 2000, true); 
    timerAlarmEnable(timer);       
}
//========================================================================
//========================== timer setup start ===========================
//========================================================================





//========================================================================
//=========================== UDP setup start ============================
//========================================================================
void STA_UDP_Set(){
    // Wifi indication
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on
    delay(1000);
    
    Serial.print("Connecting to: "); Serial.println(ssid);  // connect to WiFi   
    WiFi.begin(ssid, pass);  // connect to network with password
    IPAddress gateway(192,168,1,1);             // init gateway IP
    IPAddress subnet(255,255,255,0);            // init subnet mask
    WiFi.config(IPlocal, gateway, subnet);      // set IP address of ESP
    udp.begin(UDPlocalPort);    // configure a port for UDP comms
    // hold the code here and wait until the WiFi is connected
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected!"); 
    
    //Turn OFF to indicate WiFi connected successfully
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
}
//========================================================================
//============================ UDP setup end =============================
//========================================================================





//========================================================================
//========================= WiFi reconnect start =========================
//========================================================================
void WiFi_Reconnect(){  // if WiFi disconnected, try to reconnect automatically
    if (WiFi.status() != WL_CONNECTED){
        digitalWrite(LED_BUILTIN, HIGH);  
        WiFi.begin(ssid, pass); 
        IPAddress gateway(192,168,1,1);
        IPAddress subnet(255,255,255,0);
        WiFi.config(IPlocal, gateway, subnet);
        udp.begin(UDPlocalPort);
    } 
    while(WiFi.status() != WL_CONNECTED);
    digitalWrite(LED_BUILTIN, LOW);
}
//========================================================================
//========================== WiFi reconnect end ==========================
//========================================================================





//========================================================================
//=========================== UDP receive start ==========================
//========================================================================
void UDPreceiveData(){
    if (udp.parsePacket())
    {
        udp.read(receive_buffer, packetSize);

        VRX = receive_buffer[0];
        VRY = receive_buffer[1];
        PTM = receive_buffer[2];
        SW_1 = receive_buffer[3];
        SW_2 = receive_buffer[4];

        back_left_speed = abs(VRX - MOTOR_ZERO_SPEED);
        if(VRX > MOTOR_ZERO_SPEED) {
            back_dir = CLOCKWISE;
            back_left_speed = map(back_left_speed,1,MOTOR_RESOLUTION - MOTOR_ZERO_SPEED,0,MOTOR_RESOLUTION);
        }
        else if(VRX < MOTOR_ZERO_SPEED) {
            back_dir = COUNTERCLOCKWISE;
            back_left_speed = map(back_left_speed,0,MOTOR_ZERO_SPEED - 1,0,MOTOR_RESOLUTION);
        }
        else {
            back_dir = STANDBY;
            back_left_speed = 0;
        }
        back_right_speed = back_left_speed;

        orient_pos = map(VRY, 1, 255, ORIENT_MIN_ANGLE, ORIENT_MAX_ANGLE);
        
        if(SW_1 == 1)weapon_mode = AUTONOMOUS;
        else weapon_mode = MANUAL;
        
        if(SW_2 == 1)front_standby = true;
        else front_standby = false ;
    }
}
//========================================================================
//============================ UDP receive end ===========================
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
//===================== orient servo control start =======================
//========================================================================
void orientServoControl(){
    ledcWrite(ORIENT_CHANNEL,orient_pos);
}
//========================================================================
//====================== orient servo control end ========================
//========================================================================





//========================================================================
//====================== weapon servo control start ======================
//========================================================================
void weaponServoControl(){
    weaponAutoControl();
    weaponManualControl();
}
//========================================================================
//====================== weapon servo control end ========================
//========================================================================





//========================================================================
//====================== weapon auto control start =======================
//========================================================================
void weaponAutoControl(){
    if(weapon_auto){
        weapon_auto = false;
        if(weapon_mode == AUTONOMOUS){
            int max_pos = map(PTM, 1, 255, WEAPON_MIN_ANGLE, WEAPON_MAX_ANGLE);
            int angle_max = WEAPON_MID_ANGLE + abs(max_pos - WEAPON_MID_ANGLE);
            int angle_min = WEAPON_MID_ANGLE - abs(max_pos - WEAPON_MID_ANGLE);

            if(weapon_pos < angle_min || weapon_pos > angle_max){
                weapon_pos = WEAPON_MID_ANGLE;
            }
            ledcWrite(WEAPON_CHANNEL,weapon_pos);
            weapon_pos += weapon_dir;
            if(weapon_pos == angle_max || weapon_pos == angle_min){
                weapon_dir = -weapon_dir;
            }
        }
    }
}
//========================================================================
//======================= weapon auto control end ========================
//========================================================================





//========================================================================
//===================== weapon manual control start ======================
//========================================================================
void weaponManualControl(){
    if(weapon_mode == MANUAL){
        weapon_pos = map(PTM, 1, 255, WEAPON_MIN_ANGLE, WEAPON_MAX_ANGLE);
        ledcWrite(WEAPON_CHANNEL,weapon_pos);
    }
}
//========================================================================
//====================== weapon manual control end =======================
//========================================================================





//========================================================================
//====================== back motor control start ========================
//========================================================================
void backMotorControl(){
    if(is_pid){
        if(back_dir == STANDBY) digitalWrite(BACK_STANDBY_PIN,LOW);
        else{
            digitalWrite(BACK_STANDBY_PIN,HIGH);
            if(back_dir == CLOCKWISE)digitalWrite(BACK_DIR_PIN,HIGH);
            else if(back_dir == COUNTERCLOCKWISE)digitalWrite(BACK_DIR_PIN,LOW);
        }
        ledcWrite(BACK_LEFT_CHANNEL,back_left_speed);
        ledcWrite(BACK_RIGHT_CHANNEL,back_right_speed);
    } else {
        digitalWrite(BACK_STANDBY_PIN,HIGH);
        if(back_dir == CLOCKWISE)digitalWrite(BACK_DIR_PIN,HIGH);
        else if(back_dir == COUNTERCLOCKWISE)digitalWrite(BACK_DIR_PIN,LOW);
        ledcWrite(BACK_LEFT_CHANNEL,back_left_speed);
        ledcWrite(BACK_RIGHT_CHANNEL,back_right_speed);
    }
    
}
//========================================================================
//======================= back motor control end =========================
//========================================================================





//========================================================================
//===================== encoder calculation start ========================
//========================================================================  
void encoderCalc(){
//***************************** left encoder *****************************  
    if(digitalRead(LEFT_ENCODER_PIN)==HIGH) left_encoder_state = true;
    else left_encoder_state = false;
    
    if(left_encoder_state && !last_left_encoder_state) left_start = micros();
    else if(!left_encoder_state && last_left_encoder_state){
        left_fall = micros();
        left_period = 2 * (micros() - left_start);
        if(left_period > PERIOD_THRESH)left_period = 0;
        left_data[left_data_cnt] = left_period;
        float round_period = 0;
        int err_num_cnt = 0;
        for (int i=0; i<360; ++i) {
            if(!left_data[i]) err_num_cnt++;
            round_period += left_data[i]; 
        }
        if(err_num_cnt < ERR_NUM_THRESH){
            round_period = round_period * 360 / (360 - err_num_cnt);
            left_rps = 1000000.0 / round_period;
        }
        else left_rps = 0;
        left_data_cnt = (left_data_cnt + 1) % 360;
    }

    if(micros() - left_fall > PAUSE_THRESH) left_rps = 0;
    
    last_left_encoder_state = left_encoder_state;    
//**************************** right encoder ***************************** 
    if(digitalRead(RIGHT_ENCODER_PIN)==HIGH) right_encoder_state = true;
    else right_encoder_state = false;
    
    if(right_encoder_state && !last_right_encoder_state) right_start = micros();
    else if(!right_encoder_state && last_right_encoder_state){
        right_fall = micros();
        right_period = 2 * (micros() - right_start);
        if(right_period > PERIOD_THRESH)right_period = 0;
        right_data[right_data_cnt] = right_period;
        float round_period = 0;
        int err_num_cnt = 0;
        for (int i=0; i<360; ++i) {
            if(!right_data[i]) err_num_cnt++;
            round_period += right_data[i]; 
        }
        if(err_num_cnt < ERR_NUM_THRESH){
            round_period = round_period * 360 / (360 - err_num_cnt);
            right_rps = 1000000.0 / round_period;
        }
        else right_rps = 0;
        right_data_cnt = (right_data_cnt + 1) % 360;
    }

    if(micros() - right_fall > PAUSE_THRESH) right_rps = 0;

    last_right_encoder_state = right_encoder_state;
}
//========================================================================
//====================== encoder calculation end =========================
//========================================================================

 



//========================================================================
//========================= PID control start ============================
//========================================================================
void pidControl(){
    double P = 2, D = 0.4, I = 0.1;
    diff_rps = left_rps - right_rps; 
    all_diff_rps += diff_rps;
    double d_error = D * (diff_rps - last_diff_rps);
    double p_error = P * diff_rps;
    double i_error = I * all_diff_rps;
    back_right_speed += (d_error + p_error + i_error);
    back_right_speed = max(min(back_right_speed,255.0),0.0);
    last_diff_rps = diff_rps;
}
//========================================================================
//========================== PID control end =============================
//========================================================================





//========================================================================
//===================== front motor control start ========================
//========================================================================
void frontMotorControl(){
    if(left_rps < 0.5 || right_rps < 0.5){
        digitalWrite(FRONT_EN_PIN,LOW); 
        digitalWrite(FRONT_DIR_PIN, LOW);  
    } else {
        if(front_standby) {
            digitalWrite(FRONT_EN_PIN,HIGH);
            digitalWrite(FRONT_DIR_PIN, HIGH);
        } else {
            digitalWrite(FRONT_EN_PIN,LOW); 
            digitalWrite(FRONT_DIR_PIN, LOW);
        }
    }
}
//========================================================================
//====================== front motor control end =========================
//========================================================================





//========================================================================
//========================= vive receive start ===========================
//========================================================================
void viveReceive(){
    if(digitalRead(VIVE_PIN)==HIGH) vive_state = true;
    else vive_state = false;

    int x_temp = 0, y_temp = 0;

    if(vive_state && !last_vive_state) signal_start = micros();
    else if(!vive_state && last_vive_state){
        long bandwidth = micros() - signal_start;
        
        if(bandwidth > 60){
            sync_cnt ++;
            sync_end = micros();
        } else if (bandwidth < 60 && bandwidth > 0){
            if(sync_cnt == 3){
                is_x_found = true;
                x_temp = micros() - sync_end;
            } else if(sync_cnt == 1){
                is_y_found = true;
                y_temp = micros() - sync_end;
            }
            sync_cnt = 0;
        }
    }
    last_vive_state = vive_state;
    if(is_x_found && is_y_found){
        x_coor = x_temp;
        y_coor = y_temp;
        is_x_found = false;
        is_y_found = false;
    }
}
//========================================================================
//========================== vive receive end ============================
//========================================================================





//========================================================================
//========================== data print start ============================
//========================================================================
void show(){
    if(millis() % 500 == 0){ // print every half second
//********************************* WiFi *********************************
        Serial.print("VRX: ");Serial.print(VRX);
        Serial.print("    VRY: ");Serial.print(VRY);
        Serial.print("    PTM: ");Serial.print(PTM);
        Serial.print("    SW_1: ");Serial.print(SW_1);
        Serial.print("    SW_2: ");Serial.println(SW_2);
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
        Serial.print("    Weapon: ");
        Serial.print(weapon_pos);
        Serial.print("    Weapon Mode: ");
        if(weapon_mode == AUTONOMOUS)Serial.println("AUTO");
        else Serial.println("MANUAL");
//********************************* motor ********************************
//        if(back_dir == CLOCKWISE) Serial.println("CLOCKWISE");
//        else if(back_dir == COUNTERCLOCKWISE) Serial.println("COUNTERCLOCKWISE");
//        else Serial.println("STANDBY");
//        
//        Serial.print("back left PWM: ");
//        Serial.print(back_left_speed);
//        Serial.print("    back right PWM: ");
//        Serial.println(back_right_speed);
//******************************** encoder *******************************
//        Serial.print("left period: ");
//        Serial.print(left_period);
//        Serial.print("   right period: ");
//        Serial.println(right_period);
        Serial.print("left speed: ");
        Serial.print(left_rps);
        Serial.print(" rps    right speed: ");
        Serial.print(right_rps);
        Serial.print(" rps    difference: ");
        Serial.print(diff_rps);
        Serial.println(" rps"); 
//********************************* vive *********************************
        if(is_x_found && is_y_found){
            Serial.print("x = ");
            Serial.print(x_coor);
            Serial.print("    y = ");
            Serial.println(y_coor);
        } 
    }
}
//========================================================================
//=========================== data print end =============================
//========================================================================
