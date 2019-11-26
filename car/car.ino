//========================================================================
//============================ MEAM 510 Final ============================
//========================================================================
//Author: Jiatong Sun, Xinyue Wei, Haojiong Lu
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
//========================================================================
#define    CLOCKWISE               1  
#define    COUNTERCLOCKWISE        -1  
#define    LEDC_CHANNEL            0
#define    LEDC_RESOLUTION_BITS    13
#define    SERVO_LEDC_FREQ_HZ      50 
#define    MOTOR_LEDC_FREQ_HZ      1000    
#define    ORIENT_MIN_ANGLE        900
#define    ORIENT_MAX_ANGLE        1000
#define    WEAPON_MIN_ANGLE        200
#define    WEAPON_MAX_ANGLE        1000
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
#define    BACK_LEFT_DIR_PIN       26
#define    BACK_RIGHT_EN_PIN       27
#define    BACK_RIGHT_DIR_PIN      14
#define    BACK_DRIVER_STANDBY     21
#define    FRONT_WHEEL_PIN         19
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
//********************************* WiFi *********************************
const char* ssid = "BAT_V", * pass = "BATATABANANA";  
IPAddress IPlocal(192,168,1,179), IPtarget(192,168,1,171);  
WiFiUDP udp;
unsigned int UDPlocalPort = 2816, UDPtargetPort = 2918; 
const int packetSize = 100; 
byte receive_buffer[packetSize];
//****************************** ultrosonic ******************************
long distance = 0;
long duration = 0;
//********************************* servo ********************************
int orient_pos = ORIENT_MIN_ANGLE;
int orient_dir = CLOCKWISE;
int weapon_pos = WEAPON_MIN_ANGLE;
int weapon_dir = CLOCKWISE;
bool weapon_auto = false;
//********************************* motor ********************************
int back_left_speed = 0, back_right_speed = 0;
bool back_left_dir = true, back_right_dir = true;
bool front_standby = true, back_standby = true;
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
    pinSetup();
    PWMSetup();
}  

void loop(){
    WiFi_Reconnect();
    UDPreceiveData();
//    ultraDectect();
    show();
//    orientServoControl();
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
    pinMode(BACK_LEFT_DIR_PIN,OUTPUT);
    pinMode(BACK_RIGHT_DIR_PIN,OUTPUT);
    pinMode(BACK_DRIVER_STANDBY,OUTPUT);
    pinMode(FRONT_WHEEL_PIN,OUTPUT);
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
    ledcSetup(LEDC_CHANNEL,SERVO_LEDC_FREQ_HZ,LEDC_RESOLUTION_BITS);
//********************************* servo ********************************
    ledcAttachPin(ORIENT_SERVO_PIN,LEDC_CHANNEL); 
    ledcAttachPin(WEAPON_SERVO_PIN,LEDC_CHANNEL); 
//********************************* motor ********************************
    ledcAttachPin(BACK_LEFT_EN_PIN,LEDC_CHANNEL); 
    ledcAttachPin(BACK_RIGHT_EN_PIN,LEDC_CHANNEL);
}
//========================================================================
//============================ PWM setup end =============================
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
        
        back_left_speed = receive_buffer[0];
        back_right_speed = receive_buffer[0];
        orient_pos = receive_buffer[1];
        weapon_pos = receive_buffer[2];
        
        if(receive_buffer[3]==1)front_standby = true;
        else front_standby = false;
        
        if(receive_buffer[4]==1)weapon_auto = true;
        else weapon_auto = false ;
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
    while(millis()%3==0){
        ledcWrite(LEDC_CHANNEL,orient_pos);
        orient_pos += orient_dir;
        if(orient_pos == ORIENT_MAX_ANGLE || orient_pos == ORIENT_MIN_ANGLE){
            orient_dir = -orient_dir;
        }
    }
}
//========================================================================
//====================== orient servo control end ========================
//========================================================================





//========================================================================
//===================== weapon servo control start =======================
//========================================================================
void weaponServoControl(){
    while(millis()%3==0){
        ledcWrite(LEDC_CHANNEL,weapon_pos);
        weapon_pos += weapon_dir;
        if(weapon_pos == WEAPON_MAX_ANGLE || weapon_pos == WEAPON_MIN_ANGLE){
            weapon_dir = -weapon_dir;
        }
    }
}
//========================================================================
//====================== weapon servo control end ========================
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
        Serial.print("VRX: ");Serial.print(back_left_speed);
        Serial.print("    VRY: ");Serial.print(orient_pos);
        Serial.print("    PTM: ");Serial.print(weapon_pos);
        Serial.print("    SW_1: ");Serial.print(front_standby);
        Serial.print("    SW_2: ");Serial.println(weapon_auto);
//****************************** ultrosonic ******************************
//        Serial.print("Distance: ");
//        if(distance>=400 || distance<=2){
//            Serial.println("Out of range!");
//        } else {
//            Serial.print(distance);
//            Serial.println(" cm");
//        }
//********************************* servo ********************************
//        Serial.print("Orient: ");
//        Serial.print(orient_pos);
//        Serial.print("    Weapon: ");
//        Serial.println(weapon_pos);
//********************************* vive *********************************
        if(is_x_found && is_y_found){
//            Serial.print("x = ");
//            Serial.print(x_coor);
//            Serial.print("    y = ");
//            Serial.println(y_coor);
        } 
    }
}
//========================================================================
//=========================== data print end =============================
//========================================================================
