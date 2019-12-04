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
#define    FRONT_CHANNEL            4
#define    MOTOR_ZERO_SPEED         119
//******************************** encoder *******************************
#define    PERIOD_THRESH            10000
#define    ERR_NUM_THRESH           180
#define    PAUSE_THRESH             20000
//========================================================================
//======================= constant definition end ========================
//========================================================================




//========================= pin definition start =========================
//========================================================================
//****************************** ultrosonic ******************************
#define    TRIGGER_PIN             19
#define    ECHO_PIN                18
//********************************* servo ********************************
#define    ORIENT_SERVO_PIN        23
#define    WEAPON_SERVO_PIN        22
//********************************* motor ********************************
#define    BACK_LEFT_EN_PIN        25
#define    BACK_RIGHT_EN_PIN       26
#define    BACK_DIR_PIN            32
#define    BACK_IDIR_PIN           33
#define    FRONT_DIR_PIN           13
#define    FRONT_EN_PIN            27
//******************************** encoder *******************************
#define    LEFT_ENCODER_PIN        36
#define    RIGHT_ENCODER_PIN       39
//********************************* vive *********************************
#define    VIVE_PIN                21
//********************************** LED *********************************
#define    LED_PIN                 14
//========================================================================
//========================== pin definition end ==========================
//========================================================================





//========================================================================
//================== global variables definition start ===================
//========================================================================
//********************************* mode *********************************
bool cur_mode = MANUAL;
//********************************* timer ********************************
hw_timer_t* timer_1 = NULL;
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





// =================================================================
// ========================= I2C start =============================
// =================================================================
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 128                             // data buffer length of test buffer
#define W_LENGTH 1                                  // data length for w, [0,DATA_LENGTH]
#define R_LENGTH 16                                 // data length for r, [0,DATA_LENGTH]

#define I2C_MASTER_SCL_IO (gpio_num_t)17            // gpio number for I2C master clock
#define I2C_MASTER_SDA_IO (gpio_num_t)16            // gpio number for I2C master data
#define I2C_MASTER_NUM I2C_NUMBER(1)                // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ 40000                    // I2C master clock frequency (Hz)
#define I2C_MASTER_TX_BUF_DISABLE 0                 // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE 0                 // I2C master doesn't need buffer

#define CONFIG_I2C_SLAVE_ADDRESS 0x28
#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS     // ESP32 slave address, you can set any 7bit value
#define WRITE_BIT I2C_MASTER_WRITE                  // I2C master write
#define READ_BIT I2C_MASTER_READ                    // I2C master read
#define ACK_CHECK_EN 0x1                            // I2C master will check ack from slave
#define ACK_CHECK_DIS 0x0                           // I2C master will not check ack from slave
#define ACK_VAL I2C_MASTER_ACK                      // I2C ack value
#define NACK_VAL I2C_MASTER_NACK                    // I2C nack value

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */
static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t nsize)
{
    if (nsize == 0) 
    {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN); 
    if (nsize > 1) 
    {
        i2c_master_read(cmd, data_rd, nsize - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + nsize - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS); // send all queued commands
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief Test code to write esp-i2c-slave
 *        Master device write data to slave(both esp32),
 *        the data will be stored in slave buffer.
 *        We can read them out from slave buffer.
 *
 * ___________________________________________________________________
 * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
 * --------|---------------------------|----------------------|------|
 *
 */
static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t nsize)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, nsize, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init()
{
    i2c_port_t i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    return i2c_driver_install(i2c_master_port, conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief test function to show buffer
 */
static void disp_buf(uint8_t *buf, int len)
{
    int i;
    for (i = 0; i < len; i++) 
    {
        Serial.printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) 
        {
            Serial.printf("\n");
        }
    }
    Serial.printf("\n");
}

uint8_t data_wr[DATA_LENGTH];
uint8_t data_rd[DATA_LENGTH];

static void i2c_read_test()
{
    int ret;

    ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, DATA_LENGTH);

    if (ret == ESP_ERR_TIMEOUT) 
    {
        ESP_LOGE(TAG, "I2C Timeout");
        Serial.println("I2C Timeout");
    } 
    else if (ret == ESP_OK) 
    {
        // uncomment the following 2 lines if you want to display information read from I2C
        Serial.printf(" MASTER READ FROM SLAVE ******\n");
        disp_buf(data_rd, DATA_LENGTH);
    } 
    else 
    {
        ESP_LOGW(TAG, " %s: Master read slave error, IO not connected...\n",
            esp_err_to_name(ret));
    }
}

static void i2c_write_test()
{ 
    int ret;
                                                                             
    ret = i2c_master_write_slave(I2C_MASTER_NUM, data_wr, W_LENGTH);
    if (ret == ESP_ERR_TIMEOUT) 
    {
        ESP_LOGE(TAG, "I2C Timeout");
    } 
    else if (ret == ESP_OK) 
    {
        // uncomment the following 2 lines if you want to display information being send over I2C
        Serial.printf(" MASTER WRITE TO SLAVE\n");
        disp_buf(data_wr, W_LENGTH);
    } 
    else 
    {
        ESP_LOGW(TAG, "%s: Master write slave error, IO not connected....\n",
            esp_err_to_name(ret));
    }
}
// =================================================================
// ========================== I2C end ==============================
// =================================================================


// =================================================================
// ====================== Interrupt start ==========================
// =================================================================
// Timer + Interrupt for reading I2C
hw_timer_t* timer = NULL;                               // initialize a timer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;   // needed to sync between main loop and ISR when modifying shared variable
volatile bool readI2C = 0;                              // should we read data from I2C?

void IRAM_ATTR readI2COnTimer()
{
    portENTER_CRITICAL_ISR(&timerMux);
    readI2C = 1;                        // need to read I2C next loop
    portEXIT_CRITICAL_ISR(&timerMux);
}
// =================================================================
// ======================= Interrupt end ===========================
// =================================================================


// =================================================================
// ========================= LED start =============================
// =================================================================
#include "FastLED.h"
FASTLED_USING_NAMESPACE

#define RED             0xFF0000    // color for the red team
#define BLUE            0x0000FF    // color for the blue team
#define HEALTHCOLOR     0x00FF00    // color for the health LEDs   

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

// ===== GAME VARIABLES =====
// change ROBOTNUM (1-4) and TEAMCOLOR (BLUE or RED) as necessary
#define ROBOTNUM    4               // robot number on meta team (1-4)
#define TEAMCOLOR   BLUE            // color for the robot team, either RED or BLUE
// ==========================

#define NEO_LED_PIN 4              // pin attached to LED ring
#define LED_TYPE    WS2812          // APA102
#define COLOR_ORDER GRB             // changes the order so we can use standard RGB for the values
#define NUM_LEDS    24              // number of LEDs in the ring
CRGB leds[NUM_LEDS];                // set value of LED, each LED is 24 bits

#define BRIGHTNESS          60      // lower the brightness a bit

// core to run FastLED.show()
#define FASTLED_SHOW_CORE 0

// task handles for use in the notifications
static TaskHandle_t FastLEDshowTaskHandle = 0;
static TaskHandle_t userTaskHandle = 0;

/** show() for ESP32
 *  Call this function instead of FastLED.show(). It signals core 0 to issue a show, 
 *  then waits for a notification that it is done.
 */
void FastLEDshowESP32()
{
    if (userTaskHandle == 0) 
    {
        // -- Store the handle of the current task, so that the show task can
        //    notify it when it's done
        userTaskHandle = xTaskGetCurrentTaskHandle();

        // -- Trigger the show task
        xTaskNotifyGive(FastLEDshowTaskHandle);

        // -- Wait to be notified that it's done
        const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 200 );
        ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
        userTaskHandle = 0;
    }
}

/** show Task
 *  This function runs on core 0 and just waits for requests to call FastLED.show()
 */
void FastLEDshowTask(void *pvParameters)
{
    // -- Run forever...
    for(;;) 
    {
        // -- Wait for the trigger
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // -- Do the show (synchronously)
        FastLED.show();

        // -- Notify the calling task
        xTaskNotifyGive(userTaskHandle);
    }
}

void SetupFastLED(void)
{
    // tell FastLED about the LED strip configuration
    FastLED.addLeds<LED_TYPE,NEO_LED_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);

    // set master brightness control
    FastLED.setBrightness(BRIGHTNESS);

    int core = xPortGetCoreID();
    Serial.print("FastLED: Main code running on core ");
    Serial.println(core);

    // -- Create the FastLED show task
    xTaskCreatePinnedToCore(FastLEDshowTask, "FastLEDshowTask", 2048, NULL, 2, &FastLEDshowTaskHandle, FASTLED_SHOW_CORE);
}

void ShowRobotNum(void)
{
    int robotLeds[] = {0,6,12,18};      // location of the LEDs used to display the robot number

    // change the LEDs based on the robot number
    leds[robotLeds[0]] = TEAMCOLOR;     // The first LED is always displayed with the robot number

    switch (ROBOTNUM)
    {
        case 1:
            leds[robotLeds[1]] = 0;
            leds[robotLeds[2]] = 0;
            leds[robotLeds[3]] = 0;
            break;
        case 2:
            leds[robotLeds[1]] = 0;
            leds[robotLeds[2]] = TEAMCOLOR;
            leds[robotLeds[3]] = 0;
            break;
        case 3:
            leds[robotLeds[1]] = TEAMCOLOR;
            leds[robotLeds[2]] = 0;
            leds[robotLeds[3]] = TEAMCOLOR;
            break;
        case 4:
            leds[robotLeds[1]] = TEAMCOLOR;
            leds[robotLeds[2]] = TEAMCOLOR;
            leds[robotLeds[3]] = TEAMCOLOR;
            break;
    }
}

void ShowHealth(int health)
{
    if(!health) return;
    int healthleds[] = {1,2,3,4,5,7,8,9,10,11,13,14,15,16,17,19,20,21,22,23};
    for(int i = 0; i < health; i++) leds[healthleds[i]] = HEALTHCOLOR;
    for(int i = health; i < (NUM_LEDS - 4);i++) leds[healthleds[i]] = 0;
}

void clearLEDs(void)
{
    for(int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = 0;
    }
}

void ShowRespawnTimer(int respawnTime)
{
    int num_on_leds = NUM_LEDS * respawnTime/ 15; 
    for(int i = 0; i < num_on_leds; i++) leds[i] = RED;
    for(int i = num_on_leds; i < NUM_LEDS - num_on_leds;i++) leds[i] = 0;
}
// =================================================================
// ========================== LED end ==============================
// =================================================================





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
    ledcSetup(FRONT_CHANNEL,MOTOR_FREQ_HZ,MOTOR_RESOLUTION_BITS);
    ledcAttachPin(BACK_LEFT_EN_PIN,BACK_LEFT_CHANNEL); 
    ledcAttachPin(BACK_RIGHT_EN_PIN,BACK_RIGHT_CHANNEL);
    ledcAttachPin(FRONT_EN_PIN,FRONT_CHANNEL);
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
    timer_1 = timerBegin(1, 80, true);
    timerAttachInterrupt(timer_1, &onTimer, true);
    timerAlarmWrite(timer_1, 1000, true); 
    timerAlarmEnable(timer_1);       
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
        digitalWrite(BACK_DIR_PIN,LOW);
        digitalWrite(BACK_IDIR_PIN,LOW);
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
    if(is_pid){
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
}
//========================================================================
//========================== PID control end =============================
//========================================================================





//========================================================================
//===================== front motor control start ========================
//========================================================================
void frontMotorControl(){
    if(left_rps < 0.5 || right_rps < 0.5 || back_dir == COUNTERCLOCKWISE || front_standby){
        digitalWrite(FRONT_DIR_PIN, LOW);
        ledcWrite(FRONT_CHANNEL,0); 
    } else {
        digitalWrite(FRONT_DIR_PIN, HIGH);
        ledcWrite(FRONT_CHANNEL,200);
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
        if(back_dir == CLOCKWISE) Serial.println("CLOCKWISE");
        else if(back_dir == COUNTERCLOCKWISE) Serial.println("COUNTERCLOCKWISE");
        else Serial.println("STANDBY");
        
        Serial.print("back left PWM: ");
        Serial.print(back_left_speed);
        Serial.print("    back right PWM: ");
        Serial.println(back_right_speed);
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






// =====================================================================
// ============================= SETUP =================================
// =====================================================================
void setup()
{
    Serial.begin(115200);
    
    // ========================= I2C start =============================
    ESP_ERROR_CHECK(i2c_master_init()); // initialize the i2c
    // ========================== I2C end ==============================

    // ===================== Interrupts start ==========================
    // default clock speed is 240MHz
    // 240MHz / 240 = 1MHz      1 000 000 timer increments per second
    // 1 000 000 / 20 = 50 000  timer value to count up to before calling interrupt (call 20 times per second)
    timer = timerBegin(0, 240, true);                       // initialize timer with pre-scaler of 240
    timerAttachInterrupt(timer, &readI2COnTimer, true);     // attach function to timer interrupt
    timerAlarmWrite(timer, 50000, true);                    // set count value before interrupt occurs
    timerAlarmEnable(timer);                                // start the timer
    // ====================== Interrupts end ===========================

    // ========================= LED start =============================
    SetupFastLED(); // set the LEDs
    // ========================== LED end ==============================

    STA_UDP_Set();
    timerSetup();
    pinSetup();
    PWMSetup();
}
// =====================================================================
// ========================== END OF SETUP =============================
// =====================================================================


// =====================================================================
// ============================== LOOP =================================
// =====================================================================
void loop()
{
    // ========================= I2C start =============================
    // static variables
    // static variables only get initialized once, when the loop function is first called
    // values of static variables persist through function calls
    static bool gameStatus = 0;     // game on: 1, game off: 0
    static bool reset = 0;          // 1 for resetting
    static bool autoMode = 0;       // not autonomous mode: 0, is auto mode: 1
    static bool syncStatus = 0;     // 1 for sync

    static int health;              // robot's health

    static int respawnTimer;        // amount of time remaining on respawn

    if (readI2C)
    {
        readI2C = 0;                // set readI2C to be false
        i2c_write_test();           // need this write for some reason in order to get read to work?
        i2c_read_test();            // read information from slave (top hat)  

        // read information
        gameStatus  = 1 & (data_rd[0] >> 0);
        reset       = 1 & (data_rd[0] >> 1);
        autoMode    = 1 & (data_rd[0] >> 2);
        syncStatus  = 1 & (data_rd[0] >> 3);
    
        if (data_rd[1] != 0xFF)
        {
            // make sure the data isn't 0xFF (0xFF means that something is wrong)
            health = data_rd[1];
        }

        if (data_rd[2] != 0xFF)
        {
            // make sure the data isn't 0xFF (0xFF means that something is wrong)
            respawnTimer = data_rd[2];
        }
    }
    // ========================== I2C end ==============================


    // ========================= LED start =============================
    ShowRobotNum();         // set the LEDs for the robot number
    ShowHealth(health);     // set the LEDs for the health
    
    if (health == 0)
    {
        clearLEDs();
        ShowRespawnTimer(respawnTimer);
    }

    FastLEDshowESP32();
    // ========================== LED end ==============================
    
    WiFi_Reconnect();
    UDPreceiveData();
    
    orientServoControl();
    weaponServoControl();

    pidControl();
    encoderCalc();
    viveReceive(); 

    backMotorControl();
    frontMotorControl();  
    
//    ultraDectect();
    show();
}
// =====================================================================
// ========================== END OF LOOP ==============================
// =====================================================================
