//========================================================================
//======================= MEAM 510 Final Main Board ======================
//========================================================================
//*****************************Team 04: BAT_V*****************************
//**************Members: Jiatong Sun, Xinyue Wei, Haojiong Lu*************
//========================================================================
//======================= MEAM 510 Final Main Board ======================
//========================================================================





//========================================================================
//============================ library start =============================
//========================================================================
#include <WiFi.h>
#include <WiFiUDP.h>
//========================================================================
//============================= library end ==============================
//========================================================================



// ===== GAME VARIABLES =====
// change ROBOTNUM (1-4) and TEAMCOLOR (BLUE or RED) as necessary
#define RED             0xFF0000    // color for the red team
#define BLUE            0x0000FF    // color for the blue team
#define ROBOTNUM    4               // robot number on meta team (1-4)
#define TEAMCOLOR   BLUE            // color for the robot team, either RED or BLUE
#define MAX_HP                   5
#define MAX_RESPAWN_TIME         15






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

#define I2C_MASTER_SCL_IO (gpio_num_t)4             // gpio number for I2C master clock
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
//        Serial.printf("%02x ", buf[i]);
        if ((i + 1) % 16 == 0) 
        {
//            Serial.printf("\n");
        }
    }
//    Serial.printf("\n");
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
//        Serial.println("I2C Timeout");
    } 
    else if (ret == ESP_OK) 
    {
        // uncomment the following 2 lines if you want to display information read from I2C
//        Serial.printf(" MASTER READ FROM SLAVE ******\n");
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
//        Serial.printf(" MASTER WRITE TO SLAVE\n");
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

#define HEALTHCOLOR     0x00FF00    // color for the health LEDs   

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

// ==========================

#define NEO_LED_PIN 17              // pin attached to LED ring
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
//    Serial.print("FastLED: Main code running on core ");
//    Serial.println(core);

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
    int num_on_leds = (NUM_LEDS - 4) * health / MAX_HP;
    int healthleds[] = {1,2,3,4,5,7,8,9,10,11,13,14,15,16,17,19,20,21,22,23};
    for(int i = 0; i < num_on_leds; i++) leds[healthleds[i]] = HEALTHCOLOR;
    for(int i = num_on_leds; i < (NUM_LEDS - 4); i++) leds[healthleds[i]] = 0;
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
//====================== constant definition start =======================
//=============================== ========================================
#define    CLOCKWISE                1  
#define    COUNTERCLOCKWISE         -1  
#define    STANDBY                  0
#define    AUTONOMOUS               1
#define    MANUAL                   0
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
//******************************** connect *******************************
#define    CONNECT_PIN             21
//********************************* servo ********************************
#define    ORIENT_SERVO_PIN        18
#define    WEAPON_SERVO_PIN        22
//********************************* motor ********************************
#define    BACK_LEFT_EN_PIN        25
#define    BACK_RIGHT_EN_PIN       26
#define    BACK_DIR_PIN            32
#define    BACK_IDIR_PIN           33
#define    FRONT_DIR_PIN           13
#define    FRONT_EN_PIN            27
//********************************* vive *********************************
#define    VIVE_X_1                36
#define    VIVE_Y_1                34
//********************************** LED *********************************
#define    LED_PIN                 14
//========================================================================
//========================== pin definition end ==========================
//========================================================================





//========================================================================
//================== global variables definition start ===================
//========================================================================
//********************************* mode *********************************
bool cur_mode = MANUAL, last_mode = MANUAL;
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
//********************************* vive *********************************
int x_coor_1 = 0, y_coor_1 = 0;
int x_coor_2 = 0, y_coor_2 = 0;
//========================================================================
//=================== global variables definition end ====================
//========================================================================





//========================================================================
//=========================== pin setup start ============================
//========================================================================
void pinSetup(){
//********************************* motor ********************************
    pinMode(BACK_DIR_PIN,OUTPUT);
    pinMode(BACK_IDIR_PIN,OUTPUT);
    pinMode(FRONT_DIR_PIN,OUTPUT);
//******************************** encoder *******************************
    pinMode(VIVE_X_1,INPUT);
    pinMode(VIVE_Y_1,INPUT);
//******************************* connection *****************************
    pinMode(CONNECT_PIN, OUTPUT); 
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
//============================ preset start ==============================
//========================================================================
void preset(){
    digitalWrite(BACK_DIR_PIN,LOW);
    digitalWrite(BACK_IDIR_PIN,LOW);
    digitalWrite(FRONT_DIR_PIN,LOW);
}
//========================================================================
//============================= preset end ===============================
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
        
        if(SW_2 == 1)front_standby = false;
        else front_standby = true;
    }
}
//========================================================================
//============================ UDP receive end ===========================
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
            int max_pos = map(PTM, 1, 255, (WEAPON_MIN_ANGLE + (WEAPON_MAX_ANGLE - WEAPON_MIN_ANGLE) / 3), WEAPON_MAX_ANGLE);
            int front_angle = WEAPON_MAX_ANGLE - (WEAPON_MAX_ANGLE - WEAPON_MIN_ANGLE) / 3;
            int angle_max = front_angle + abs(max_pos - front_angle);
            int angle_min = front_angle - abs(max_pos - front_angle);

            if(weapon_pos < angle_min || weapon_pos > angle_max){
                weapon_pos = front_angle;
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
//============================= vive start ===============================
//========================================================================
void viveReceive(){
    x_coor_1 = analogRead(VIVE_X_1);
    y_coor_1 = analogRead(VIVE_Y_1);
}
//========================================================================
//============================== vive end ================================
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
//    ledcWrite(BACK_LEFT_CHANNEL,back_left_speed * 3 / 4);
//    ledcWrite(BACK_RIGHT_CHANNEL,back_right_speed * 3 / 4);
    ledcWrite(BACK_LEFT_CHANNEL, back_left_speed);
    ledcWrite(BACK_RIGHT_CHANNEL, back_left_speed);  
}
//========================================================================
//======================= back motor control end =========================
//========================================================================





//========================================================================
//========================== data print start ============================
//========================================================================
void show(){
    if(millis() % 500 == 0){ // print every half second
//********************************* mode *********************************     
        Serial.print("Mode: ");
        if(cur_mode)Serial.println("AUTONOMOUS");
        else Serial.println("MANUAL");
//********************************* WiFi *********************************
        Serial.print("VRX: ");Serial.print(VRX);
        Serial.print("    VRY: ");Serial.print(VRY);
        Serial.print("    PTM: ");Serial.print(PTM);
        Serial.print("    SW_1: ");Serial.print(SW_1);
        Serial.print("    SW_2: ");Serial.println(SW_2);
//********************************* servo ********************************
        Serial.print("Orient: ");
        Serial.print(orient_pos);
        Serial.print("    Weapon: ");
        Serial.println(weapon_pos);
        if(weapon_mode == AUTONOMOUS)Serial.println("weapon auto");
        else Serial.println("weapon manual");
//********************************* motor ********************************
        if(back_dir == CLOCKWISE) Serial.println("CLOCKWISE");
        else if(back_dir == COUNTERCLOCKWISE) Serial.println("COUNTERCLOCKWISE");
        else Serial.println("STANDBY");
        
        Serial.print("back left PWM: ");
        Serial.print(back_left_speed);
        Serial.print("    back right PWM: ");
        Serial.println(back_right_speed);
//********************************** vive ********************************
        Serial.print("vive 1: ");
        Serial.print(x_coor_1);
        Serial.print("    ");
        Serial.println(y_coor_1);
        Serial.print("vive 2: ");
        Serial.print(x_coor_2);
        Serial.print("    ");
        Serial.println(y_coor_2);
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
//    Serial.begin(115200);
//    
//    // ========================= I2C start =============================
//    ESP_ERROR_CHECK(i2c_master_init()); // initialize the i2c
//    // ========================== I2C end ==============================
//
//    // ===================== Interrupts start ==========================
//    // default clock speed is 240MHz
//    // 240MHz / 240 = 1MHz      1 000 000 timer increments per second
//    // 1 000 000 / 20 = 50 000  timer value to count up to before calling interrupt (call 20 times per second)
//    timer = timerBegin(0, 240, true);                       // initialize timer with pre-scaler of 240
//    timerAttachInterrupt(timer, &readI2COnTimer, true);     // attach function to timer interrupt
//    timerAlarmWrite(timer, 50000, true);                    // set count value before interrupt occurs
//    timerAlarmEnable(timer);                                // start the timer
//    // ====================== Interrupts end ===========================
//
//    // ========================= LED start =============================
//    SetupFastLED(); // set the LEDs
//    // ========================== LED end ==============================

    STA_UDP_Set();
    timerSetup();
    pinSetup();
    PWMSetup();
}
// =====================================================================
// ========================== END OF SETUP =============================
// =====================================================================



int test_mode = 0;
long test_time = 0;



// =====================================================================
// ============================== LOOP =================================
// =====================================================================
void loop()
{
//    // ========================= I2C start =============================
//    // static variables
//    // static variables only get initialized once, when the loop function is first called
//    // values of static variables persist through function calls
//    static bool gameStatus = 0;     // game on: 1, game off: 0
//    static bool reset = 0;          // 1 for resetting
//    static bool autoMode = 0;       // not autonomous mode: 0, is auto mode: 1
//    static bool syncStatus = 0;     // 1 for sync
//
//    static int health;              // robot's health
//
//    static int respawnTimer;        // amount of time remaining on respawn
//
//    if (readI2C)
//    {
//        readI2C = 0;                // set readI2C to be false
//        i2c_write_test();           // need this write for some reason in order to get read to work?
//        i2c_read_test();            // read information from slave (top hat)  
//
//        // read information
//        gameStatus  = 1 & (data_rd[0] >> 0);
//        reset       = 1 & (data_rd[0] >> 1);
//        autoMode    = 1 & (data_rd[0] >> 2);
//        syncStatus  = 1 & (data_rd[0] >> 3);
//    
//        if (data_rd[1] != 0xFF)
//        {
//            // make sure the data isn't 0xFF (0xFF means that something is wrong)
//            health = data_rd[1];
//        }
//
//        if (data_rd[2] != 0xFF)
//        {
//            // make sure the data isn't 0xFF (0xFF means that something is wrong)
//            respawnTimer = data_rd[2];
//        }
//    }
//    // ========================== I2C end ==============================
//
//
//    // ========================= LED start =============================
//    ShowRobotNum();         // set the LEDs for the robot number
//    ShowHealth(health);     // set the LEDs for the health
//    
//    if (health == 0)
//    {
//        clearLEDs();
//        ShowRespawnTimer(respawnTimer);
//    }
//
//    FastLEDshowESP32();
//    // ========================== LED end ==============================
//
//    // ===================== own function start ========================
    preset();
//    if(!gameStatus) return;
//    if(health == 0) return;
//    cur_mode = autoMode;

    WiFi_Reconnect();
    UDPreceiveData();

    cur_mode = front_standby;

    if(cur_mode == AUTONOMOUS){
//        digitalWrite(CONNECT_PIN,HIGH);
//        viveReceive();
          if(last_mode != cur_mode) test_time = millis();
          if(millis()-test_time>2000){
              test_time = millis();
              test_mode ++;
          }
          switch (test_mode){
              case 0: case 2: case 4: 
                  forwards();
              case 1: turnRight();
              case 3: turnLeft();
              default: return; 
          }
    }
    else{
//        digitalWrite(CONNECT_PIN,LOW);
        orientServoControl();
        weaponServoControl();
        backMotorControl();
    }
    last_mode = cur_mode;
    show();
    
//    // ====================== own function end =========================
}
// =====================================================================
// ========================== END OF LOOP ==============================
// =====================================================================










//********************************* vive ********************************
#define    AIM_X                    3500
#define    AIM_Y                    5000
#define    X_ADJUST_THRESH          200
#define    Y_ADJUST_THRESH          200
#define    X_APPROACH_THRESH        500
#define    Y_APPROACH_THRESH        500
#define    BACK_TIME                300000
#define    TURN_TIME                300000




int pos_x_error = 0, pos_y_error = 0;
int dx = 0, dy = 0; 




//========================================================================
//=========================== turn left start ============================
//========================================================================
void turnLeft(){    
    ledcWrite(ORIENT_CHANNEL,ORIENT_MAX_ANGLE);
    digitalWrite(BACK_DIR_PIN,HIGH);
    digitalWrite(BACK_IDIR_PIN,LOW); 
    ledcWrite(BACK_LEFT_CHANNEL,50);
    ledcWrite(BACK_RIGHT_CHANNEL,50);
}
//========================================================================
//============================ turn left end =============================
//========================================================================





//========================================================================
//========================== turn right start ============================
//========================================================================
void turnRight(){    
    ledcWrite(ORIENT_CHANNEL,ORIENT_MIN_ANGLE);
    digitalWrite(BACK_DIR_PIN,HIGH);
    digitalWrite(BACK_IDIR_PIN,LOW); 
    ledcWrite(BACK_LEFT_CHANNEL,50);
    ledcWrite(BACK_RIGHT_CHANNEL,50);
}
//========================================================================
//=========================== turn right end =============================
//========================================================================




//========================================================================
//============================ forwards start ============================
//========================================================================
void forwards(){    
    ledcWrite(ORIENT_CHANNEL,ORIENT_MIN_ANGLE + (ORIENT_MAX_ANGLE - ORIENT_MIN_ANGLE) * 2 / 3);
    digitalWrite(BACK_DIR_PIN,HIGH);
    digitalWrite(BACK_IDIR_PIN,LOW); 
    ledcWrite(BACK_LEFT_CHANNEL,60);
    ledcWrite(BACK_RIGHT_CHANNEL,60);
}
//========================================================================
//============================= forwards end =============================
//========================================================================





//========================================================================
//=========================== backwards start ============================
//========================================================================
void backwards(){    
    ledcWrite(ORIENT_CHANNEL,ORIENT_MIN_ANGLE + (ORIENT_MAX_ANGLE - ORIENT_MIN_ANGLE) * 2 / 3);
    digitalWrite(BACK_DIR_PIN,LOW);
    digitalWrite(BACK_IDIR_PIN,HIGH); 
    ledcWrite(BACK_LEFT_CHANNEL,70);
    ledcWrite(BACK_RIGHT_CHANNEL,70);
}
//========================================================================
//============================ backwards end =============================
//========================================================================




//========================================================================
//======================= adjust X position start ========================
//========================================================================
void adjustXPosition(){
    if(((pos_x_error<0)&&(dx>0)&&(dy<0)) || ((pos_x_error>0)&&(dx<0)&&(dy>0))){
      turnRight();
      forwards();
    }else if(((pos_x_error<0)&&(dx>0)&&(dy>0)) || ((pos_x_error>0)&&(dx<0)&&(dy<0))){
      turnLeft();
      forwards();
    }else if(((pos_x_error<0)&&(dx<0)&&(dy<0)) || ((pos_x_error>0)&&(dx>0)&&(dy>0))){
      if(abs(dx) > X_ADJUST_THRESH){
        turnLeft();
        backwards();
      }else{
        turnRight();
        forwards();
      }
    }else if(((pos_x_error<0)&&(dx<0)&&(dy>0)) || ((pos_x_error>0)&&(dx>0)&&(dy<0))){
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
    if(((pos_y_error<0)&&(dy>0)&&(dx<0)) || ((pos_y_error>0)&&(dy<0)&&(dx>0))){
      turnRight();
      forwards();
    }else if(((pos_y_error<0)&&(dy>0)&&(dx>0)) || ((pos_y_error>0)&&(dy<0)&&(dx<0))){
      turnLeft();
      forwards();
    }else if(((pos_y_error<0)&&(dy<0)&&(dx<0)) || ((pos_y_error>0)&&(dy>0)&&(dx>0))){
      if(abs(dy) > Y_ADJUST_THRESH){
        turnLeft();
        backwards();
      }else{
        turnRight();
        forwards();
      }
    }else if(((pos_y_error<0)&&(dy<0)&&(dx>0)) || ((pos_y_error>0)&&(dy>0)&&(dx<0))){
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
    if((abs(pos_x_error) > X_APPROACH_THRESH) || (abs(pos_y_error) > Y_APPROACH_THRESH) ) {
      adjustXPosition();
    }else if((abs(dx) < X_ADJUST_THRESH) && abs(pos_x_error)> X_APPROACH_THRESH){
      forwards();
    }else if((abs(dy) > Y_ADJUST_THRESH)){
      if(((pos_y_error) * dx)< 0) turnLeft();
      else if (((pos_y_error) * dx)> 0) turnRight();
    }else if((abs(dy) < Y_ADJUST_THRESH) && abs(pos_y_error)> Y_APPROACH_THRESH){
      forwards();
    }
}
//========================================================================
//========================== to destination end==========================
//========================================================================
