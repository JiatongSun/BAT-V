//=============================================================================================================================
//================================================= MEAM 510 Final Main Board =================================================
//=============================================================================================================================
//******************************************************* Team 04: BAT_V ******************************************************
//*************************************** Members: Jiatong Sun, Xinyue Wei, Haojiong Lu ***************************************
//=============================================================================================================================
//================================================= MEAM 510 Final Main Board =================================================
//=============================================================================================================================





//=============================================================================================================================
//======================================================= library start =======================================================
//=============================================================================================================================
#include <WiFi.h>
#include <WiFiUDP.h>
//=============================================================================================================================
//======================================================== library end ========================================================
//=============================================================================================================================





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

#define RED             0xFF0000    // color for the red team
#define BLUE            0x0000FF    // color for the blue team
#define HEALTHCOLOR     0x00FF00    // color for the health LEDs   

//******************************** top hat *******************************
#define    MAX_HP                   5
#define    MAX_RESPAWN_TIME         15

#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif

// ===== GAME VARIABLES =====
// change ROBOTNUM (1-4) and TEAMCOLOR (BLUE or RED) as necessary
#define ROBOTNUM    4               // robot number on meta team (1-4)
#define TEAMCOLOR   RED            // color for the robot team, either RED or BLUE
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
    for(int i = 0; i < num_on_leds; i++) leds[i] = RED; //red LED number proportional to respawn time
    for(int i = num_on_leds; i < NUM_LEDS - num_on_leds;i++) leds[i] = 0;
}
// =================================================================
// ========================== LED end ==============================
// =================================================================















//=============================================================================================================================
//==================================================== own function start =====================================================
//=============================================================================================================================
//=============================================================================================================================
//================================================= constant definition start =================================================
//=============================================================================================================================
//********************************************************** general **********************************************************
#define    CLOCKWISE                1         // moving forwards
#define    COUNTERCLOCKWISE         -1        // moving backwards
#define    STANDBY                  0         // stop the car
#define    AUTONOMOUS               1         // autonomous control mode
#define    MANUAL                   0         // manual control mode
#define    OUT                      1         // out of the range
#define    IN                       0         // in the range
//*********************************************************** servo ***********************************************************
#define    SERVO_RESOLUTION_BITS    13        // servo ledc resolution bits
#define    SERVO_RESOLUTION         8191      // servo ledc resolution
#define    SERVO_FREQ_HZ            50        // servo ledc frequency
#define    ORIENT_CHANNEL           0         // orient servo ledc channel
#define    WEAPON_CHANNEL           1         // weapon servo ledc channel
#define    ORIENT_MIN_ANGLE         320       // orient servo minimal angle
#define    ORIENT_MAX_ANGLE         400       // orient servo maximal angle
#define    WEAPON_MIN_ANGLE         200       // weapon servo minimal angle
#define    WEAPON_MID_ANGLE         600       // weapon servo middle angle
#define    WEAPON_MAX_ANGLE         1000      // weapon servo maximal angle
//*********************************************************** motor ***********************************************************
#define    MOTOR_RESOLUTION_BITS    8         // motor ledc resolution bits
#define    MOTOR_RESOLUTION         255       // motor ledc resolution
#define    MOTOR_FREQ_HZ            1000      // motor ledc frequency
#define    BACK_LEFT_CHANNEL        2         // back left motor ledc channel
#define    BACK_RIGHT_CHANNEL       3         // back right motor ledc channel
#define    FRONT_CHANNEL            4         // front motor ledc channel
#define    MOTOR_ZERO_SPEED         119       // data received while not touching joystick
//=============================================================================================================================
//================================================== constant definition end ==================================================
//=============================================================================================================================





//=============================================================================================================================
//=================================================== pin definition start ====================================================
//=============================================================================================================================
//*********************************************************** servo ***********************************************************
#define    ORIENT_SERVO_PIN        18         // orient servo signal pin
#define    WEAPON_SERVO_PIN        22         // weapon servo signal pin
//*********************************************************** motor ***********************************************************
#define    BACK_LEFT_EN_PIN        25         // back left motor PWM pin
#define    BACK_RIGHT_EN_PIN       26         // back right motor PWM pin
#define    BACK_DIR_PIN            32         // back motors DIR pin
#define    BACK_IDIR_PIN           33         // back motors inverse DIR pin
#define    FRONT_EN_PIN            27         // front motors PWM pin
#define    FRONT_DIR_PIN           13         // front motors DIR pin
//*********************************************************** vive ************************************************************
#define    COMM_PIN_1              36         // communication pin 1
#define    COMM_PIN_2              34         // communication pin 2
//=============================================================================================================================
//==================================================== pin definition end =====================================================
//=============================================================================================================================





//=============================================================================================================================
//============================================= global variables definition start =============================================
//=============================================================================================================================
//*********************************************************** mode ************************************************************
bool cur_mode = MANUAL, last_mode = MANUAL;                   // mode
//*********************************************************** timer ***********************************************************
hw_timer_t* timer_1 = NULL;                                   // timer 1
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;              // portMUX
//*********************************************************** WiFi ************************************************************
const char* ssid = "BAT_V", * pass = "BATATABANANA";          // WiFi name and password
IPAddress IPlocal(192,168,1,179), IPtarget(192,168,1,171);    // WiFi address
WiFiUDP udp;                                                  // WiFi UDP
unsigned int UDPlocalPort = 2816, UDPtargetPort = 2918;       // WiFi port
const int packetSize = 100;                                   // packet size  
byte receive_buffer[packetSize];                              // array to store received buffer
int VRX = 0, VRY = 0, PTM = 0, SW_1 = 0, SW_2 = 0;            // five data received  
int last_VRX = 0, last_VRY = 0, last_PTM = 0, last_SW_2 = 0;  // data in last loop
//*********************************************************** servo ***********************************************************
bool is_new_orient = false;                                   // flag to indicate if the orient servo position changes
int orient_pos = ORIENT_MIN_ANGLE;                            // orient servo position
int orient_dir = CLOCKWISE;                                   // orient servo direction
bool is_new_weapon = false;                                   // flag to indicate if the orient servo position changes
int weapon_pos = WEAPON_MIN_ANGLE;                            // weapon position
int weapon_dir = CLOCKWISE;                                   // weapon direction
bool weapon_mode = MANUAL;                                    // weapon mode
bool is_weapon_move = false;                                  // flag to indicate if the interrupt for weapon happens
//*********************************************************** motor ***********************************************************
bool is_new_speed = false;                                    // flag to indicate if the back motor speed changes 
double back_left_speed = 0, back_right_speed = 0;             // back motors PWM
int back_dir = 0;                                             // back motors direction
int back_standby = STANDBY;                                   // back motors condition
int front_standby = STANDBY;                                  // front motors condition
//*********************************************************** vive ************************************************************
bool command_1 = false, command_2 = false;                    // commands received from the auxiliary board
//=============================================================================================================================
//============================================== global variables definition end ==============================================
//=============================================================================================================================





//=============================================================================================================================
//====================================================== pin setup start ======================================================
//=============================================================================================================================
void pinSetup(){
//*********************************************************** motor ***********************************************************
    pinMode(BACK_DIR_PIN,OUTPUT);
    pinMode(BACK_IDIR_PIN,OUTPUT);
    pinMode(FRONT_DIR_PIN,OUTPUT);
//******************************************************** connection *********************************************************
    pinMode(COMM_PIN_1, INPUT); 
    pinMode(COMM_PIN_2, INPUT);  
}
//=============================================================================================================================
//======================================================= pin setup end =======================================================
//=============================================================================================================================





//=============================================================================================================================
//====================================================== PWM setup start ======================================================
//=============================================================================================================================
void PWMSetup(){
//*********************************************************** servo ***********************************************************
    ledcSetup(ORIENT_CHANNEL,SERVO_FREQ_HZ,SERVO_RESOLUTION_BITS);
    ledcSetup(WEAPON_CHANNEL,SERVO_FREQ_HZ,SERVO_RESOLUTION_BITS);
    ledcAttachPin(ORIENT_SERVO_PIN,ORIENT_CHANNEL); 
    ledcAttachPin(WEAPON_SERVO_PIN,WEAPON_CHANNEL); 
//*********************************************************** motor ***********************************************************
    ledcSetup(BACK_LEFT_CHANNEL,MOTOR_FREQ_HZ,MOTOR_RESOLUTION_BITS);
    ledcSetup(BACK_RIGHT_CHANNEL,MOTOR_FREQ_HZ,MOTOR_RESOLUTION_BITS);
    ledcAttachPin(BACK_LEFT_EN_PIN,BACK_LEFT_CHANNEL); 
    ledcAttachPin(BACK_RIGHT_EN_PIN,BACK_RIGHT_CHANNEL);
}
//=============================================================================================================================
//======================================================= PWM setup end =======================================================
//=============================================================================================================================





//=============================================================================================================================
//====================================================== interrupt start ======================================================
//=============================================================================================================================
void IRAM_ATTR onTimer(){      
    portENTER_CRITICAL_ISR(&mux);
    is_weapon_move = true;      //whenever this interrupt happens, weapon is enabled to change position
    portEXIT_CRITICAL_ISR(&mux);
}
//=============================================================================================================================
//======================================================= interrupt end =======================================================
//=============================================================================================================================





//=============================================================================================================================
//===================================================== timer setup start =====================================================
//=============================================================================================================================
void timerSetup(){
    timer_1 = timerBegin(1, 80, true);              // 80MHz / 80 = 1MHz ~ 1us
    timerAttachInterrupt(timer_1, &onTimer, true);  
    timerAlarmWrite(timer_1, 500, true);            //interrupt every 500 us
    timerAlarmEnable(timer_1);       
}
//=============================================================================================================================
//===================================================== timer setup start =====================================================
//=============================================================================================================================





//=============================================================================================================================
//====================================================== UDP setup start ======================================================
//=============================================================================================================================
void STA_UDP_Set(){       //set UDP to STA mode
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
//=============================================================================================================================
//======================================================= UDP setup end =======================================================
//=============================================================================================================================





//=============================================================================================================================
//======================================================= preset start ========================================================
//=============================================================================================================================
void preset(){          // set every DIR and inverse DIR to LOW to stop the car
    digitalWrite(BACK_DIR_PIN,LOW);
    digitalWrite(BACK_IDIR_PIN,LOW);
    digitalWrite(FRONT_DIR_PIN,LOW);
}
//=============================================================================================================================
//======================================================== preset end =========================================================
//=============================================================================================================================





//=============================================================================================================================
//=================================================== WiFi reconnect start ====================================================
//=============================================================================================================================
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
    digitalWrite(LED_BUILTIN, LOW);   // use a builtin LED to indicate WiFi status
}
//=============================================================================================================================
//==================================================== WiFi reconnect end =====================================================
//=============================================================================================================================





//=============================================================================================================================
//===================================================== UDP receive start =====================================================
//=============================================================================================================================
void UDPreceiveData(){
    if (udp.parsePacket())
    {
        udp.read(receive_buffer, packetSize);

        // receive data from controller
        VRX = receive_buffer[0];
        VRY = receive_buffer[1];
        PTM = receive_buffer[2];
        SW_1 = receive_buffer[3];
        SW_2 = receive_buffer[4];

        // use flags to indicate whether PWM command has changed
        // ledcWrite() will only happen if PWM command changes
        if(VRX!=last_VRX)is_new_speed = true;     
        if(VRY!=last_VRY)is_new_orient = true;
        if(PTM!=last_PTM)is_new_weapon = true;

        // use the [1,255] data to decide back motors's speed(PWM) and direction
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

        // use the [1,255] data to decide orient servo's position
        orient_pos = map(VRY, 1, 255, ORIENT_MIN_ANGLE, ORIENT_MAX_ANGLE);

        // use the {1,2} data to decide weapon's mode
        if(SW_1 == 1)weapon_mode = AUTONOMOUS;
        else weapon_mode = MANUAL;

        // use the {1,2} data to decide front motors' mode
        if(SW_2 == 1)front_standby = STANDBY;
        else front_standby = CLOCKWISE;

        // save current loop's data
        last_VRX = VRX;
        last_VRY = VRY;
        last_PTM = PTM;
    }
}
//=============================================================================================================================
//====================================================== UDP receive end ======================================================
//=============================================================================================================================





//=============================================================================================================================
//================================================ orient servo control start =================================================
//=============================================================================================================================
void orientServoControl(){
    if(is_new_orient){      // servo's position changes only if PWM command changes
        is_new_orient = false;
        ledcWrite(ORIENT_CHANNEL,orient_pos);
    }
}
//=============================================================================================================================
//================================================= orient servo control end ==================================================
//=============================================================================================================================





//=============================================================================================================================
//================================================ weapon servo control start =================================================
//=============================================================================================================================
void weaponServoControl(){
    if(weapon_mode == AUTONOMOUS) weaponAutoControl(); 
    else if(weapon_mode == MANUAL) weaponManualControl();
}
//=============================================================================================================================
//================================================= weapon servo control end ==================================================
//=============================================================================================================================





//=============================================================================================================================
//================================================= weapon auto control start =================================================
//=============================================================================================================================
void weaponAutoControl(){
    if(is_weapon_move){           // weapon's position only changes if timer interrupt happens
        is_weapon_move = false;   // flag reset

        // calculate angle range
        // note that only 180 degrees out of 270 will be used because the rest 90 is only for initial position
        int max_pos = map(PTM, 1, 255, (WEAPON_MIN_ANGLE + (WEAPON_MAX_ANGLE - WEAPON_MIN_ANGLE) / 3), WEAPON_MAX_ANGLE);
        int front_angle = WEAPON_MAX_ANGLE - (WEAPON_MAX_ANGLE - WEAPON_MIN_ANGLE) / 3;
        int angle_max = front_angle + abs(max_pos - front_angle);
        int angle_min = front_angle - abs(max_pos - front_angle);

        if(weapon_pos < angle_min || weapon_pos > angle_max){
            weapon_pos = front_angle;
        }
        ledcWrite(WEAPON_CHANNEL,weapon_pos);

        // change position(PWM) by one whenever an interrupt happens
        weapon_pos += weapon_dir;
        if(weapon_pos == angle_max || weapon_pos == angle_min){
            weapon_dir = -weapon_dir;
        }
    }
}
//=============================================================================================================================
//================================================== weapon auto control end ==================================================
//=============================================================================================================================





//=============================================================================================================================
//================================================ weapon manual control start ================================================
//=============================================================================================================================
void weaponManualControl(){
    if(is_new_weapon){      // servo's position changes only if PWM command changes
        weapon_pos = map(PTM, 1, 255, WEAPON_MIN_ANGLE, WEAPON_MAX_ANGLE);
        ledcWrite(WEAPON_CHANNEL,weapon_pos);
    }
}
//=============================================================================================================================
//================================================= weapon manual control end =================================================
//=============================================================================================================================





//=============================================================================================================================
//================================================= back motor control start ==================================================
//=============================================================================================================================
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
    if(is_new_speed){      // motors' PWM changes only if PWM command changes
        is_new_speed = false;
        ledcWrite(BACK_LEFT_CHANNEL,back_left_speed);
        ledcWrite(BACK_RIGHT_CHANNEL,back_right_speed); 
    } 
}
//=============================================================================================================================
//================================================== back motor control end ===================================================
//=============================================================================================================================





//=============================================================================================================================
//================================================= front motor control start =================================================
//=============================================================================================================================
void frontMotorControl(){
    if(back_dir != CLOCKWISE || front_standby == STANDBY){
        digitalWrite(FRONT_DIR_PIN, LOW);
        ledcWrite(FRONT_CHANNEL,0); 
    } else {
        digitalWrite(FRONT_DIR_PIN, HIGH);
        ledcWrite(FRONT_CHANNEL,200); // front motor is quicker than back so set to 200
    }
}
//=============================================================================================================================
//================================================== front motor control end ==================================================
//=============================================================================================================================











//=============================================================================================================================
//====================================================== autonomy start =======================================================
//=============================================================================================================================
//=============================================================================================================================
//==================================================== vive receive start =====================================================
//=============================================================================================================================
void receiveCommand(){      // receive commands from auxiliary board
    command_1 = digitalRead(COMM_PIN_1); // read first {0,1} data
    command_2 = digitalRead(COMM_PIN_2); // read second {0,1} data
}
//=============================================================================================================================
//===================================================== vive receive end ======================================================
//=============================================================================================================================





//=============================================================================================================================
//====================================================== turn left start ======================================================
//=============================================================================================================================
void turnLeft(){    
    // orient servo turning to left
    ledcWrite(ORIENT_CHANNEL,ORIENT_MIN_ANGLE);
    
    digitalWrite(BACK_DIR_PIN,HIGH);
    digitalWrite(BACK_IDIR_PIN,LOW); 
    
    // only back right wheel rotates
    ledcWrite(BACK_LEFT_CHANNEL,0);
    ledcWrite(BACK_RIGHT_CHANNEL,255);
}
//=============================================================================================================================
//======================================================= turn left end =======================================================
//=============================================================================================================================





//=============================================================================================================================
//===================================================== turn right start ======================================================
//=============================================================================================================================
void turnRight(){    
    // orient servo turning to right
    ledcWrite(ORIENT_CHANNEL,ORIENT_MAX_ANGLE);
    
    digitalWrite(BACK_DIR_PIN,HIGH);
    digitalWrite(BACK_IDIR_PIN,LOW); 

    // only back left wheel rotates
    ledcWrite(BACK_LEFT_CHANNEL,255);
    ledcWrite(BACK_RIGHT_CHANNEL,0);
}
//=============================================================================================================================
//====================================================== turn right end =======================================================
//=============================================================================================================================




//=============================================================================================================================
//====================================================== forwards start =======================================================
//=============================================================================================================================
void forwards(){   
    // orient servo turning to middle 
    ledcWrite(ORIENT_CHANNEL,(ORIENT_MAX_ANGLE + ORIENT_MIN_ANGLE) / 2);
    
    digitalWrite(BACK_DIR_PIN,HIGH);
    digitalWrite(BACK_IDIR_PIN,LOW); 

    // both two back wheels rotate
    ledcWrite(BACK_LEFT_CHANNEL,255);
    ledcWrite(BACK_RIGHT_CHANNEL,255);
}
//=============================================================================================================================
//======================================================= forwards end ========================================================
//=============================================================================================================================





//=============================================================================================================================
//====================================================== backwards start ======================================================
//=============================================================================================================================
void backwards(){    
    // orient servo turning to middle 
    ledcWrite(ORIENT_CHANNEL,(ORIENT_MAX_ANGLE + ORIENT_MIN_ANGLE) / 2);
    
    digitalWrite(BACK_DIR_PIN,LOW);
    digitalWrite(BACK_IDIR_PIN,HIGH);

    // both two back wheels rotate inversely
    ledcWrite(BACK_LEFT_CHANNEL,255);
    ledcWrite(BACK_RIGHT_CHANNEL,255);
}
//=============================================================================================================================
//======================================================= backwards end =======================================================
//=============================================================================================================================





//=============================================================================================================================
//====================================================== backwards start ======================================================
//=============================================================================================================================
void standby(){   
    // orient servo turning to middle  
    ledcWrite(ORIENT_CHANNEL,(ORIENT_MAX_ANGLE + ORIENT_MIN_ANGLE) / 2);
    
    digitalWrite(BACK_DIR_PIN,LOW);
    digitalWrite(BACK_IDIR_PIN,LOW); 

    // both two back wheels stop
    ledcWrite(BACK_LEFT_CHANNEL,0);
    ledcWrite(BACK_RIGHT_CHANNEL,0);
}
//=============================================================================================================================
//======================================================= backwards end =======================================================
//=============================================================================================================================





//=============================================================================================================================
//====================================================== auto run start =======================================================
//=============================================================================================================================
void autoRun(){    // move according to commands from the auxiliary board
    if(!command_1 && !command_2)standby();
    else if(!command_1 && command_2)turnLeft();
    else if(command_1 && !command_2)turnRight();
    else forwards();
}
//=============================================================================================================================
//======================================================= auto run end ========================================================
//=============================================================================================================================
//=============================================================================================================================
//======================================================= autonomy end ========================================================
//=============================================================================================================================











//=============================================================================================================================
//===================================================== data print start ======================================================
//=============================================================================================================================
void show(){
    if(millis() % 500 == 0){ // print every half second
        if(cur_mode == AUTONOMOUS)Serial.println("AUTONOMOUS");
        else Serial.println("MANUAL");
//*********************************************************** WiFi ************************************************************
        Serial.print("VRX: ");Serial.print(VRX);
        Serial.print("    VRY: ");Serial.print(VRY);
        Serial.print("    PTM: ");Serial.print(PTM);
        Serial.print("    SW_1: ");Serial.print(SW_1);
        Serial.print("    SW_2: ");Serial.println(SW_2);
//*********************************************************** servo ***********************************************************
        Serial.print("Orient: ");
        Serial.print(orient_pos);
        Serial.print("    Weapon: ");
        Serial.println(weapon_pos);
        if(weapon_mode == AUTONOMOUS)Serial.println("weapon is auto");
        else Serial.println("Weapon is manual");
//*********************************************************** motor ***********************************************************
        if(back_dir == CLOCKWISE) Serial.println("CLOCKWISE");
        else if(back_dir == COUNTERCLOCKWISE) Serial.println("COUNTERCLOCKWISE");
        else Serial.println("STANDBY");
        
        Serial.print("back left PWM: ");
        Serial.print(back_left_speed);
        Serial.print("    back right PWM: ");
        Serial.println(back_right_speed);
//********************************************************** command **********************************************************
        if(command_1 || command_2){
            Serial.print("command: ");
            Serial.print(command_1);
            Serial.println(command_2);  
        }
    }
}
//=============================================================================================================================
//====================================================== data print end =======================================================
//=============================================================================================================================
//=============================================================================================================================
// ===================================================== own function end =====================================================
//=============================================================================================================================












//=============================================================================================================================
// ======================================================== main start ========================================================
//=============================================================================================================================
// =====================================================================
// ============================= SETUP =================================
// =====================================================================
void setup()
{
    Serial.begin(115200);
    
    // ========================= I2C start =============================
    ESP_ERROR_CHECK(i2c_master_init());
    // ========================== I2C end ==============================

    // ===================== Interrupts start ==========================
    timer = timerBegin(0, 240, true);                      
    timerAttachInterrupt(timer, &readI2COnTimer, true);    
    timerAlarmWrite(timer, 50000, true);                  
    timerAlarmEnable(timer);                            
    // ====================== Interrupts end ===========================

    // ========================= LED start =============================
    SetupFastLED(); // set the LEDs
    // ========================== LED end ==============================





    // =================================================================
    // ===================== own function start ========================
    // =================================================================
    STA_UDP_Set();        // set WiFi UDP
    timerSetup();         // set timer 1 and interrupt
    pinSetup();           // set pin mode
    PWMSetup();           // set ledc
    // =================================================================
    // ====================== own function end =========================
    // =================================================================
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






    // =================================================================
    // ===================== own function start ========================
    // =================================================================
    preset();                 // stop all 
    WiFi_Reconnect();         // if WiFi is disconnected, try to reconnect
    UDPreceiveData();         // receive data from controller
    show();                   // print data
    
    if(!gameStatus) return;   // cannot move if game status is off
    if(health == 0) return;   // cannot move if the car is dead

    if(autoMode) cur_mode = AUTONOMOUS;   // determine current mode
    else cur_mode = MANUAL;

    if(cur_mode == AUTONOMOUS){
        receiveCommand();     // receive data from auxiliary board
        autoRun();            // autonomy
    }
    else{  
        orientServoControl(); // change orient servo's position
        weaponServoControl(); // change weapon servo's position
        backMotorControl();   // control back motors  
        frontMotorControl();  // control front motors   
    }
    // =================================================================
    // ====================== own function end =========================
    // =================================================================
}
// =====================================================================
// ========================== END OF LOOP ==============================
// =====================================================================
//=============================================================================================================================
// ========================================================= main end =========================================================
//=============================================================================================================================
