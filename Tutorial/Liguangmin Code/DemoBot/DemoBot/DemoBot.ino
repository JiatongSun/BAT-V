/*
 * DemoBot code
 */

// ========================= includes =============================
// add any libraries here


// =================================================================
// ========================= I2C start =============================
// =================================================================
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include <WiFi.h>;//include WiFi library
#include <WiFiUdp.h>;//include UDP packet

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 128                             // data buffer length of test buffer
#define W_LENGTH 1                                  // data length for w, [0,DATA_LENGTH]
#define R_LENGTH 16                                 // data length for r, [0,DATA_LENGTH]

#define I2C_MASTER_SCL_IO (gpio_num_t)33            // gpio number for I2C master clock
#define I2C_MASTER_SDA_IO (gpio_num_t)25            // gpio number for I2C master data
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
const int LEDC_CHANNEL = 0;//interrupt channel 0
const int LEDC_CHANNEL1 = 1;//interrupt channel 1
const int LEDC_CHANNEL_SERVO = 2;//interrupt channel 2
const int LEDC_CHANNEL_WEAPON = 3;
const int LEDC_RESOLUTION_BITS=13;//bits of resolution is 13
const int LEDC_RESOLUTION = ((1<<LEDC_RESOLUTION_BITS)-1);//resolution is 2^13
const int LEDC_FREQ_HZ = 5000;//interrupt frequency set to be 5000
const int LEDC_FREQ_HZ_SERVO = 50;//interrupt frequency for servo set to be 50
const int A1= 18;//1A used to control the direction on H bridge
const int N_A1 = 19;//4A used to control the NOT direction on H bridge
const int Enable = 4;//Enable pin used to control the PWM on H bridge 1
const int Enable1 = 0;//Enable pin used to control PWM on H bridge 2
const int ServoControl = 22;//Servo control pin used to control the PWM for the servo
const int PhotoDiode = 27;//Pin saved for communication with Photodiode
const int Weaponcontrol = 9;

/*Constants for WiFiUDP*/
int port = 1609;//Port number 1609 used for communication
int cb = 0;//A flag used to tell if a package is received
int val = 0;//A value used to store analogread value
int servoread = 0;//A value used to store servo receving value
int weaponread = 0;
uint32_t weaponduty;
void receivePacket();//function prototype for receiving packet
int MessageReceived = 0;//declare the messagereceived integer
const char* ssid = "SmartGuangming";//initialize WiFi name
const int onboardLED = 2;
WiFiUDP  UDPTestServer;//Setup UDP protocal
IPAddress myIPaddress(192,168,1,142);//Declare my own IP address
const int UDP_PACKET_SIZE = 100;//declare the packet size to be 100
byte packetBuffer[UDP_PACKET_SIZE + 1];//delcare the an array with size of packet+1
char udpBuffer[UDP_PACKET_SIZE];//declare an array with size of udp packet
IPAddress ipTarget(192,168,1,125);//declare the ipaddress sending packet to

//--------------------------------------------------------------------------------------
//The following are the constants used for the VIVE sensing
volatile int prevT =0;
volatile int currT = 0;
volatile int flagx = 1;
volatile int flagy = 0;
volatile int start = 0 ;
volatile int counter = 0;
volatile int timediff = 0;
volatile int endofAuto = 1;
//The end for constants used for VIVE sensing
//--------------------------------------------------------------------------------------


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
//--------------------------------------------------------------------------------------
//The following function is stored in RAM for interrupt
void IRAM_ATTR calcT(){ // interrupt function
  if(digitalRead(PhotoDiode) == LOW){ // This checks for pulse width
    currT = micros(); // finds time in microseconds
    timediff = currT - prevT; // calculate time of the pulse width
    if(timediff < 60){ // if the width is less than 60, which is less than the pulse width of the sync pulse, but larger than the x/y pulses
      if(flagx){ // if the x flag is on
        // Serial.print("x-pulse: "); // print out the pulse value
        // Serial.println(timediff);
        delayMicroseconds(1);
        flagx = 0; // set the flag to 0 so that we dont read the next pulse as x
        flagy = 1; // turn this flag on to read the next pulse as y
      }
      else if(flagy){ // if x flag isnt on but y flag is on
        // Serial.print("  y-pulse: "); // print out the pulse value
        // Serial.println(timediff);
        delayMicroseconds(1);
//        flagx = 1;
        flagy = 0; // reset the flag back to 0. At this point, both the x and y flags are off.
      }
      counter = 0; // reset the counter. This is mainly for the else statement where we look at the counter value and the flags on line 49 (if nothing changes)
    }
    else{
//      Serial.println("im counting up");
      counter++; // increments the counter
//      Serial.println(counter);
      if(counter == 3){ // if I have counted 3 sync pulses, then I know the next pulse has to be an x pulse.
        flagx = 1; // because we know the next one is an x pulse, we can safely set the x flag on and prepare for the next pulse.
        counter = 1; // reset the counter to 1. This time we did not set it to 0 because we need to calculate the x time distance, which requires the counter to be 1 (as seen in the else statement below)
//        Serial.println(counter);
      }
    }
    start = prevT; // sets the start time to the rising edge of the pulse. This helps find the x/y time distance
    prevT = currT; // resets the previous time to the current time to be used in finding the pulse width
  }
  else{ // Here we are looking at the rising edge.
//    Serial.println("im in else");
    currT = micros(); // finds the time in microseconds
    timediff = currT - start; //calculate the time difference
    if(counter == 1 && (flagx || flagy)){ // checks whether the counter is 1 and if either flag is on. The counter is always on before the x and y pulses, as well as after the y pulse; however, we prevent the issue of counting the sync pulse after the y pulse by checking the flags, which are both 0 after the y pulse.
      if(flagx){ // checks the x flag. If x flag is on here, then we are on the x pulse. Else, on the y pulse.
        // Serial.print("    x time:"); // print the x time distance
        // Serial.println(timediff);
        digitalWrite(onboardLED,HIGH);
        delayMicroseconds(1);
      }
      else{ // on y pulse
        // Serial.print("      y time:"); // print the y time distance
        // Serial.println(timediff);
        delayMicroseconds(1);
      }
    }
    prevT = currT; // set the previous time to current time (on rising edge) for pulse width.
  }
}
//The end of the interrupt function for VIVE sensing
//--------------------------------------------------------------------------------------
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
#define ROBOTNUM    2               // robot number on meta team (1-4)
#define TEAMCOLOR   BLUE            // color for the robot team, either RED or BLUE
// ==========================

#define NEO_LED_PIN 12              // pin attached to LED ring
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

int first = 1;
int maxHp = 0;
double multiplier = 0.0;
int healthLeds[] = {1,2,3,4,5,7,8,9,10,11,13,14,15,16,17,19,20,21,22,23};

void ShowHealth(int health)
{
    // TODO: implement this function
    int healthLength = 20;
    Serial.print( "HEALTH " );
    Serial.println(health);
    if(first && (health != 0)){
      maxHp = health;
    Serial.print( "Max hp " );
    Serial.println(maxHp);
      for(int i = 0; i < healthLength; i++){
        leds[healthLeds[i]] = HEALTHCOLOR;
      }
      multiplier = 20.0/maxHp;
      first = 0;
    }
    else{
      int hpDiff = maxHp - health;
      if(hpDiff > 0){
        int minusHp = hpDiff*multiplier;
        for(int i = 0; i < minusHp; i++){
          leds[healthLeds[i]] = 0;
        }
      }
    }
}

void clearLEDs(void)
{
    for(int i = 0; i < NUM_LEDS; i++)
    {
        leds[i] = 0;
    }
}


int maxTime = 15;

void ShowRespawnTimer(int respawnTime)
{
    // TODO: implement this function
    int numberleds = map(respawnTime,0,maxTime,0,24);
    for(int i = 0; i <= numberleds; i++){
      leds[i] = RED;
    }
}
// =================================================================
// ========================== LED end ==============================
// =================================================================


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

    // put your setup code here, to run once:
  Serial.println();   //Printing information regarding to connection
  Serial.print("Connecting to");
  Serial.println(ssid);
  pinMode(onboardLED,OUTPUT);

  WiFi.mode(WIFI_STA);//Set WiFi mode to station
  WiFi.config(myIPaddress,IPAddress(192,168,1,1),IPAddress(255,255,255,0));
  //Set up IP address, GateWay and Mask
  WiFi.begin(ssid);//Begin WiFi connection at selected WiFi name
  WiFi.setSleep(false);//Set mode of WiFi to let it never sleep
  while (WiFi.status()!=WL_CONNECTED);//A loop runs until WiFi is connected
  {
    delay(500);//wait for 0.5 seconds
    Serial.print(".");//print a dot while waiting
  }
  Serial.println("WiFi Connected ");//Print WiFi is connected when it is connected
  // digitalWrite(onboardLED,HIGH);
  /* The following lines turns the WiFi mode into AP mode */
  // WiFi.mode(WIFI_AP);
  // WiFi.softAP(ssid);
  // delay(100);
  // WiFi.softAPConfig(myIPaddress,IPAddress(192,168,1,1),IPAddress(255,255,255,0));

  UDPTestServer.begin(port);//begin UDP protocal communication on the assigned port
  packetBuffer[UDP_PACKET_SIZE] = 0;//Null terminated the packetBuffer
  ledcSetup(LEDC_CHANNEL,LEDC_FREQ_HZ,LEDC_RESOLUTION_BITS);//Set up LEDC channel 0 at 5000HZ, 2^13 resolution
  ledcSetup(LEDC_CHANNEL1,LEDC_FREQ_HZ,LEDC_RESOLUTION_BITS);//Set up LEDC Channel 1 at 5000Hz, 2^13 resolution
  ledcSetup(LEDC_CHANNEL_SERVO,LEDC_FREQ_HZ_SERVO,LEDC_RESOLUTION_BITS);//Setup LEDC Channel 2 at 50Hz, 2^13 resolution
  ledcSetup(LEDC_CHANNEL_WEAPON,LEDC_FREQ_HZ_SERVO,LEDC_RESOLUTION_BITS);
  ledcAttachPin(Enable,LEDC_CHANNEL);//Attach ledc at LEDC_CHANNEL to PWM1
  ledcAttachPin(Enable1,LEDC_CHANNEL1);//Attch ledc at LEDC_CHANNEL_1 to PWM2
  ledcAttachPin(ServoControl,LEDC_CHANNEL_SERVO);//Attach ledc at LEDC_CHANNEL_SERVO to servo control pin
  ledcAttachPin(Weaponcontrol,LEDC_CHANNEL_WEAPON);
  pinMode(A1,OUTPUT);//Set 1A as output
  pinMode(N_A1,OUTPUT);//Set 4A as output
  // pinMode(EnableControl,INPUT);
  pinMode(ServoControl,OUTPUT);//Set ServoControl as output
  pinMode(PhotoDiode,INPUT);//Set the PhotoDiode pin as an input pin
  pinMode(Weaponcontrol,OUTPUT);


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
    while (WiFi.status()!=WL_CONNECTED)
    {
      WiFi.begin(ssid);//Begin WiFi connection at selected WiFi name
    }
    // ========================== I2C end ==============================

    // ========================= LED start =============================
    ShowRobotNum();         // set the LEDs for the robot number
    ShowHealth(health);     // set the LEDs for the health
    // ========================== LED end ==============================
    // ========================== WiFi Control start ==============================
    receivePacket();//run the receive packet subroutine
    int psudoduty = val -2000;//remap the DC PWM received -> -1000 to 1000
    Serial.println("WeaponRead");
    Serial.println(weaponread);//Print out the received value (for debugging)
    uint32_t duty = LEDC_RESOLUTION*abs(psudoduty)/1000;//duty cycle = psudoduty*resolution/1000
    uint32_t servoduty = map(servoread,4000,5000,450,1050)*LEDC_RESOLUTION/10000;//map the servo duty to 450-1050 which correspond to full left and full right

    if (weaponread == 2000)
    {
       weaponduty = 450*LEDC_RESOLUTION/10000;
    }
    else if(weaponread ==1000)
    {
       weaponduty = 1050*LEDC_RESOLUTION/10000;
    }
    if (health == 0)
    {
        clearLEDs();
        first=1;
        ShowRespawnTimer(respawnTimer);

    }
     FastLEDshowESP32();
    if ((servoread <= 4200 || servoread >= 4800) && psudoduty < 0)
    {
      duty = LEDC_RESOLUTION*850/1000;
    }

    if (health ==0 || gameStatus == 0 || autoMode ==1)
    {
      duty = 0;
      servoduty =0;
      weaponduty = 750*LEDC_RESOLUTION/10000;
    }
    // if (autoMode == 1 && gameStatus == 1)
    // {
    //   attachInterrupt(digitalPinToInterrupt(PhotoDiode),calcT,CHANGE);//make the interrupt
    // }
    //
    // if (autoMode == 0)
    // {
    //   detachInterrupt(digitalPinToInterrupt(PhotoDiode));
    // }
  //When psudoduty is greater than 0, turn the wheels forward by setting both direction pins to HIGH
    if ((psudoduty) >= 0){
    digitalWrite(A1,HIGH);
    digitalWrite(N_A1,LOW);
    Serial.println("Rotating forward");
    }

    //When the psudoduty is smaller than 0, turn the wheels backward by setting both direction pins to LOW
    else if ((psudoduty)<0)
    {
    Serial.println("Rotating backward");
    digitalWrite(A1,LOW);
    digitalWrite(N_A1,HIGH);
    }

    ledcWrite(LEDC_CHANNEL,duty);//Run the motors at given duty cycle to control there speed
    ledcWrite(LEDC_CHANNEL1,duty);//Run the motors at given duty cycle to control there speed
      Serial.println("Servo duty");
      Serial.println(map(servoread,4000,5000,450,1050));//Print out the servo duty(For debugging)
    ledcWrite(LEDC_CHANNEL_SERVO,servoduty);//PWM to servo motor based on the received duty cycle
    ledcWrite(LEDC_CHANNEL_WEAPON,weaponduty);

    cb = 0;//Set the receive pack back to 0
    // ========================== WiFi Control end ==============================


}
void receivePacket(){
  cb = UDPTestServer.parsePacket();//Set cb to the length of the packet received
  if (cb){//When cb is not 0
    UDPTestServer.read(packetBuffer,UDP_PACKET_SIZE);//Read packetbuffer with UDP_PACKET_SIZE from the UDP packet received
    val = (packetBuffer[1]<<8 |packetBuffer[0]);//Save the first two bytes of the packet received as the PWM for DC motors
    servoread = (packetBuffer[3]<<8|packetBuffer[2]);//Save the second two bytes of the packet received as the PWM for servo motors
    weaponread = (packetBuffer[5]<<8|packetBuffer[4]);
  }
}
// =====================================================================
// ========================== END OF LOOP ==============================
// =====================================================================
