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

void ShowHealth(int health)
{
    // TODO: implement this function
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
    // TODO: implement this function
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


}
// =====================================================================
// ========================== END OF LOOP ==============================
// =====================================================================
