//=============================================================================================================================
//============================================== MEAM 510 Final Auxiliary Board ===============================================
//=============================================================================================================================
//******************************************************* Team 04: BAT_V ******************************************************
//*************************************** Members: Jiatong Sun, Xinyue Wei, Haojiong Lu ***************************************
//=============================================================================================================================
//============================================== MEAM 510 Final Auxiliary Board ===============================================
//=============================================================================================================================





//=============================================================================================================================
//======================================================= library start =======================================================
//=============================================================================================================================
#include "driver/i2s.h"
#include "freertos/queue.h"
#include <pgmspace.h>

#include "audio_example_file.h"
//=============================================================================================================================
//======================================================== library end ========================================================
//=============================================================================================================================





//=============================================================================================================================
//================================================= constant definition start =================================================
//=============================================================================================================================
//*********************************************************** calc ************************************************************
#define    OUT(x,LIM)               (abs(x)-LIM) > 0 ? true : false
//*********************************************************** vive ************************************************************
#define    X_MAX                    5500        // maximal x coordinate of the arene
#define    Y_MAX                    6100        // maximal y coordinate of the arene
#define    X_MIN                    3000        // minimal x coordinate of the arene
#define    Y_MIN                    1700        // minimal y coordinate of the arene
#define    AIM_X                    4500        // target x coordinate
#define    AIM_Y                    3000        // target y coordinate
#define    ORI_LIM                  25          // thresh for deciding if the car's orient is correct
#define    POS_LIM_X                25          // thresh for deciding if the car's x position is correct
#define    POS_LIM_Y                500         // thresh for deciding if the car's y position is correct
//=============================================================================================================================
//================================================== constant definition end ==================================================
//=============================================================================================================================





//=============================================================================================================================
//=================================================== pin definition start ====================================================
//=============================================================================================================================
//*********************************************************** comm ************************************************************
#define    COMM_PIN_1               32           // pin 1 for communication
#define    COMM_PIN_2               33           // pin 2 for communication 
//*********************************************************** vive ************************************************************
#define    VIVE_PIN_1               21           // pin 1 for receiving vive
#define    VIVE_PIN_2               39           // pin 2 for receiving vive
//************************************************************ I2S ************************************************************
#define    BCK_PIN                  25           // BCK pin of I2S
#define    LRCK_PIN                 26           // LRCK pin of I2S
#define    DATA_OUT_PIN             22           // data out pin of I2S
//=============================================================================================================================
//==================================================== pin definition end =====================================================
//=============================================================================================================================





//=============================================================================================================================
//============================================= global variables definition start =============================================
//=============================================================================================================================
//*********************************************************** vive ************************************************************
bool vive_state_1 = false, last_vive_state_1 = false;             // vive voltage level
int sync_cnt_1 = 0;                                               // sync signal cumulative count
long signal_start_1 = 0;                                          // time of rising edge 
long sync_end_1 = 0;                                              // time of sync signal's falling edge
long x_coor_1 = 0, y_coor_1 = 0;                                  // x and y coordinates
long last_x_coor_1 = 0, last_y_coor_1 = 0;                        // moving averaged x and y coordinates
bool is_x_found_1 = false, is_y_found_1 = false;                  // flag indicating if x or y is found
bool is_vive_read_1 = false;                                      // flag indicating if x and y are found
bool is_vive_display_1 = false;                                   // flag indicating display condition

bool vive_state_2 = false, last_vive_state_2 = false;             // vive voltage level
int sync_cnt_2 = 0;                                               // sync signal cumulative count
long signal_start_2 = 0;                                          // time of rising edge 
long sync_end_2 = 0;                                              // time of sync signal's falling edge
long x_coor_2 = 0, y_coor_2 = 0;                                  // x and y coordinates
long last_x_coor_2 = 0, last_y_coor_2 = 0;                        // moving averaged x and y coordinates
bool is_x_found_2 = false, is_y_found_2 = false;                  // flag indicating if x or y is found
bool is_vive_read_2 = false;                                      // flag indicating if x and y are found
bool is_vive_display_2 = false;                                   // flag indicating display condition
//********************************************************* autonomy **********************************************************
long dx = x_coor_1 - x_coor_2;                                    // x difference of two vives
long dy = y_coor_1 - y_coor_2;                                    // y difference of two vives
long x_car = 0.5 * (x_coor_1 + x_coor_2);                         // x coordinate of the car
long y_car = 0.5 * (y_coor_1 + y_coor_2);                         // y coordinate of the car
long pos_x_error = x_car - AIM_X;                                 // x position error
long pos_y_error = y_car - AIM_Y;                                 // y position error
bool send_sth1 = false, send_sth2 = false;                        // flags representing commands sent to the main board
//************************************************************ i2s ************************************************************
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
    .bck_io_num = BCK_PIN,                                        // I2S BCK pin
    .ws_io_num = LRCK_PIN,                                        // I2S LRCK pin
    .data_out_num = DATA_OUT_PIN,                                 // I2S data out pin
    .data_in_num = -1
};
//=============================================================================================================================
//============================================== global variables definition end ==============================================
//=============================================================================================================================





//=============================================================================================================================
//==================================================== main function start ====================================================
//=============================================================================================================================
void setup(){
    Serial.begin(115200); 
    pinSetup();                 // set up pins
}  

void loop(){   
    viveReceive();              // receive vive data
    processData();              // process vive data and send them to the main board
    show();                     // print data
}
//=============================================================================================================================
//===================================================== main function end =====================================================
//=============================================================================================================================





//=============================================================================================================================
//====================================================== pin setup start ======================================================
//=============================================================================================================================
void pinSetup(){
//******************************************************** connection *********************************************************
    pinMode(COMM_PIN_1,OUTPUT);
    pinMode(COMM_PIN_2,OUTPUT);
//*********************************************************** vive ************************************************************
    pinMode(VIVE_PIN_1,INPUT); 
    pinMode(VIVE_PIN_2,INPUT); 
}
//=============================================================================================================================
//======================================================= pin setup end =======================================================
//=============================================================================================================================





//=============================================================================================================================
//==================================================== vive receive start =====================================================
//=============================================================================================================================
void viveReceive(){
//********************************************************** vive 1 ***********************************************************
    // read current voltage level
    if(digitalRead(VIVE_PIN_1)==HIGH) vive_state_1 = true;                
    else vive_state_1 = false;

    // capture rising edge or falling edge
    if(vive_state_1 && !last_vive_state_1) signal_start_1 = micros();   // store time on rising edge
    else if(!vive_state_1 && last_vive_state_1){
        long bandwidth_1 = micros() - signal_start_1;                   // calculate bandwidth on falling edge  
        
        if(bandwidth_1 > 60){       
            sync_cnt_1 ++;                                              // a new sync is found
            sync_end_1 = micros();                                      // save current time for furhter coordinate calculation
        } else if (bandwidth_1 < 60 && bandwidth_1 > 10){
            if(sync_cnt_1 == 3){
                long x_temp = micros() - sync_end_1;                    // an x is found
                if(x_temp < X_MAX && x_temp > X_MIN){                   // judge if the x coordinate is reasonable
                    is_x_found_1 = true;
                    x_coor_1 = (x_temp * 2 + last_x_coor_1) / 3;        // applying a moving average filter
                    last_x_coor_1 = x_coor_1;                           // store the result for next loop's calculation
                }
                else is_x_found_1 = false;
            } else if(sync_cnt_1 == 1){
                long y_temp = micros() - sync_end_1;                    // a y is found
                if(y_temp < Y_MAX && y_temp > Y_MIN){                   // judge if the y coordinate is reasonable
                    is_y_found_1 = true;
                    y_coor_1 = (y_temp * 2 + last_y_coor_1) / 3;        // applying a moving average filter
                    last_y_coor_1 = y_coor_1;                           // store the result for next loop's calculation
                }
            }
            sync_cnt_1 = 0;                                             // reset sync count 
        }
    }
    last_vive_state_1 = vive_state_1;                                   // store current voltage level
    if (is_x_found_1 && is_y_found_1) {
        is_vive_read_1 = true;                                          // a set of coordinate (x,y) is received
        is_vive_display_1 = true;                                       // can be displayed in this loop
        is_x_found_1 = false;                                           // reset flag
        is_y_found_1 = false;                                           // reset flag
    }
//********************************************************** vive 2 *********************************************************** 
    // read current voltage level
    if(digitalRead(VIVE_PIN_2)==HIGH) vive_state_2 = true;
    else vive_state_2 = false;

    // capture rising edge or falling edge
    if(vive_state_2 && !last_vive_state_2) signal_start_2 = micros();   // store time on rising edge
    else if(!vive_state_2 && last_vive_state_2){
        long bandwidth_2 = micros() - signal_start_2;                   // save current time for furhter coordinate calculation
        
        if(bandwidth_2 > 60){
            sync_cnt_2 ++;                                              // a new sync is found
            sync_end_2 = micros();                                      // save current time for furhter coordinate calculation
        } else if (bandwidth_2 < 60 && bandwidth_2 > 10){
            if(sync_cnt_2 == 3){
                long x_temp = micros() - sync_end_2;                    // an x is found
                if(x_temp < X_MAX && x_temp > X_MIN){                   // judge if the x coordinate is reasonable
                    is_x_found_2 = true;
                    x_coor_2 = (x_temp * 2 + last_x_coor_2) / 3;        // applying a moving average filter
                    last_x_coor_2 = x_coor_2;                           // store the result for next loop's calculation
                }
                else is_x_found_2 = false;
            } else if(sync_cnt_2 == 1){
                long y_temp = micros() - sync_end_2;                    // a y is found
                if(y_temp < Y_MAX && y_temp > Y_MIN){                   // judge if the y coordinate is reasonable
                    is_y_found_2 = true;
                    y_coor_2 = (y_temp * 2 + last_y_coor_2) / 3;        // applying a moving average filter
                    last_y_coor_2 = y_coor_2;                           // store the result for next loop's calculation
                }
                else is_y_found_2 = false;
            }
            sync_cnt_2 = 0;                                             // reset sync count 
        }
    }
    last_vive_state_2 = vive_state_2;                                   // store current voltage level
    if (is_x_found_2 && is_y_found_2) {
        is_vive_read_2 = true;                                          // a set of coordinate (x,y) is received
        is_vive_display_2 = true;                                       // can be displayed in this loop
        is_x_found_2 = false;                                           // reset flag
        is_y_found_2 = false;                                           // reset flag
    }
}
//=============================================================================================================================
//===================================================== vive receive end ======================================================
//=============================================================================================================================





//=============================================================================================================================
//================================================== generate command start ===================================================
//=============================================================================================================================
void processData(){
    if(!is_vive_read_1 || !is_vive_read_2) return;                      // data will only be sent if both coordinates are found
    x_car = 0.5 * (x_coor_1 + x_coor_2);
    y_car = 0.5 * (y_coor_1 + y_coor_2);
    if((y_car - AIM_Y) > POS_LIM_Y){
        // the car is not facing to the y direction and needs to adjust orientation first
        if(x_coor_2 - x_coor_1 >  ORI_LIM) {                            
            digitalWrite(COMM_PIN_1,1);                                 // turn right
            digitalWrite(COMM_PIN_2,0); 
            send_sth1 = true;
            send_sth2 = false;
        }else if(x_coor_1 - x_coor_2 >  ORI_LIM) {
            digitalWrite(COMM_PIN_1,0);                                 // turn left
            digitalWrite(COMM_PIN_2,1); 
            send_sth1 = false;
            send_sth2 = true;
        }else{
            digitalWrite(COMM_PIN_1,1);                                 // go forward
            digitalWrite(COMM_PIN_2,1); 
            send_sth1 = true;
            send_sth2 = true;
        }
    }
    // the car is facing the right direction and needs to go straight to its target
    else if((y_car - AIM_Y) < POS_LIM_Y){
         if((x_car - AIM_X) > POS_LIM_X){
            digitalWrite(COMM_PIN_1,0);                                 // turn left
            digitalWrite(COMM_PIN_2,1); 
         }else if((AIM_X-x_car) > POS_LIM_X){
            digitalWrite(COMM_PIN_1,1);                                 // turn right
            digitalWrite(COMM_PIN_2,0); 
        }else{
            digitalWrite(COMM_PIN_1,0);                                 // arrives target position, stop now
            digitalWrite(COMM_PIN_2,0);
            playPROGMEMsample(audio_table);                             // broadcast laugh sound to indicate the autonomy works
        }
    }
}
//=============================================================================================================================
//=================================================== generate command end ====================================================
//=============================================================================================================================





//=============================================================================================================================
//========================================================= I2S start =========================================================
//=============================================================================================================================
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
//=============================================================================================================================
//========================================================== I2S end ==========================================================
//=============================================================================================================================





//=============================================================================================================================
//===================================================== data print start ======================================================
//=============================================================================================================================
void show(){
    if(millis() % 500 == 0){ // print every half second
//*********************************************************** vive ************************************************************
        if(is_vive_display_1 && is_vive_display_2){    
            Serial.print("x_1 = ");
            Serial.print(x_coor_1);
            Serial.print("    y_1 = ");
            Serial.print(y_coor_1);
            
            Serial.print("    x_2 = ");
            Serial.print(x_coor_2);
            Serial.print("    y_2 = ");
            Serial.println(y_coor_2);
            
            Serial.print(send_sth1);
            Serial.println(send_sth2);
            
            is_vive_display_1 = false;
            is_vive_display_2 = false;
        }
    }
}
//=============================================================================================================================
//====================================================== data print end =======================================================
//=============================================================================================================================
