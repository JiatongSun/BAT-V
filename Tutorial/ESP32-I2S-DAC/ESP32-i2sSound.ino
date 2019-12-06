/*****************************************************************************
  The MIT License (MIT)

  Copyright (c) 2016 by bbx10node@gmail.com

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
 **************************************************************************/
#include "driver/i2s.h"
#include "freertos/queue.h"
#include "wavspiffs.h"

#include "SPIFFS.h"

#define BUFFERSIZE 512 // 8266 version 512
#define LIGHTNINGLENGTH 85 // ms
#define DEBOUNCETIME 400   // ms

const char blast[] = "/Blast.wav"; // sound files uploaded to SPIFFS

//i2s configuration 
int i2s_num = 0; // i2s port number
i2s_config_t i2s_config = {
     .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
     .sample_rate = 36000,
     .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
     .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
     .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
     .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // high interrupt priority
     .dma_buf_count = 8,
     .dma_buf_len = 64   //Interrupt level 1
 };
    
i2s_pin_config_t pin_config = {
    .bck_io_num = 27, //this is BCK pin
    .ws_io_num = 14, // this is LRCK pin
    .data_out_num = 22, // this is DATA output pin
    .data_in_num = -1   //Not used
};

// Non-blocking I2S write for left and right 16-bit PCM
int  i2s_write_lr_nb(int16_t left, int16_t right){
  int sample = right & 0xFFFF;
  sample = sample << 16;
  sample |= left & 0xFFFF;
/* write sample data to I2S , set 3rd param = 0 for non-block*/
  return i2s_write_bytes((i2s_port_t)i2s_num, (const char *)&sample, sizeof(uint32_t), 0);
  // ESP-IDF call, docs say this will be deprecated but couldn't find i2s_write().
}

struct I2S_status_s {
  wavFILE_t wf;
  int16_t buffer[BUFFERSIZE];
  int bufferlen;
  int buffer_index;
  int playing;
} I2S_WAV;

void wav_setup()
{
  I2S_WAV.bufferlen = -1;
  I2S_WAV.buffer_index = 0;
  I2S_WAV.playing = false;
  i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL); // ESP-IDF call
  i2s_set_pin((i2s_port_t)i2s_num, &pin_config);     // ESP-IDF call
}

/**************************************************************************************/
// EVENT CHECKERS

bool timeToStart = false; 
bool wav_was_playing = false;
long starttime;

bool wav_justStarted() {
  if (timeToStart && !wav_was_playing) return true;
  else return false; 
}

bool wav_justFinished() {
  if (!wav_playing() && wav_was_playing) {
    wav_was_playing = false;
    return true;
  }
  else return false;  
}

bool touched(touch_pad_t channel) {
  uint16_t tvalue;
  static long touchstarttime;
  
  touch_pad_read(channel, &tvalue);
  if (tvalue < 400 && (millis()-touchstarttime)>DEBOUNCETIME) {
    touchstarttime = millis();
  Serial.print("touched: ");    Serial.println(tvalue);
    if (wav_playing()) {
      wav_stopPlaying();
      wav_justFinished();
    }
    return true;
  }
  else 
    return false;
}

bool timeToEndLight() {
  if (millis() > starttime + LIGHTNINGLENGTH) return true;
  else return false;
}

bool wav_playing()
{
  return I2S_WAV.playing;
}

/**************************************************************************************/
// SERVICES
void wav_stopPlaying()
{
  I2S_WAV.playing = false;
  wavClose(&I2S_WAV.wf);
}

void wav_loop()
{
  bool i2s_full = false;
  int rc;

  while (I2S_WAV.playing && !i2s_full) {
    while (I2S_WAV.buffer_index < I2S_WAV.bufferlen) {
      int16_t pcm = I2S_WAV.buffer[I2S_WAV.buffer_index];
      if (i2s_write_lr_nb(pcm, pcm)) { // send same data to both L & R channels
        I2S_WAV.buffer_index++;
      }
      else {
        i2s_full = true;
        break;
      }
    }
    yield(); //  feed WDT for 8266. Feed inside loop causes sound glitches
    if (i2s_full) break;

    rc = wavRead(&I2S_WAV.wf, I2S_WAV.buffer, sizeof(I2S_WAV.buffer));
    if (rc > 0) {
      I2S_WAV.bufferlen = rc / sizeof(I2S_WAV.buffer[0]);
      I2S_WAV.buffer_index = 0;
    }
    else {
      Serial.println(F("Stop playing"));
      wav_stopPlaying();
      break;
    }
  }
}

void wav_startPlayingFile(const char *wavfilename)
{
  wavProperties_t wProps;

  Serial.print("Playing File ");Serial.println(wavfilename);
  if ( wavOpen(wavfilename, &I2S_WAV.wf, &wProps) ) {
    Serial.print("wavOpen failed");
    return;
  }
  Serial.print(wProps.sampleRate); Serial.print(" bps ");
  Serial.print(wProps.bitsPerSample);
  i2s_set_clk((i2s_port_t)i2s_num, wProps.sampleRate*2,(i2s_bits_per_sample_t)wProps.bitsPerSample, (i2s_channel_t)wProps.numChannels);  // ESP-IDF call

  I2S_WAV.bufferlen = -1;
  I2S_WAV.buffer_index = 0;
  I2S_WAV.playing = true;
  wav_loop();
}

void sendQuiet() {
  i2s_zero_dma_buffer((i2s_port_t)i2s_num); // ESP-IDF call
}


void startProcesses() {
   digitalWrite(23,HIGH); // light LED
   starttime = millis();
   timeToStart = false;
   wav_was_playing = true;
}


/**************************************************************************************/

void setup()
{
  Serial.begin(115200); Serial.println();
  delay (100);
  Serial.println("Sound through I2S device and SPIFF file");

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS.begin() failed");
    return;
  }

  wav_setup();
  pinMode(23,OUTPUT);
  touch_pad_init();   // ESP-IDF call
}


void loop()
{ 

  // monitor events
  if (timeToStart)        wav_startPlayingFile(blast);
  if (wav_playing())      wav_loop();           // contiue sending data to sound buffer
  if (wav_justStarted())  startProcesses();
  if (wav_justFinished()) sendQuiet();          //make sure amp doesn't buzz
  if (touched((touch_pad_t)9))      timeToStart = true;   // flag to start sound if touch sensor
  if (timeToEndLight())   digitalWrite(23,LOW); // lightning LED off

  // add other things...
  
  delay(3);
}
