/***********************************************************************
 *  (c) 2022 MAYBstudio - MIT license 
 *  Mosquito Harmonizer
 *         
 *  Version 0.1       
 *  
 *        made possible by the samplerate code by Frank Boesing, and bat detector code by Frank DD4WH.
 *        to read more about the bat detector please visit: https://forum.pjrc.com/threads/38988-Bat-detector
 * 
 *  tested on Teensy 3.6 + Teensy audio board 
 *  + standard tiny electret MIC soldered to the MIC Input of the audio board 
 *  
 *  User adjustments - with buttons
 *  
 *  MIC-GAIN          33 + 34 -
 *  FREQUENCY         35 + 36 -
 *  SAMPLE RATE     37 + 38 -
 *  
 * Audio sample rate code - function setI2SFreq  
 * Copyright (c) 2016, Frank Bösing, f.boesing@gmx.de
 * 
 * Ultrasonic detection code - function spectrum (modified)
 * Copyright (c) 2016  Frank DD4WH 2016_11_01 - MIT license 
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice, development funding notice, and this permission
 * notice shall be included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <Bounce.h>
#include <Metro.h>

#include <ILI9341_t3.h>
#include "font_Arial.h"

#define VERSION     " v0.3"
//_________________________________________________________SCREEN STUFF

#define BACKLIGHT_PIN 0
#define TFT_DC      20 //orange
#define TFT_CS      21 //white
#define TFT_RST     32  // 255 = unused. connect to 3.3V //brown
#define TFT_MOSI     7 //yellow
#define TFT_SCLK    14 //blue
#define TFT_MISO    12 //grey

ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

//________________________________________________________AUDIO OBJECTS AND CORDS
// this audio comes from the codec by I2S2
AudioInputI2S            i2s_in; // MIC input           
AudioSynthWaveformSine   sine1; // local oscillator
AudioEffectMultiply      mult1; // multiply = mix
AudioAnalyzeFFT256       myFFT; // for spectrum display
//AudioAnalyzeFFT1024      myFFT; // for waterfall display
AudioOutputI2S           i2s_out; // headphone output
AudioSynthWaveform       waveform1;      //xy=188,240
AudioSynthWaveform       waveform2;      //xy=188,240
AudioEffectEnvelope      envelope1;      //xy=371,237
AudioEffectEnvelope      envelope2;      //xy=371,237
AudioMixer4              mixer1;         //xy=872,445

AudioConnection          patchCord1(waveform1, envelope1);
AudioConnection          patchCord2(waveform2, envelope2);
AudioConnection          patchCord3(i2s_in,0,myFFT,0);
AudioConnection          patchCord4(envelope1, 0, mixer1, 0);
AudioConnection          patchCord5(envelope2, 0, mixer1, 1);
AudioConnection          patchCord6(mixer1, 0, i2s_out, 0);
AudioConnection          patchCord7(mixer1, 0, i2s_out, 1);

AudioControlSGTL5000     sgtl5000_1;  

const int myInput = AUDIO_INPUT_MIC;

const int button1 = 28;
const int button2 = 29;
const int sensorPin = A12;
int sensorValue;
int highCutoff;
int lowCutoff;

#define SAMPLE_RATE_MIN               0
#define SAMPLE_RATE_44K               0
#define SAMPLE_RATE_48K               1
#define SAMPLE_RATE_88K               2
#define SAMPLE_RATE_96K               3
#define SAMPLE_RATE_176K              4
#define SAMPLE_RATE_192K              5
#define SAMPLE_RATE_MAX               5

int count_help = 0;

int idx1 = 0;
int idx2 = 0;
int16_t FFT_bin [128]; 


int peak[512];
int barm[512];
int prevState =0;

//_________________________________________________________audio sample rate options
//set the sample rate (96k is what we use)
//options: 44100, 48000, 88200, 96000, 176400, 192000

int sample_rate_real = 96000;
int sample_rate = SAMPLE_RATE_96K;

//__________________________________________________start detecting at this frequency 
int freq_real = 15000;
//___________________________________________start detecting with this MIC_GAIN in dB 
int8_t mic_gain = 42; 
//__________________________________________________________Local Oscilator frequency
int pitch1[] = {220, 131, 147, 175, 165, 98};
int pitch2[] = {131, 165, 196, 220, 165, 0, 98, 131, 123};
int count1 = 6;
int count2 = 9;

//int freq_LO = 7000;

typedef struct SR_Descriptor
{
    const int SR_n;
    const char* const f1;
    const char* const f2;
    const char* const f3;
    const char* const f4;
    const float32_t x_factor;
} SR_Desc;

// Text and position for the FFT spectrum display scale
const SR_Descriptor SR [SAMPLE_RATE_MAX + 1] =
{
    //   SR_n ,  f1, f2, f3, f4, x_factor = pixels per f1 kHz in spectrum display
    {  SAMPLE_RATE_44K,  "5", "10", "15", "20", 58.05}, // which means 58.05 pixels per 5 kHz
    {  SAMPLE_RATE_48K,  "5", "10", "15", "20", 53.33},
    {  SAMPLE_RATE_88K,  "10", "20", "30", "40", 58.05},
    {  SAMPLE_RATE_96K,  "10", "20", "30", "40", 53.33},
    {  SAMPLE_RATE_176K,  "20", "40", "60", "80", 58.05},
    {  SAMPLE_RATE_192K,  "20", "40", "60", "80", 53.33} // which means 53.33 pixels per 20kHz
};  

//______________________________________________________________________________________SETUP
void setup() {
  Serial.begin(115200);
  delay(200);
  
  // Audio connections require memory. 
  AudioMemory(100);
  
  // Enable the audio shield. select input. and enable output
  sgtl5000_1.enable();
  sgtl5000_1.inputSelect(myInput);
  sgtl5000_1.volume(0.4);
  sgtl5000_1.micGain (mic_gain);
  //sgtl5000_1.adcHighPassFilterDisable(); // does not help too much!
  waveform1.frequency(440);
  waveform1.begin(0.4, 220, WAVEFORM_SINE);

  envelope1.attack(50);
  envelope1.decay(50);
  envelope1.release(250);

  waveform2.frequency(440);
  waveform2.begin(0.4, 220, WAVEFORM_SINE);

  envelope2.attack(50);
  envelope2.decay(50);
  envelope2.release(250);
  
  pinMode( BACKLIGHT_PIN, OUTPUT );
  analogWrite( BACKLIGHT_PIN, 1023 );
  
  // start the screen up with an intro
  tft.begin();
  tft.setRotation( 3 );
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(14, 7);
  tft.setTextColor(ILI9341_ORANGE);
  tft.setFont(Arial_12);
  tft.print("Mosquito transposer"); //tft.print(VERSION);
  tft.setTextColor(ILI9341_WHITE);

//run the set frequency and set oscilator functions
  setI2SFreq(sample_rate_real);
//  set_freq_LO (freq_LO);
  prepare_spectrum_display();
  
}
//__________________________________________________________________________________END SETUP
//_________________________________________________________________________________START LOOP
void loop() {
  spectrum();
  sensorValue = analogRead(sensorPin);
  lowCutoff = map(sensorValue,0,1023,10,50);
  Serial.println(lowCutoff);
}
//_____________________________________________________________SPECTRUM
 void spectrum() { // spectrum analyser code by rheslip - modified
  //potValue = analogRead(sensorPin);
  if (myFFT.available()) {
    int scale;
    scale = 5;
  for (int16_t x = 2; x < 128; x++) {
     int newValue = myFFT.output[x];
     FFT_bin[x] = abs(newValue); 
     int bar = (FFT_bin[x] * scale);
     if (bar >175) bar=175;
     // this is a very simple first order IIR filter to smooth the reaction of the bars
     bar = 0.05 * bar + 0.95 * barm[x]; 
     tft.drawPixel(x*2+10, 210-barm[x], ILI9341_BLACK);
     tft.drawPixel(x*2+10, 210-bar, ILI9341_WHITE);
     
     barm[x] = bar;
  }
  int batDetected = 0;
  //35 and 43 are the db cutoffs for 16
  highCutoff = lowCutoff +8;
  for(int i=lowCutoff;i<highCutoff;i++){
    if(batDetected == 0){
      //THIS VALUE HERE IS THE CUTOFF
      if (FFT_bin[i]>70){
        batDetected = 1;
      }
    }
  }
  int sign_x = 148; 
  int sign_y = 57;
  //if mosquito detected and state has changed (if we switched up)
  if(batDetected == 1 and prevState != batDetected){
    //sgtl5000_1.volume(0.8);
    //pick new pitch value from list
    if (idx1<(count1-1)){
      idx1 += 1;
    }
    else{
      idx1=0;
    }
    if (idx2<(count2-1)){
      idx2 += 1;
    }
    else{
      idx2=0;
    }
    waveform1.frequency(pitch1[idx1]);
    envelope1.noteOn();
    waveform2.frequency(pitch2[idx2]);
    envelope2.noteOn();
    //Serial.println(potValue);
    prevState = 1;
  }
  //if we switched down
  else if(batDetected == 0 and prevState != batDetected){
    prevState = 0;
    envelope1.noteOff();
    envelope2.noteOff();
  }
  }   
  } //end if
//___________________________________________________________________________________END LOOP

//_______________________________________________________SET SAMPLE RATE (ON INPUT DEVICE)
void setI2SFreq (int freq) {
  typedef struct {
    uint8_t mult;
    uint16_t div;
  } tmclk;

  const int numfreqs = 14;
  const int samplefreqs[numfreqs] = { 8000, 11025, 16000, 22050, 32000, 44100, (int)44117.64706 , 48000, 88200, (int)44117.64706 * 2, 96000, 176400, (int)44117.64706 * 4, 192000};
  
  #if (F_PLL==16000000)
    const tmclk clkArr[numfreqs] = {{16, 125}, {148, 839}, {32, 125}, {145, 411}, {64, 125}, {151, 214}, {12, 17}, {96, 125}, {151, 107}, {24, 17}, {192, 125}, {127, 45}, {48, 17}, {255, 83} };
  #elif (F_PLL==72000000)
    const tmclk clkArr[numfreqs] = {{32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {128, 1125}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375}, {249, 397}, {32, 51}, {185, 271} };
  #elif (F_PLL==96000000)
    const tmclk clkArr[numfreqs] = {{8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {32, 375}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125}, {151, 321}, {8, 17}, {64, 125} };
  #elif (F_PLL==120000000)
    const tmclk clkArr[numfreqs] = {{32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {128, 1875}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625}, {178, 473}, {32, 85}, {145, 354} };
  #elif (F_PLL==144000000)
    const tmclk clkArr[numfreqs] = {{16, 1125}, {49, 2500}, {32, 1125}, {49, 1250}, {64, 1125}, {49, 625}, {4, 51}, {32, 375}, {98, 625}, {8, 51}, {64, 375}, {196, 625}, {16, 51}, {128, 375} };
  #elif (F_PLL==168000000)
    const tmclk clkArr[numfreqs] = {{32, 2625}, {21, 1250}, {64, 2625}, {21, 625}, {128, 2625}, {42, 625}, {8, 119}, {64, 875}, {84, 625}, {16, 119}, {128, 875}, {168, 625}, {32, 119}, {189, 646} };
  #elif (F_PLL==180000000)
    const tmclk clkArr[numfreqs] = {{46, 4043}, {49, 3125}, {73, 3208}, {98, 3125}, {183, 4021}, {196, 3125}, {16, 255}, {128, 1875}, {107, 853}, {32, 255}, {219, 1604}, {214, 853}, {64, 255}, {219, 802} };
  #elif (F_PLL==192000000)
    const tmclk clkArr[numfreqs] = {{4, 375}, {37, 2517}, {8, 375}, {73, 2483}, {16, 375}, {147, 2500}, {1, 17}, {8, 125}, {147, 1250}, {2, 17}, {16, 125}, {147, 625}, {4, 17}, {32, 125} };
  #elif (F_PLL==216000000)
    const tmclk clkArr[numfreqs] = {{32, 3375}, {49, 3750}, {64, 3375}, {49, 1875}, {128, 3375}, {98, 1875}, {8, 153}, {64, 1125}, {196, 1875}, {16, 153}, {128, 1125}, {226, 1081}, {32, 153}, {147, 646} };
  #elif (F_PLL==240000000)
    const tmclk clkArr[numfreqs] = {{16, 1875}, {29, 2466}, {32, 1875}, {89, 3784}, {64, 1875}, {147, 3125}, {4, 85}, {32, 625}, {205, 2179}, {8, 85}, {64, 625}, {89, 473}, {16, 85}, {128, 625} };
  #endif
  
    for (int f = 0; f < numfreqs; f++) {
      if ( freq == samplefreqs[f] ) {
        while (I2S0_MCR & I2S_MCR_DUF) ;
        I2S0_MDR = I2S_MDR_FRACT((clkArr[f].mult - 1)) | I2S_MDR_DIVIDE((clkArr[f].div - 1));
        return;
    }
  }
}
//______________________________________________________END SET SAMPLE RATE (ON INPUT DEVICE)
//_____________________________________________PREPARE SPECTRUM DISPLAY
void prepare_spectrum_display() {
    int base_y = 211; 
    int b_x = 10;
    int x_f = SR[sample_rate].x_factor;
    tft.fillRect(0,base_y,320,240 - base_y,ILI9341_BLACK); 
    tft.drawFastHLine(b_x, base_y + 2, 256, ILI9341_MAROON);  
    tft.drawFastHLine(b_x, base_y + 3, 256, ILI9341_MAROON);  
    // vertical lines
    tft.drawFastVLine(b_x - 4, base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine(b_x - 3, base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f + 1 + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 2 + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 2 + 1 + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 3 + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 3 + 1 + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 4 + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 4 + 1 + b_x,  base_y + 1, 10, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 0.5 + b_x,  base_y + 1, 6, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 1.5 + b_x,  base_y + 1, 6, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 2.5 + b_x,  base_y + 1, 6, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 3.5 + b_x,  base_y + 1, 6, ILI9341_YELLOW);  
    tft.drawFastVLine( x_f * 4.5 + b_x,  base_y + 1, 6, ILI9341_YELLOW);  
    // text
    tft.setTextColor(ILI9341_WHITE);
    tft.setFont(Arial_9);
    int text_y_offset = 16;
    int text_x_offset = - 5;
    tft.setCursor (b_x + text_x_offset + 256, base_y + text_y_offset);
    tft.print("kHz");
    // zero
    tft.setCursor (b_x + text_x_offset, base_y + text_y_offset);
    tft.print("0");
    tft.setCursor (b_x + x_f + text_x_offset, base_y + text_y_offset);
    tft.print(SR[sample_rate].f1);
    tft.setCursor (b_x + x_f * 2 + text_x_offset, base_y + text_y_offset);
    tft.print(SR[sample_rate].f2);
    tft.setCursor (b_x + x_f *3 + text_x_offset, base_y + text_y_offset);
    tft.print(SR[sample_rate].f3);
    tft.setCursor (b_x + x_f *4 + text_x_offset, base_y + text_y_offset);
    tft.print(SR[sample_rate].f4);
    tft.setFont(Arial_14);
} // END prepare_spectrum_display
