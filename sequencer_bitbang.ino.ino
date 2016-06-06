#include <avr/pgmspace.h>
#include "NotesTable.h"
#include "MyLedControl.h"
#include <EEPROM.h>
#include <Wire.h>

#if defined(__AVR_ATmega328P__)
// Timer2 is the same on the mega328 and mega168
#define __AVR_ATmega168__
#endif
//#define profiling    // define this if profiling information in serial monitor shall be outputted

/*
  Badass bass synth
  Current manual:
  Keypad
   1  2  3  4   Act Load
   5  6  7  8   Acc FX
   9 10 11 12   Gld Play
  13 14 15 16   Snd Trans
  Stopped:
  Play note on keys 5-16 (C is left bottom)
  1 Oct--
  2 Oct++
  3 Step back
  4 Step forward
  Playing:
  Play note on keys 5-16 (C is left bottom) overriding the note in the current pattern
  1 Oct--
  2 Oct++
  Act  +key 1-16: Mute/Unmute step
  Acc  +key 1-16: Set/unset accent on step
  Gld  +key 1-16: Set/unset glide on step (glide to that step)
  Snd  +key 1-16: Select sound 1-16
  Load +key 1-16: Load pattern 1-16
  FX   +key 1-16: Apply FX 1-16 on current step
  Trans+key 1-16: Transpose by half-tones (bottom left=no transpose, top right=transpose by 15 halt-tones)
  Play +key 1-16: When playing: Jump to step 1-16 and play from there. When play is released, original step sequence is restored
  Act+Load +key 1-16: Store current pattern to pattern memory 1-16
  Acc+FX   +key 1-16: Set pattern length
  Snd+Trans+key 1-16: Set BPM (1/2:-/+1  5/6:-/+5  9/10:-/+10  13:set to 90  14:set to 120  15:set to 140  16:set to 180)
  Act+Acc+Gld: Clear current pattern
  Sounds:
  1 saw filter1   2 square filter1    3 sine filter1    4 noise filter1
  5 saw filter2   6 square filter2    7 sine filter2    8 noise filter2
  9 pulse (LFO)  10 multi square     11 waveshaper
  Effects:
  1 no effect     2 1/8 octave shift  3 1/32 trigger
  5 major         6 minor             7 maj7            8 diminished
  9 compressor   10 overdrive        11 sine mod       12 bit crush
*/

// min and max range values for resonance and cutoff respectively
// values are found by experimentation
const int soundRange[16][4]={
  {718,200,  70,1020},
  {670,260,  80,670},
  {670,0,    0,1020},
  {670,270,  80,1020},
  {1020,250,  1020,0},
  {1023,0,    1023,0},
  {1020,600,  1020,0},
  {1023,0,    1023,0},
  {0,1023,    0,1023},
  {128,1023,  0,1023},
  {0,1023,    0,1023},
  {0,1023,    0,1023},
  {0,1023,    1023,740},
  {0,1023,    0,1023},
  {0,1023,    0,1023},
  {0,1023,    0,1023}
};

static byte i;
static long j;
static uint16_t freq;
static bool act;
static uint16_t cnt;
static int16_t volSub=0;        // 0=full volume, 255=silence

static int16_t o;              // output temp
static int16_t dist;
static int16_t memO;
static int16_t lastO;          // used for LPF

static int16_t cutoff=0;
static int16_t resonance=0;
static int16_t cutRead=0;
static int16_t resRead=0;
static byte phasePosSwitch;
static int falloffSpeed=0;
static long targetNote=0;
static long targetNoteStep=0;
static byte runningStep=0;
static byte step=0;
static byte stepOffset=0;
#ifdef profiling
static bool playing=true;
#else
static bool playing=false;
#endif
static bool stepOffsetSelected=false;
static uint8_t selectedPattern=0;
static bool anyNotePressed=false;
static byte transpose=0;
static byte transposeTemp=0;
#define PLAYKEY_STOPPED 0
#define PLAYKEY_STARTED 1
#define PLAYKEY_PLAYING 2
#define PLAYKEY_PRESSED_PLAY 3
#define PLAYKEY_STEP_SELECTED 4
static uint8_t playKeyStatus=PLAYKEY_STOPPED;
static int8_t octave=0;
#define MIN_OCTAVE 0
#define MAX_OCTAVE 3
static byte selectedSound=0;
static long notes[16]={12,12,12,15,12,24,19,17,12,19,17,19,36,12,24,27};
//long notes[16]={0,0,0,3,0,4,0,5,0,7,5,7,0,6,0,12};
//static long notes[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static bool activ[16]={true,true,false,true,true,true,false,true,true,true,true,true,true,false,true,true};
//static bool activ[16]={true,true,true,true,true,true,true,true,true,true,true,true,true,true,true,true};
static bool glide[16]={false,false,false,false,false,true,false,true,false,false,false,true,true,true,true,false};
//static bool glide[16]={false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
//static bool accent[16]={true,false,false,true,false,false,true,false,false,true,false,false,true};
static bool accent[16]={false,false,false,false,false,false,false,false,false,false,false,false,false};
static uint8_t effect[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static uint8_t currentEffect;
static unsigned long lastCall;
static unsigned long now;
static int micron;
static int lastMicron;
static byte lastArpeggioStep=0;
static int bpm=140;
static byte patternLength=16;
#define MINBPM 60
#define MAXBPM 240
#define EEPROM_BPM 768
static long barLength;
static unsigned long lastSync;
static unsigned long syncLength=0;
static unsigned long lastSyncLength=0;
static bool alreadySynced=false;
static bool alreadySyncedStep=false;
static int lastBPM[4]={140,140,140,140};
static byte lastBPMpointer=0;
static uint32_t lfoPos=0;
static int lfoOffset=0;
static uint16_t freqBroad[8];
static uint16_t cntBroad[8];
static uint8_t numberVoicesBroad;
static uint8_t volBroadStandard;
static uint8_t volBroadClip;
static uint8_t interruptWaiter;

static uint8_t songLength=0;            // 0=1 pattern playing, 1=2 patterns playing,etc.
static uint8_t songPatterns[16];
static uint8_t currentSongStep=0;
static bool patternButtonPressed=false;
static uint8_t blinker=0;

/*static int displayMode=0;
#define DISP_STD 0
#define DISP_OSC 1
*/
#define OCT1 1
#define OCT2 2
#define OCT3 3
#define OCT4 4
#define NOTE_C 5
#define NOTE_CIS 6
#define NOTE_D 7
#define NOTE_DIS 8
#define NOTE_E 9
#define NOTE_F 10
#define NOTE_FIS 11
#define NOTE_G 12
#define NOTE_GIS 13
#define NOTE_A 14
#define NOTE_AIS 15
#define NOTE_H 16
#define MODE_ACTIVE 17
#define MODE_ACCENT 18
#define MODE_GLIDE 19
#define MODE_SOUND 20
#define MODE_WRITEPATTERN 21
#define MODE_LOADPATTERN 22
#define MODE_CLEAR 23
#define MODE_HUNDRED 24    // 3 patterns
#define MODE_NUMBER 27     // 10 patterns
#define MODE_NOTE 37       // 4 patterns
#define MODE_JUMP 41
#define MODE_LENGTH 42
#define MODE_EFFECT 43
#define MODE_TRANSPOSE 44
#define MODE_SONG 45
const byte infoDisp[46*4] PROGMEM =
  {B01000100,B10101010,B10101010,B01000100,      // octave 0
   B01000100,B10101100,B10100100,B01000100,      // octave 1
   B01001100,B10100010,B10100100,B01001110,      // octave 2
   B01001100,B10100110,B10100110,B01001100,      // octave 3
   B01001010,B10101010,B10101110,B01000010,      // octave 4
   B01100000,B10000000,B10000000,B01100000,      // C
   B01100100,B10001110,B10000100,B01100000,      // C#
   B11000000,B10100000,B10100000,B11000000,      // D
   B11000100,B10101110,B10100100,B11000000,      // D#
   B11100000,B11000000,B10000000,B11100000,      // E
   B11100000,B10000000,B11000000,B10000000,      // F
   B11100100,B10001110,B11000100,B10000000,      // F#
   B01100000,B10000000,B10100000,B01100000,      // G
   B01100100,B10001110,B10100100,B01100000,      // G#
   B01000000,B10100000,B11100000,B10100000,      // A
   B01000100,B10101110,B11100100,B10100000,      // A#
   B10100000,B11100000,B10100000,B10100000,      // H
   B01001001,B10101101,B10101011,B01001001,      // On=Active
   B01000110,B10101000,B11101000,B10100110,      // Ac=Accent
   B01101000,B10001000,B10101000,B01101110,      // Gl=Glide
   B01101001,B10001101,B01101011,B11001001,      // Sn=Sound select
   B10010110,B10010101,B11110110,B11110101,      // Wr=Write pattern
   B10001100,B10001010,B10001010,B11101100,      // Ld=Load pattern
   B01010110,B10010101,B10010110,B01011101,      // Clr=Clear pattern
   B00000000,B00000000,B00000000,B00000000,      //hundred:0
   B01000000,B11000000,B01000000,B01000000,      //hundred:1
   B11000000,B01000000,B10000000,B11000000,      //hundred:2
   B00010000,B00101000,B00101000,B00010000,      //0
   B00001000,B00011000,B00001000,B00001000,      //1
   B00110000,B00001000,B00010000,B00111000,      //2
   B00110000,B00011000,B00001000,B00110000,      //3
   B00101000,B00111000,B00001000,B00001000,      //4
   B00111000,B00110000,B00001000,B00110000,      //5
   B00010000,B00110000,B00101000,B00010000,      //6
   B00111000,B00001000,B00010000,B00010000,      //7
   B00010000,B00111000,B00111000,B00010000,      //8
   B00010000,B00101000,B00011000,B00001000,      //9
   B00000000,B10001000,B11001100,B10001000,      // playing anim stage 1
   B00000000,B01000100,B01100110,B01000100,      // playing anim stage 2
   B00000000,B00100010,B00110011,B00100010,      // playing anim stage 3
   B00000000,B00010001,B10011001,B00010001,      // playing anim stage 4
   B00000000,B11000010,B11011111,B11000010,      // jump to step
   B00000000,B10000001,B11111111,B10000001,      // pattern length
   B11101001,B10000110,B11000110,B10001001,      // FX
   B01011001,B01011001,B11010011,B11010011,      // Transpose
   B01100110,B10001001,B01101001,B11000110,      // SO=Song
 };

static LedControl lc=LedControl();

static const byte ROWS = 6; // Four rows
static const byte COLS = 4; // Three columns
static const byte rowPins[ROWS] = { 1,2,3,4,13,0 };
static const byte colPins[COLS] = { 9,8,7,6 }; 
static byte keypadState[ROWS][COLS];
static bool keyLocked[ROWS][COLS];
static byte dispBuff[8];
const uint8_t sinetable[256] PROGMEM ={127,130,133,136,139,142,145,148,151,154,157,160,163,166,169,172,
                          175,178,181,184,186,189,192,194,197,200,202,205,207,209,212,214,
                          216,218,221,223,225,227,229,230,232,234,235,237,239,240,241,243,
                          244,245,246,247,248,249,250,250,251,252,252,253,253,253,253,253,
                          254,253,253,253,253,253,252,252,251,250,250,249,248,247,246,245,
                          244,243,241,240,239,237,235,234,232,230,229,227,225,223,221,218,
                          216,214,212,209,207,205,202,200,197,194,192,189,186,184,181,178,
                          175,172,169,166,163,160,157,154,151,148,145,142,139,136,133,130,
                          127,123,120,117,114,111,108,105,102,99,96,93,90,87,84,81,
                          78,75,72,69,67,64,61,59,56,53,51,48,46,44,41,39,
                          37,35,32,30,28,26,24,23,21,19,18,16,14,13,12,10,
                          9,8,7,6,5,4,3,3,2,1,1,0,0,0,0,0,
                          0,0,0,0,0,0,1,1,2,3,3,4,5,6,7,8,
                          9,10,12,13,14,16,18,19,21,23,24,26,28,30,32,35,
                          37,39,41,44,46,48,51,53,56,59,61,64,67,69,72,75,
                          78,81,84,87,90,93,96,99,102,105,108,111,114,117,120,123};
const uint8_t noisetable[256] PROGMEM={232,175,188,102,142,3,70,116,17,139,22,155,54,118,84,22,
                          251,228,160,233,30,32,6,125,16,216,122,189,95,232,135,205,
                          181,97,90,80,76,170,0,4,123,183,46,163,185,40,47,208,
                          145,67,219,87,74,140,213,10,72,51,29,142,230,63,204,123};
const uint8_t bitcrushtable[256] PROGMEM={
                          0,0,16,16,16,16,16,16,16,16,16,16,16,16,16,16,
                          32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,32,
                          48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,48,
                          64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,64,
                          80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,80,
                          96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,96,
                          112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,112,
                          128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,128,
                          144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,144,
                          160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,160,
                          176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,176,
                          192,192,192,192,192,192,192,192,192,192,192,192,192,192,192,192,
                          208,208,208,208,208,208,208,208,208,208,208,208,208,208,208,208,
                          224,224,224,224,224,224,224,224,224,224,224,224,224,224,224,224,
                          240,240,240,240,240,240,240,240,240,240,240,240,240,240,255,255
                          };
const uint8_t compressortable[256] PROGMEM={
                          0,1,3,5,7,9,10,12,14,16,18,19,21,23,25,27,
                          28,30,32,34,36,37,39,41,43,45,46,48,50,52,54,55,
                          57,59,61,63,64,66,68,70,72,73,75,77,79,81,82,84,
                          86,88,90,91,93,95,97,99,100,102,104,106,108,109,111,113,
                          115,117,118,120,122,124,126,127,129,131,133,135,136,138,140,142,
                          144,145,147,149,151,153,154,156,158,160,162,163,165,167,169,171,
                          172,174,176,178,180,181,183,185,187,189,190,192,194,196,198,199,
                          201,203,205,207,208,210,212,214,216,217,219,221,223,225,226,228,
                          228,228,228,228,228,229,229,229,229,229,230,230,230,230,230,231,
                          231,231,231,231,232,232,232,232,233,233,233,233,233,234,234,234,
                          234,234,235,235,235,235,235,236,236,236,236,237,237,237,237,237,
                          238,238,238,238,238,239,239,239,239,239,240,240,240,240,241,241,
                          241,241,241,242,242,242,242,242,243,243,243,243,243,244,244,244,
                          244,245,245,245,245,245,246,246,246,246,246,247,247,247,247,247,
                          248,248,248,248,249,249,249,249,249,250,250,250,250,250,251,251,
                          251,251,251,252,252,252,252,252,253,253,253,253,254,254,254,255
                          };
const uint8_t overdrivetable[256] PROGMEM={
//uint8_t overdrivetable[256] ={
                          0,3,6,9,12,15,18,21,24,27,30,33,36,39,42,45,
                          48,51,54,57,60,63,66,69,72,75,78,81,84,87,90,
                          93,96,99,102,105,108,111,114,117,120,123,126,129,132,135,
                          138,141,144,147,150,153,156,159,162,165,168,171,174,177,180,
                          182,184,186,188,190,192,194,196,198,200,202,204,206,208,210,
                          212,214,216,218,220,221,222,223,224,225,226,227,228,229,230,
                          231,232,233,234,235,236,237,238,239,240,240,241,241,242,242,
                          243,243,243,244,244,244,244,245,245,245,245,245,245,246,246,
                          246,246,246,246,247,247,247,247,247,247,247,247,247,247,247,
                          248,248,248,248,248,248,248,248,248,248,248,248,249,249,249,
                          249,249,249,249,249,249,249,249,249,249,250,250,250,250,250,
                          250,250,250,250,250,250,250,250,250,250,250,251,251,251,251,
                          251,251,251,251,251,251,251,251,251,251,251,252,252,252,252,
                          252,252,252,252,252,252,252,252,252,253,253,253,253,253,253,
                          253,253,253,253,253,253,254,254,254,254,254,254,254,254,254,
                          254,254,254,254,254,255,255,255,255,255,255,255,255,255,255
                          };

const int8_t arpeggio[4][4]={{0,4,7,12},{0,3,7,12},{0,4,7,11},{0,3,6,9}};
static byte outBuffer[256];
static byte outBufferPointer=0;
static uint8_t osciBuffer[128];
#define PI2 6.283185
#define syncThreshold 350

#define OLED_I2C_ADDRESS   0x3C
#define OLED_CONTROL_BYTE_CMD_SINGLE	0x80
#define OLED_CONTROL_BYTE_CMD_STREAM	0x00
#define OLED_CONTROL_BYTE_DATA_STREAM	0x40
#define OLED_CONTROL_BYTE_DATA_SINGLE   0xc0
#define OLED_CMD_SET_CONTRAST			0x81	// follow with 0x7F
#define OLED_CMD_DISPLAY_RAM			0xA4
#define OLED_CMD_DISPLAY_ALLON			0xA5
#define OLED_CMD_DISPLAY_NORMAL			0xA6
#define OLED_CMD_DISPLAY_INVERTED 		0xA7
#define OLED_CMD_DISPLAY_OFF			0xAE
#define OLED_CMD_DISPLAY_ON				0xAF
#define OLED_CMD_SET_MEMORY_ADDR_MODE	0x20	// follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE		0x21	// can be used only in HORZ/VERT mode - follow with 0x00 + 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE			0x22	// can be used only in HORZ/VERT mode - follow with 0x00 + 0x07 = PAGE7
#define OLED_CMD_SET_COL_NIBBLE_LO            0x00
#define OLED_CMD_SET_COL_NIBBLE_HI            0x10
#define OLED_CMD_SET_PAGE_START                0xb0
#define OLED_CMD_SET_DISPLAY_START_LINE	0x40
#define OLED_CMD_SET_SEGMENT_REMAP		0xA1	
#define OLED_CMD_SET_MUX_RATIO			0xA8	// follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE		0xC8	
#define OLED_CMD_SET_DISPLAY_OFFSET		0xD3	// follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP		0xDA	// follow with 0x12
#define OLED_CMD_NOP 					0xE3	// NOP
#define OLED_CMD_SET_DISPLAY_CLK_DIV	0xD5	// follow with 0x80
#define OLED_CMD_SET_PRECHARGE			0xD9	// follow with 0x22
#define OLED_CMD_SET_VCOMH_DESELCT		0xDB	// follow with 0x30
#define OLED_CMD_SET_CHARGE_PUMP		0x8D	// follow with 0x14

// profiling stuff
#ifdef profiling
static unsigned long timePerStepper;
static int numberStepper;
static unsigned long lastProfile;
#endif

void setup(){
  lastCall=0;
  
  cli();//disable interrupts
 
//set timer1 interrupt at 15,625khz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A=15;    // (16.000.000/64/15625)-1 = 16-1
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS11 bits for 64 prescaler
  TCCR1B |= (1 << CS11) | (1 << CS10);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A) | (1<<OCIE2A);

  /****Set timer0 for 8-bit fast PWM output ****/
  pinMode(5, OUTPUT); // Make timer’s PWM pin an output
  TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM01) | _BV(WGM00); 
  TCCR0B = _BV(CS00); 

  /****Set timer2 for sequencer loop ****/
  TCCR2A = (1<<WGM21) | (1<<CS20) | (1<<CS21) | (1<<CS22);
  TCNT2=0;
  OCR2A=250;            // about 1000hz
  TCCR2B|=(1<<WGM12);
  TIMSK2|=(1<<OCIE2A);

  sei();//enable interrupts
  
  // initialize keypad handling
  // set rows to HIGH Z
  for (int i = 0; i < ROWS; i++) {
    pinMode (rowPins[i], INPUT);
    digitalWrite (rowPins[i], LOW);
  }
  // set cols to input with pullups
  for (int j = 0; j < COLS; j++) {
    pinMode (colPins[j], INPUT);
    digitalWrite (colPins[j], HIGH);
  }
  for (int i = 0; i < ROWS; i++) {
    for (int j = 0; j < COLS; j++) {
      keypadState[i][j] = 0;
      keyLocked[i][j]=false;
    }
  }  

  bpm=EEPROM.read(EEPROM_BPM);

  lastSync=millis();
  
  oled_init();
  delay(250);				// **** what for? ****
  
  #ifdef profiling
  Serial.begin(115200);
  #endif
}

int restrictValue(int val, int min, int max) {
  long temp=max-min;
  temp*=(val<0?0:(val>1023?1023:val));        // restrict to valid range
  temp/=1023;
  return temp+min;
}

/*
******* additional filter code 1)*********
// a low pass filter based on the one from MeeBlip (http://meeblip.noisepages.com)
// a += f*((in-a)+ q*(a-b)
// b+= f* (a-b)
//  outValue>>=3;
// started at 4700
// 4686
******* additional filter code 2)*********
von meeblip:
;----------------------------------------------------------------------------
 ; Digitally Controlled Filter
 ;
 ; A 2-pole resonant low pass filter:
 ;
 ; a += f * ((in - a) + q * 4 * (a - b))
 ; b += f * (a - b)
 ;
 ; f = (1-F)/2+Q_offset
 ; q = Q-f = Q-(1-F)/2+Q_offset
 ;
 ; F = LPF (cutoff)
 ; Q = RESONANCE
 ; q = SCALED_RESONANCE
 ; b => output
 ;
 ; Input 16-Bit signed HDAC:LDAC (r17:r16), already scaled to minimize clipping (reduced to 25% of full code).
*/
ISR(TIMER1_COMPA_vect){//timer 1 interrupt
  cnt+=freq;              // timer will automatically wraparound at 65535
  
  switch(selectedSound) {
    case 0:
      o=127-(cnt>>8);     // sawtooth wave
      dist=o-lastO;                // LPF with resonance
      memO+=dist*cutoff/256;
      lastO+=memO+dist*resonance/256;
      lastO=(lastO<-128?-128:(lastO>127?127:lastO));      // constrain the value to -128..127
      memO=(memO<-128?-128:(memO>127?127:memO));          // constrain the value to -128..127
      o=lastO-volSub+128;
      OCR0B=(o<0?0:o);
      break;
    case 1:
      o=(cnt<32768?100:-100);    // square wave
      dist=o-lastO;                // LPF with resonance
      memO+=dist*cutoff/256;
      lastO+=memO+dist*resonance/256;
      lastO=(lastO<-128?-128:(lastO>127?127:lastO));      // constrain the value to -128..127
      memO=(memO<-128?-128:(memO>127?127:memO));          // constrain the value to -128..127
      o=lastO-volSub+128;
      OCR0B=(o<0?0:o);
      break;
    case 2:
      o=pgm_read_byte(&sinetable[cnt>>8])-128;    // sine wave
      dist=o-lastO;                // LPF with resonance
      memO+=dist*cutoff/256;
      lastO+=memO+dist*resonance/256;
      lastO=(lastO<-128?-128:(lastO>127?127:lastO));      // constrain the value to -128..127
      memO=(memO<-128?-128:(memO>127?127:memO));          // constrain the value to -128..127
      o=lastO-volSub+128;
      OCR0B=(o<0?0:o);
      break;
    case 3:
      o=pgm_read_byte(&noisetable[cnt>>10])-128;    // noise wave
      dist=o-lastO;                // LPF with resonance
      memO+=dist*cutoff/256;
      lastO+=memO+dist*resonance/256;
      lastO=(lastO<-128?-128:(lastO>127?127:lastO));      // constrain the value to -128..127
      memO=(memO<-128?-128:(memO>127?127:memO));          // constrain the value to -128..127
      o=lastO-volSub+128;
      OCR0B=(o<0?0:o);
      break;
    case 4:
      o=127-(cnt>>8);     // sawtooth wave
      lastO=o+outBuffer[(outBufferPointer-cutoff)&255]*(resonance-512)/512;
      o=lastO-volSub+128;
      o=(o<0?0:o);
      outBuffer[outBufferPointer++]=o;
      OCR0B=o;
      break;
    case 5:
      o=(cnt<32768?100:-100);    // square wave
      lastO=o+outBuffer[(outBufferPointer-cutoff)&255]*(resonance-512)/512;
      o=lastO-volSub+128;
      o=(o<0?0:o);
      outBuffer[outBufferPointer++]=o;
      OCR0B=o;
      break;
    case 6:
      o=pgm_read_byte(&sinetable[cnt>>8])-128;    // sine wave
      lastO=o+outBuffer[(outBufferPointer-cutoff)&255]*(resonance-512)/512;
      o=lastO-volSub+128;
      o=(o<0?0:o);
      outBuffer[outBufferPointer++]=o;
      OCR0B=o;
      break;
    case 7:
      o=pgm_read_byte(&noisetable[cnt>>10])-128;    // noise wave
      lastO=o+outBuffer[(outBufferPointer-cutoff)&255]*(resonance-512)/512;
      o=lastO-volSub+128;
      o=(o<0?0:o);
      outBuffer[outBufferPointer++]=o;
      OCR0B=o;
      break;
    case 8:
      if ((cnt>>8)>phasePosSwitch)
        lastO=255;
      else
        lastO=0;
      o=lastO-volSub;
      OCR0B=(o<0?0:o);
      break;
    case 9: {
      uint8_t temp=0;
      for (int8_t ii=numberVoicesBroad-1; ii>0; ii--) {
        cntBroad[ii]+=freqBroad[ii];
        if (cntBroad[ii]<32768) {
          temp+=volBroadStandard;
        }
      }
      cntBroad[0]+=freqBroad[0];
      if (cntBroad[0]<32768) {
        temp+=volBroadClip;
      }
      o=temp-volSub;
      OCR0B=(o<0?0:o);}
      break;
    case 10:
      o=pgm_read_byte(&sinetable[((cnt>>9)*resonance)>>8])-128;    // sine wave
      if (cutoff<256) {
        if (o>=0)
          lastO=o+cutoff;
        else
          lastO=o-cutoff;
      } else {
        if (o>=0)
          lastO=127-o+(511-cutoff);
        else
          lastO=-o-(511-cutoff);
      }
      lastO=(lastO<-128?-128:(lastO>127?127:lastO));
      o=lastO-volSub+128;
      OCR0B=(o<0?0:o);
      break;
    case 12:
      o=127-(cnt>>8);     // sawtooth wave
/*      dist=o-lastO;                // LPF with resonance
      memO+=dist*cutoff/256;
      lastO+=memO+dist*resonance/256;
      lastO=(lastO<-128?-128:(lastO>127?127:lastO));      // constrain the value to -128..127
      memO=(memO<-128?-128:(memO>127?127:memO));          // constrain the value to -128..127
 ; Digitally Controlled Filter
 ;
 ; A 2-pole resonant low pass filter:
 ;
 ; a += f * ((in - a) + q * 4 * (a - b))
 ; b += f * (a - b)
 ;
 ; f = (1-F)/2+Q_offset
 ; q = Q-f = Q-(1-F)/2+Q_offset
 ;
 ; F = LPF (cutoff)
 ; Q = RESONANCE
 ; q = SCALED_RESONANCE
 ; b => output
 ;
 ; Input 16-Bit signed HDAC:LDAC (r17:r16), already scaled to minimize clipping (reduced to 25% of full code).
      */
      resonance=0;//********************
      lastO+=cutoff*((o-(lastO/256))+(resonance*(lastO-memO))/256)/4;
      memO+=cutoff*((lastO-memO)/256)/4;

/*      lastO+=cutoff*(o-(lastO/256))/4;
      memO+=cutoff*((lastO-memO)/256)/4;
  */    
/*      lastO+=cutoff*((o-(lastO/8192))+(resonance*((lastO-memO)/4096)));
      memO+=cutoff*((lastO-memO)/8192);*/
      
  //    lastO=(lastO<-128?-128:(lastO>127?127:lastO));      // constrain the value to -128..127
//      memO=(memO<-128?-128:(memO>127?127:memO));          // constrain the value to -128..127
    //  o=memO-volSub+128;
    
    
    
    
    
      o=(memO/512)-volSub+128;
//      o=lastO-volSub+128;
      OCR0B=(o<0?0:o);
      break;
  }
  switch (currentEffect) {
    case 8:
      OCR0B=pgm_read_byte(&compressortable[OCR0B]);
      break;
    case 9:
      OCR0B=pgm_read_byte(&overdrivetable[OCR0B]);
      break;
    case 10:
      OCR0B=pgm_read_byte(&sinetable[OCR0B]);
      break;
    case 11:
      OCR0B=OCR0B&192;
      break;
  }
  osciBuffer[cnt>>9]=OCR0B;
}

void bpmToBarlength() {
  barLength=960000L/bpm;      // since timer 0 has prescaler 1 instead of 64, 8000L=(8000/64) ms=125ms, resulting in 8 calls per second = 2 bars=120bpm 
}

/* Keyboard layout as follows:
row 0-3: 16 keypad, rows counting from top to bottom, col 0-3 counting from left to right
row 4: separate keys, col 3-0 counting from top to bottom on bread board
*/
void keypad_scan(){
  for (int i = 0; i < ROWS; i++) {
    // set row to LOW
    pinMode (rowPins[i], OUTPUT);

    for (int j = 0; j < COLS; j++) {
      int val = digitalRead (colPins[j]);
      keypadState[i][j] = (val == LOW);
    }

    // set row to High Z
    pinMode (rowPins[i], INPUT);
  }
}

// checks if a key is pressed (i.e. return true as long as key is pressed)
bool getKeyPress(byte i, byte j) {
  return keypadState[i][j];
}

// checks if a key is clicked (i.e. only returns true once until key is depressed)
bool getKeyClick(byte i, byte j) {
  if (keyLocked[i][j]) {
    if (!keypadState[i][j]) {
      keyLocked[i][j]=false;
    }
    return false;
  } else {
    if (keypadState[i][j]) {
      keyLocked[i][j]=true;
      return true;
    } else {
      return false;
    }
  }
}

// returns the bit pattern for pressed function keys. this only returns the current status, no click detection
uint8_t getFunctionKeys() {
  uint8_t t=0;
  for (int8_t i=3; i>=0; i--) {
    t=t<<1;
    t|=keypadState[4][i]|((keypadState[5][i])<<4);
  }
  return t;
}

// get the key click for the note keys 1-16. first click (1-16 in order) is returned
// if no key is pressed, -1 is returned
int8_t getNoteClick() {
  for (byte i=0; i<4; i++)
    for (byte j=0; j<4; j++)
      if (getKeyClick(j, 3-i))
        return i*4+j;
  return -1;
}

// get the key press for the note keys 1-16. first click (1-16 in order) is returned
// if no key is pressed, -1 is returned
int8_t getNotePress() {
  for (byte i=0; i<4; i++)
    for (byte j=0; j<4; j++)
      if (getKeyPress(j, 3-i))
        return i*4+j;
  return -1;
}

// show the current info in the lower 4 lines of the display (i.e. mode, octave, or entered note)
// if index 0 is given, lower 4 lines are cleared
void showInfo(byte textIndex) {
/*  if (displayMode != DISP_STD)
    return;
  */
  if (textIndex==0) {
    for (byte i=0;i<4;i++)
      lc.setRow(i+4,0);
  } else {
    for (byte i=0;i<4;i++)
      lc.setRow(i+4,pgm_read_byte(&infoDisp[textIndex*4+i]));
  }
}

void updateDispBuff(bool stuffIn[]) {
  for (int i=0; i<4; i++) {
    if (stuffIn[i<<2]) dispBuff[i]|=8;
    if (stuffIn[i<<2|1]) dispBuff[i]|=4;
    if (stuffIn[i<<2|2]) dispBuff[i]|=2;
    if (stuffIn[i<<2|3]) dispBuff[i]|=1;
  }
}

void oled_init() {
  Wire.begin();						// Init the I2C interface (pins A4 and A5 on the Arduino Uno board) in Master Mode.
  TWBR=0;						// Set the I2C to HS mode - 400KHz! TWBR = (CPU_CLK / I2C_CLK) -16 /2. Some report that even 0 is working. **** test it out ****
  Wire.beginTransmission(OLED_I2C_ADDRESS);		// Begin the I2C comm with SSD1306's address (SLA+Write)
  Wire.write(OLED_CONTROL_BYTE_CMD_STREAM);		// Tell the SSD1306 that a command stream is incoming
  Wire.write(OLED_CMD_DISPLAY_OFF);			// Turn the Display OFF
  Wire.write(OLED_CMD_SET_CONTRAST);			// set contrast
  Wire.write(0xff);
  Wire.write(OLED_CMD_SET_VCOMH_DESELCT);		// Set the V_COMH deselect volatage to max (0,83 x Vcc)
  Wire.write(0x30);
  Wire.write(OLED_CMD_SET_MEMORY_ADDR_MODE);		// vertical addressing mode
  Wire.write(0x01);
  Wire.write(OLED_CMD_SET_CHARGE_PUMP);			// Enable the charge pump
  Wire.write(0x14);
  Wire.write(OLED_CMD_DISPLAY_ON);			// Turn the Display ON
  Wire.write(OLED_CMD_SET_PAGE_RANGE);                  // use the current page
  Wire.write(0);
  Wire.write(7);
  Wire.write(OLED_CMD_SET_COLUMN_RANGE);                // use all columns
  Wire.write(0);
  Wire.write(127);
  Wire.endTransmission();
}

void writePattern() {
  uint16_t targetAddr=selectedPattern*48;    // each pattern uses 48 bytes
  for (int i=0; i<16; i++) {
    EEPROM.update(targetAddr++, notes[i]);
  }
  for (int i=0; i<16; i++) {
    EEPROM.update(targetAddr++, effect[i]);
  }
  for (int i=0; i<16; i++) {
    EEPROM.update(targetAddr++, activ[i]|(glide[i]<<1)|(accent[i]<<2));
  }
  EEPROM.update(EEPROM_BPM, bpm&255);
}

void readPattern() {
  uint16_t targetAddr=selectedPattern*48;    // each pattern uses 48 bytes
  uint8_t temp;
  for (int i=0; i<16; i++) {
    notes[i]=EEPROM.read(targetAddr++);
  }
  for (int i=0; i<16; i++) {
    effect[i]=EEPROM.read(targetAddr++);
  }
  for (int i=0; i<16; i++) {
    temp=EEPROM.read(targetAddr++);
    activ[i]=temp&1;
    glide[i]=(temp&2)>>1;
    accent[i]=(temp&4)>>2;
  }
  bpm=EEPROM.read(EEPROM_BPM);
}

void clearPattern() {
  for (int i=0;i<16;i++) {
    notes[i]=0;
    activ[i]=false;
    glide[i]=false;
    accent[i]=false;
    effect[i]=0;
  }
}

/*
********************************
Main loop which runs all the controls, display, sequencer and value stuff
This is called every 5ms to ensure to catch the sync signal and have some proper sequencer timing
The remaining time is used for displaying the oscilloscope display (see loop())
********************************
*/
ISR(TIMER2_COMPA_vect) {
  sei();                    // re-enable interrups directly, so the sound interrupt is ensured to run
  
  interruptWaiter++;
  if (interruptWaiter<5)
    return;
  interruptWaiter=0;

  now=millis();
  
  bpmToBarlength();
  
  cutRead=analogRead(A0);
  resRead=analogRead(A1);
  falloffSpeed=analogRead(A2)+3;
  
  if (glide[(step+1)%16])            // if we glide to the next note, double the falloff time, so the glide is still audible
    falloffSpeed*=2.0;
  
  if (playing||(!anyNotePressed)) {
    if (falloffSpeed<1024) {                  // max=no falloff
        volSub+=(now-lastCall)/falloffSpeed;
    }
  } else
    volSub=0;
  
  if (volSub>500)                           // cap off to avoid overflow for very low values of falloffSpeed
    volSub=500;
  if (volSub<0)
    volSub=0;
  
  if (glide[step]) {                // glide to the target frequency if gliding note. higher increment/decrement values mean faster glide
    if (targetNote>freq) {
      freq+=targetNoteStep;
      if (freq>targetNote)
        freq=targetNote;
    }
    if (targetNote<freq) {
      freq+=targetNoteStep;
      if (freq<targetNote)
        freq=targetNote;
    }
  }
  
  if (analogRead(A3)>syncThreshold) {          // we've got a sync signal
    if (!alreadySynced) {
      syncLength=now-lastSync;
      lastSync=now;                    // now we've got milliseconds since last sync. sync appears on every 1/8th, so BPM=60000/syncLength/2
      lastSyncLength=syncLength;      
      
      lastBPM[(lastBPMpointer++)%4]=1920000/syncLength;
      bpm=0;
      for (int i=0; i<4; i++)
        bpm+=lastBPM[i];
      bpm/=4;
      alreadySynced=true;
    }
  } else {
    alreadySynced=false;
    alreadySyncedStep=false;
  }

  // do sound-specific mapping of the parameters
  if (accent[step]) {
    cutRead+=50;            // raise resonance+cutoff on accent
    resRead+=150;
  }
  resRead=restrictValue(resRead, soundRange[selectedSound][0], soundRange[selectedSound][1]);
  cutRead=restrictValue(cutRead, soundRange[selectedSound][2], soundRange[selectedSound][3]);
  switch(selectedSound) {
    case 0:
      cutoff=cutRead/4-(volSub>400?100:(volSub>>2));
      if (cutoff<0)
        cutoff=0;
      resonance=resRead-512;
      break;
    case 1:
      cutoff=cutRead/4-(volSub>400?100:(volSub>>2));
      if (cutoff<0)
        cutoff=0;
      resonance=resRead-512;
      break;
    case 2:
      cutoff=cutRead/4-(volSub>400?100:(volSub>>2));
      if (cutoff<0)
        cutoff=0;
      resonance=resRead-512;
      break;
    case 3:
      cutoff=cutRead/4-(volSub>400?100:(volSub>>2));
      if (cutoff<0)
        cutoff=0;
      resonance=resRead-512;
      break;
    case 4:
      cutoff=cutRead/4-(volSub>400?100:(volSub>>2));
      if (cutoff<0)
        cutoff=0;
      resonance=resRead-512;
      cutoff=cutoff>>4;
      break;
    case 5:
      cutoff=cutRead-(volSub>400?100:(volSub>>2));
      if (cutoff<0)
        cutoff=0;
      resonance=resRead-512;
      cutoff=cutoff>>4;
     break;
    case 6:
      cutoff=cutRead-(volSub>400?100:(volSub>>2));
      if (cutoff<0)
        cutoff=0;
      resonance=resRead-512;
      cutoff=cutoff>>4;
      break;
    case 7:
      cutoff=cutRead-(volSub>400?100:(volSub>>2));
      if (cutoff<0)
        cutoff=0;
      resonance=resRead-512;
      cutoff=cutoff>>4;
      break;
    case 8:
      if (resRead<10) { // if LFO not running, use cutoff as direct phase position
        if ((lfoOffset>0)&&(lfoOffset>(128-(cutRead>>3)))) { // if we haven't reached the target LFO position, fade down until reached
          phasePosSwitch=lfoOffset;
          lfoOffset--;
        } else { // if already reached, directly use the cutoff value
          phasePosSwitch=128-(cutRead>>3);
          lfoPos=0x10000; // always start the lfopos at the position, where sin=0 so there's no jump, when turning the resonance back up
          lfoOffset=0; // set the offset to 0 so we don't get any unwanted fading
        }
      } else { // LFO is running
        lfoPos+=resRead-10;
        phasePosSwitch=restrictValue((pgm_read_byte(&sinetable[(lfoPos>>10)&255])<<2)+8, 128, 128-(cutRead>>3)); // LFO range is 128 to LFO phase position as selected by cutoff value
        lfoOffset=phasePosSwitch; // set the fading start point
      }      
      break;
    case 9:
      numberVoicesBroad=(resRead/128)+1;
      if (numberVoicesBroad==1) {
        volBroadStandard=0;
        volBroadClip=resRead*2;
      } else {
        volBroadStandard=(255*128/resRead);
        volBroadClip=(resRead%128*volBroadStandard/128);
      }
      int32_t temp;
      for (byte i=0; i<numberVoicesBroad; i++) {
        temp=freq;
        temp*=(cutRead-512)*i+1024;
        temp/=1024;
        freqBroad[numberVoicesBroad-1-i]=temp;
      }
      break;
    case 10:
      cutoff=cutRead-(volSub>400?100:(volSub>>2));
      if (cutoff<0)
        cutoff=0;
      resonance=resRead;
      cutoff=cutoff>>1;
      break;
    case 12:
      cutoff=(1023-cutRead)/2;
//      resonance=resRead-cutoff;
      resonance=resRead;
      break;  
  }

  if (playing) {
    if (((now-lastCall)>=barLength) || (alreadySynced&&(!alreadySyncedStep)&&((now-lastCall)>=barLength*0.7))) {      // since timer 0 has prescaler 1 instead of 64, 8000L=(8000/64) ms=125ms, resulting in 8 calls per second = 2 bars=120bpm
      if (alreadySynced)
        alreadySyncedStep=true;
      else
        alreadySyncedStep=false;
 
      if (playKeyStatus!=PLAYKEY_STARTED)    // if sequencer is just started, don't skip the first step!
        runningStep++;
      if (runningStep>=patternLength) {
        runningStep=0;                       // reset step
        if (songLength>0) {
          currentSongStep++;                 // increase song pointer
          if (currentSongStep>songLength)
            currentSongStep=0;
          selectedPattern=songPatterns[currentSongStep];    // and load the next pattern
          readPattern();
        }
      }
      step=(runningStep+stepOffset)%patternLength;
      
      lastCall=now;

      anyNotePressed=true;        // trigger sound
    }
    lastMicron=micron;
    micron=(now-lastCall)*64/barLength;
  } else {
    if (((now-lastCall)>=barLength) || (alreadySynced&&(!alreadySyncedStep)&&((now-lastCall)>=barLength*0.7))) {  // replicate the timing model as if the sequencer were running
      lastCall=now;
    }
    lastMicron=micron;
    micron=0;
  }

  if (anyNotePressed) {          // trigger sounds if necessary
    if (playing) {
      act=activ[step];
      if ((!act)||(glide[step])) {
        if (!glide[step])            // only reset waveform if not gliding
          cnt=0;
      } else {
        if (accent[step])            // attenuate on accent
          volSub=0;
        else
          volSub=30;
      }
    }
    
    if (activ[step]) {
      int8_t tempNote=notes[step]+transpose+transposeTemp;
      if (tempNote<0)
        tempNote=0;
      if (tempNote>=sizeof(noteFreq))
        tempNote=sizeof(noteFreq)-1;
      if (!glide[step])
        freq=noteFreq[tempNote]/2;
      else {
        targetNote=noteFreq[tempNote]/2;
        targetNoteStep=(targetNote-freq)/12;
      }
    }
  }

  for (int i=0; i<4; i++)            // set step LED for display
    dispBuff[i]=0;
  dispBuff[step>>2]=1<<(7-(step&3));
  blinker++;
  if (blinker>18)
    blinker=0;
  if (blinker>13) {
    dispBuff[selectedPattern>>2]^=1<<(7-(selectedPattern&3));    // show the selected pattern blinking
  }
  
  if (alreadySynced)            // display sync in in upper right LED
    dispBuff[0]|=1;

  keypad_scan();
  anyNotePressed=false;
  
  int8_t pressedKey=getNotePress();
  int8_t clickedKey=getNoteClick();
  if (getFunctionKeys()!=B10000000)      // if pattern button is not pressed, leave song programming mode
    patternButtonPressed=false;
  switch (getFunctionKeys()) {
    case B00000001:                      // select sound
      if (pressedKey!=-1)
        selectedSound=pressedKey;
      dispBuff[selectedSound>>2]|=1<<(3-(selectedSound&3));
      showInfo(MODE_SOUND);
    break;
    case B00000010:                      // set glide
      if (clickedKey!=-1)
        glide[clickedKey]=!glide[clickedKey];
      updateDispBuff(glide);
      showInfo(MODE_GLIDE);
    break;
    case B00000100:                      // set accent
      if (clickedKey!=-1)
        accent[clickedKey]=!accent[clickedKey];
      updateDispBuff(accent);
      showInfo(MODE_ACCENT);
    break;
    case B00001000:                      // set mute
      if (clickedKey!=-1)
        activ[clickedKey]=!activ[clickedKey];
      updateDispBuff(activ);
      showInfo(MODE_ACTIVE);
    break;
    case B00010000:                      // transpose whole pattern
      if (pressedKey!=-1)
        transposeTemp=((3-(pressedKey/4))*4)+(pressedKey%4);
      dispBuff[3-(transposeTemp>>2)]|=1<<(3-(transposeTemp&3));
      showInfo(MODE_TRANSPOSE);
    break;
    case B00100000:                      // select step to play
      switch(playKeyStatus) {
        case PLAYKEY_STOPPED:
          playing=true;
          stepOffset=0;
          playKeyStatus=PLAYKEY_STARTED;
          stepOffsetSelected=false;
        break;
        case PLAYKEY_PLAYING:
          playKeyStatus=PLAYKEY_PRESSED_PLAY;
        break;
        case PLAYKEY_PRESSED_PLAY:
          showInfo(MODE_JUMP);
          if (clickedKey!=-1) {
            stepOffset=(clickedKey+15-runningStep)&15;
            stepOffsetSelected=true;
          } else {
            if (pressedKey!=-1) {
              stepOffsetSelected=true;
            }
          }
        break;
      }
    break;
    case B01000000:                      // FX
      if (pressedKey!=-1)
        effect[step]=pressedKey;
      dispBuff[effect[step]>>2]|=1<<(3-(effect[step]&3));
      showInfo(MODE_EFFECT);
    break;
    case B01000100:                      // Pattern length
      if (pressedKey!=-1)
        patternLength=pressedKey+1;
      dispBuff[(patternLength-1)>>2]|=1<<(3-((patternLength-1)&3));
      showInfo(MODE_LENGTH);
    break;
    case B10000000:                      // load pattern
      if (clickedKey!=-1) {
        if (patternButtonPressed) {      // song mode
          if (songLength<15)             // only store if there's still space to store patterns left
            songPatterns[++songLength]=clickedKey;
        } else {                         // button was not pressed previously => directly select pattern
          songPatterns[0]=selectedPattern=clickedKey;
          readPattern();
          songLength=0;
          currentSongStep=0;
        }
      }
      if (patternButtonPressed) {
        showInfo(MODE_SONG);
      } else {
        showInfo(MODE_LOADPATTERN);
      }
      if ((patternButtonPressed)&&(songLength>0)) {
        if (songLength>3) dispBuff[0]|=15;
        if (songLength>7) dispBuff[1]|=15;
        if (songLength>11) dispBuff[2]|=15;
        dispBuff[songLength>>2]|=16-(1<<(3-(songLength&3)));
      } else {
        dispBuff[selectedPattern>>2]|=1<<(3-(selectedPattern&3));
      }
      if (clickedKey!=-1)
        patternButtonPressed=true;
    break;
    case B10001000:                      // write pattern
      if (clickedKey!=-1) {
        selectedPattern=clickedKey;
        writePattern();
      }
      showInfo(MODE_WRITEPATTERN);
    break;
    case B00010001:                      // set BPM
      switch(clickedKey) {
        case 0:
          if (bpm>MINBPM)
            bpm--;
        break;
        case 1:
          if (bpm<MAXBPM)
            bpm++;
        break;
        case 4:
          if (bpm-5>MINBPM)
            bpm-=5;
          else
            bpm=MINBPM;
        break;
        case 5:
          if (bpm+5<MAXBPM)
            bpm+=5;
          else
            bpm=MAXBPM;
        break;
        case 8:
          if (bpm-10>MINBPM)
            bpm-=10;
          else
            bpm=MINBPM;
        break;
        case 9:
          if (bpm+10<MAXBPM)
            bpm+=10;
          else
            bpm=MAXBPM;
        break;
        case 12:
          bpm=90;
        break;
        case 13:
          bpm=120;
        break;
        case 14:
          bpm=140;
        break;
        case 15:
          bpm=180;
        break;
      }
      // now display the current BPM
      uint8_t dt;
      for (byte i=0; i<4; i++) {
        dt=pgm_read_byte(&infoDisp[(MODE_HUNDRED+(bpm/100))*4+i]);
        dt|=pgm_read_byte(&infoDisp[(MODE_NUMBER+((bpm/10)%10))*4+i]);
        dt|=(pgm_read_byte(&infoDisp[(MODE_NUMBER+(bpm%10))*4+i])>>3);
        lc.setRow(i+4, dt);
      }
      EEPROM.update(EEPROM_BPM, bpm&255);
    break;
    case B00001110:                      // clear pattern
      clearPattern();
      showInfo(MODE_CLEAR);
    break;
    case B00000000:                      // enter notes
      switch(playKeyStatus) {
        case PLAYKEY_STARTED:
          playKeyStatus=PLAYKEY_PLAYING;
        break;
        case PLAYKEY_PRESSED_PLAY:
          if (!stepOffsetSelected) {
            playKeyStatus=PLAYKEY_STOPPED;
            playing=false;
            runningStep=0;
            step=0;
            stepOffset=0;
            if (songLength>0) {
              currentSongStep=0;
              selectedPattern=songPatterns[currentSongStep];    // and load the next pattern
              readPattern();
            }
          } else {
            playKeyStatus=PLAYKEY_PLAYING;
            stepOffsetSelected=false;
          }
        break;
      }
      switch(clickedKey) {
        case 0:
          octave--;
          if (octave<MIN_OCTAVE)
            octave=MIN_OCTAVE;
            if (activ[step])
              notes[step]=(notes[step]%12)+octave*12;
//          showInfo(OCT1+octave);
          break;
        case 1:
          octave++;
          if (octave>MAX_OCTAVE)
            octave=MAX_OCTAVE;
            if (activ[step])
              notes[step]=(notes[step]%12)+octave*12;
  //        showInfo(OCT1+octave);
          break;
        case 2:
          if (!playing) {
            step=runningStep=(runningStep+15)&15;
          }
        break;
        case 3:
          if (!playing) {
            step=runningStep=(runningStep+1)&15;
          }
        break;
      }
      if (((pressedKey>=0)&&(pressedKey<4))&&(!playing)&&(activ[step])) {
        anyNotePressed=true;
      }
      byte playedNote=notes[step]%12;      // show currently played note/octave when write is pressed
      byte playedOctave=notes[step]/12;
      if (pressedKey>=4) {
        notes[step]=((3-(pressedKey/4))*4)+(pressedKey%4)+octave*12;
        activ[step]=true;
        showInfo(NOTE_C+playedNote);
        anyNotePressed=true;
      }
      if (activ[step]) {                    // only show played note if it is active
        dispBuff[0]|=1<<(3-playedOctave);
        dispBuff[3-(playedNote>>2)]|=1<<(3-(playedNote&3));
      }

      if ((pressedKey==0)||(pressedKey==1)) {
        showInfo(OCT1+octave);
      } else {      
        if (playing) {
          showInfo(MODE_NOTE+(step%4));
        } else {
          if (activ[step])
            showInfo(NOTE_C+playedNote);
          else
            showInfo(0);
        }
      }
    break;
  }
  
  currentEffect=effect[step];
  transpose=0;
  switch(currentEffect) {
    case 0:                    // no effect
      transpose=0;
    break;
    case 1:                    // shift every 8th note up
      transpose=(step&2)*6;
    break;
    case 2:
      if (playing)
        if (micron==32) {
          anyNotePressed=true;
          volSub=0;              // retrigger
        }
      break;
    case 4:                    // arpeggios
    case 5:
    case 6:
    case 7:
      transpose=arpeggio[currentEffect-4][micron/16];
      if ((micron/16)!=lastArpeggioStep) {
        lastArpeggioStep=micron/16;
        anyNotePressed=true;
      }
    break;
  }

//  if (displayMode == DISP_STD) {
    for (int i=0; i<4; i++)                    // update display upper half
      lc.setRow(i, dispBuff[i]);
  //}
  /*
  int i,j,r;
  if (displayMode == DISP_OSC) {
    for (i=0;i<16;i+=2) {
      for (j=0;j<8;j++)
        dispBuff[j]=dispBuff[j]<<1;
      r=osciBuffer[i<<3]+128;
      r=(r>=0?(r<=255?r:255):0);
      dispBuff[r>>5]|=1;
      r=osciBuffer[(i+1)<<3]+128;
      r=(r>=0?(r<=255?r:255):0);
      dispBuff[r>>5]|=1;
    }
    for (j=0;j<8;j++)                        // only display oscilloscope if current step is active
      lc.setRow(j, (act?dispBuff[7-j]:0));
  }
  */

  #ifdef profiling
  numberStepper++;                    // profiling stuff. The number of steps per sek should be >100 at all costs, otherwise sync could break!!!
  timePerStepper+=millis()-now;
  if (millis()-lastProfile>=64000){
    Serial.print(numberStepper);
    Serial.println(" steps/sec");
    numberStepper=0;
    timePerStepper=0;
    lastProfile=millis();
  }
  #endif
}


void loop(){
  uint8_t sample, sampleLo, sampleHi;
  uint8_t dispII, page;
  while (true) {
    for (dispII=0; dispII<128; dispII++) {
      Wire.beginTransmission(OLED_I2C_ADDRESS);
      Wire.write(OLED_CONTROL_BYTE_DATA_STREAM);
      sample=(osciBuffer[127-dispII])>>2;
      sampleLo=1<<(sample&7);
      sampleHi=sample>>3;
      for (page=0; page<8; page++)
        Wire.write(page==sampleHi?sampleLo:0);
      Wire.endTransmission();   
    }
  }
}
