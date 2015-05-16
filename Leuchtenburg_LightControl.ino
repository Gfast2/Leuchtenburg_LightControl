/* This code can control Light control board and Musik Shield. This version for Music have only very limited functions.
 * This code is derived from "_2013_12_12_Musik_SoundControl_Serial_Protocol".
 * working in progress (don't know if work).
 * Msg. format: "start;", "stop;", "blink;","sd A100 B100;"
 *
 * Usage:
 * "kerze;"  :  Four LEDs do kerze simulation
 * "blink;"  :  Four LEDs blink.
 * "lt E0;"  :  Four LEDs are turned off
 * "lt E1;"  :  Four LEDs are turned on (max. brightness)
 * "lt A1000 B2000 C3000 D4000;"
 *           :  set first LED's birghtness to 1000, second to 2000 and so on. Valid value is from 0 to 4095
 * "lt KA;"  :  set first LED do kerze simulation when in blink modus.
 * "lt Ka;"  :  set first LED do blink when in blink modus.
 *
 * New feature:
 * / the LED can be single blinked.
 * 
 * TODO: timer(int milliseconds) retire blink(int) to let code not be blocked by "delay()"
 *
 * Edit: 13 Mai. 2015
 * Written by Su Gao
 *
 */
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>  // for type definitions
// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define KerzeSim 0
#define Blink 1
#define ON 2
#define OFF 3
int LEDState = 1;

#define normal 0
#define finish 1

#define fastSpeed 10
#define middleSpeed 30
#define slowSpeed 60

int LEDs[4] = {3,5,6,9}; //save the pin number of all four LEDs. (only for Programm test)


byte Message[6]={
  0x86, 0x01, 0x02, 0x01, 0x1F, 0x00};
byte CRC, Channel, Volume;

int VolumeNow[8] = {
  0,0,0,0,0,0,0,0}; //0-255, Max-0, Min-255
int SoundBlendSpeed[8] = {
  3,3,3,3,3,3,3,3};

static const byte p[] = {
  151,160,137,91,90, 15,131, 13,201,95,96,
  53,194,233, 7,225,140,36,103,30,69,142, 8,99,37,240,21,10,23,190, 6,
  148,247,120,234,75, 0,26,197,62,94,252,219,203,117, 35,11,32,57,177,
  33,88,237,149,56,87,174,20,125,136,171,168,68,175,74,165,71,134,139,
  48,27,166, 77,146,158,231,83,30,229,122,6,211,133,255,255,105,92,
  41,55,46,245,40,244,102,143,54,65,25,63,161, 1,216,80,73,209,76,132,
  187,208, 89, 18,169,200,196,135,130,116,188,159, 86,255,100,109,198,
  173,186, 3,64,52,217,226,250,124,123,5,202,38,147,118,126,255,82,85,
  212,207,206, 59,227, 47,16,58,17,182,189, 1,42,255,183,170,213,119,
  248,152,2,44,154,163,70,221,153,101,155,167,43,172, 1,129,22,39,253,
  19,98,40,110,79,113,224,232,178,185,112,104,218,246, 97,228,251,34,
  242,193,238,210,144,12,191,179,162,241,81,51,145,235,249,14,239,107,
  49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,1,4,1, 4,
  150,254,138,236,205, 93,222,114, 67,29,24, 2,243,141,128,195,78,66,
  215,61,156,180
};

double fade(double t){ 
  return t * t * t * (t * (t * 6 - 15) + 10); 
}
double lerp(double t, double a, double b){ 
  return a + t * (b - a); 
}
double grad(int hash, double x, double y, double z)
{
  int     h = hash & 15;          /* CONVERT LO 4 BITS OF HASH CODE */
  double  u = h < 8 ? x : y,      /* INTO 12 GRADIENT DIRECTIONS.   */
  v = h < 4 ? y : h==12||h==14 ? x : z;
  return ((h&1) == 0 ? u : -u) + ((h&2) == 0 ? v : -v);
}

#define P(x) p[(x) & 255]

double pnoise(double x, double y, double z)
{
  int   X = (int)floor(x) & 255,             /* FIND UNIT CUBE THAT */
  Y = (int)floor(y) & 255,             /* CONTAINS POINT.     */
  Z = (int)floor(z) & 255;
  x -= floor(x);                             /* FIND RELATIVE X,Y,Z */
  y -= floor(y);                             /* OF POINT IN CUBE.   */
  z -= floor(z);
  double  u = fade(x),                       /* COMPUTE FADE CURVES */
  v = fade(y),                          /* FOR EACH OF X,Y,Z.  */
  w = fade(z);
  int  A = P(X)+Y,
  AA = P(A)+Z,
  AB = P(A+1)+Z,                        /* HASH COORDINATES OF */
  B = P(X+1)+Y,
  BA = P(B)+Z,
  BB = P(B+1)+Z;                        /* THE 8 CUBE CORNERS, */

  return lerp(w,lerp(v,lerp(u, grad(P(AA  ), x, y, z),   /* AND ADD */
  grad(P(BA  ), x-1, y, z)),               /* BLENDED */
  lerp(u, grad(P(AB  ), x, y-1, z),        /* RESULTS */
  grad(P(BB  ), x-1, y-1, z))),            /* FROM  8 */
  lerp(v, lerp(u, grad(P(AA+1), x, y, z-1),/* CORNERS */
  grad(P(BA+1), x-1, y, z-1)),             /* OF CUBE */
  lerp(u, grad(P(AB+1), x, y-1, z-1),
  grad(P(BB+1), x-1, y-1, z-1))));
}

#define ledPin 13

void blink(int dur)
{
  digitalWrite(ledPin, HIGH);
  delay(dur);
  digitalWrite(ledPin, LOW);
}


// Serial comm reception
#define MAX_BUF         (64)
static char buffer[MAX_BUF];  // Serial buffer
static int sofar;             // Serial buffer progress

boolean kerzeBlinked[4] = {0,0,0,0}; //toggler to check if in kerzeSim() mode still do kerzeBlink(), 1 = kerzeBlink, 0 = kerzeSim.

//----------------------------------
int brightNow[4] = {
  1500,1500,1500,1500}; //default brightness 1500. Max. 4095
int brightBlend[4] = {
  300,300,300,300}; //blend speed, 300 is very natural blend.

//----------------------------------
void setKerzeBright(int num, int brightness){
  brightness = brightness > 4000 ? 4000 : brightness;
  brightness = brightness < 500 ? 500 : brightness;
  brightNow[num-1] = brightness; //set new brightness.
}

//----------------------------------
void setKerzeBlend(int num, int blendSpeed=10){
  blendSpeed = blendSpeed > 800 ? 800 : blendSpeed;
  blendSpeed = blendSpeed < 10 ? 10 : blendSpeed;
  brightBlend[num-1] = blendSpeed;
}

//----------------------------------
void kerzeON(){
  for(int i=0; i<4;i++) pwm.setPWM(i,0,4000);
}

//----------------------------------
void kerzeOFF(){
  for(int i=0; i<4;i++) pwm.setPWM(i,0,0);
}

//---------------------------------//
void processCommand(){

  if(!strncmp(buffer,"kerze",5)) {
    LEDState = KerzeSim;
    Serial.println(F("LEDs all KerzeSim"));
  }
  else if(!strncmp(buffer,"blink",5)) {
    LEDState = Blink;
    Serial.println(F("LEDs all blink"));
  }
  else if(!strncmp(buffer,"lt",2)){

    static float brightness1;
    static float brightness2;
    static float brightness3;
    static float brightness4;

    static float blendSpeed1;
    static float blendSpeed2;
    static float blendSpeed3;
    static float blendSpeed4;

    static float LEDtrigger; //boolean variable enable / disable all LEDs (ON/OFF)

    char *ptr=buffer;
    while(ptr && ptr<buffer+sofar && strlen(ptr)) {
      switch(*ptr) {
      case 'A': // brightness for each LED
        brightness1 = atof(ptr+1);
        Serial.print("LED1 brightness: ");
        Serial.println(brightness1);
        setKerzeBright(1,brightness1);
        break;
      case 'B':
        brightness2 = atof(ptr+1);
        Serial.print("LED2 brightness: ");
        Serial.println(brightness2);
        setKerzeBright(2,brightness2);
        break;
      case 'C':
        brightness3 = atof(ptr+1);
        Serial.print("LED3 brightness: ");
        Serial.println(brightness3);
        setKerzeBright(3,brightness3);
        break;
      case 'D':
        brightness4 = atof(ptr+1);
        Serial.print("LED4 brightness: ");
        Serial.println(brightness4);
        setKerzeBright(4,brightness4);
        break;
      case 'a': // LED blend speed for each LED
        blendSpeed1 = atof(ptr+1);
        Serial.print("LED1 blendSpeed: ");
        Serial.println(blendSpeed1);
        break;
      case 'b':
        blendSpeed2 = atof(ptr+1);
        Serial.print("LED2 blendspeed: ");
        Serial.println(blendSpeed2);
        break;
      case 'c':
        blendSpeed3 = atof(ptr+1);
        Serial.print("LED3 blendspeed: ");
        Serial.println(blendSpeed3);
        break;
      case 'd':
        blendSpeed4 = atof(ptr+1);
        Serial.print("LED4 blendspeed: ");
        Serial.println(blendSpeed4);
        break;
      case 'E': //enable or disable all LED
        LEDtrigger = atof(ptr+1);
        Serial.print("LEDtrigger set:"); //lt E0;
        Serial.println(LEDtrigger);
        if(LEDtrigger)
          LEDState = ON;
        else 
          LEDState = OFF;
        break;
      case 'K': //K+channel number in which we blinke LEDs in kerzSim() mode. For example "lt KA KB KC" means LED1,2,3 will blink and the LED4 will still simulate candle.
          // When we wanna set all LEDs do simulation, only send "lt K" or even something else without 'A','B','C','D'
        switch(*(ptr+1)){ //read which LED should play blink. '*' operator is "pointer to a variable".
          case 'A':
            kerzeBlinked[0] = true;            
            break; 
          case 'B':
            kerzeBlinked[1] = true;
            break; 
          case 'C':
            kerzeBlinked[2] = true;
            break; 
          case 'D':
            kerzeBlinked[3] = true;
            break; 
          case 'a':
            kerzeBlinked[0] = false;
            break; 
          case 'b':
            kerzeBlinked[1] = false;
            break; 
          case 'c':
            kerzeBlinked[2] = false;
            break; 
          case 'd':
            kerzeBlinked[3] = false;
            break;   
          default: //let all LEDs do kerze simulation.
            for(int i=0; i<4; i++){
              kerzeBlinked[i] = true;
            }          
        }
        break;        
      }
      ptr=strchr(ptr,' ')+1; //find the next empty space & let pointer point to the next addr
    }
  }
}

//----------------------------------
void kerzeBlink(){
  blink(5); //control the speed of the light from dark to Bright
  static int brightness = 100;
  static int val = 100;
  val = (brightness==0 || brightness==4000) ? -val : val;
  brightness += val;
  for(int i=0; i<4;i++) 
    pwm.setPWM(i,0,brightness);
}

//----------------------------------
// Overloaded KerzeBlink(). It's used in KerzeSim() mode to let a LED still do blink.
void kerzeBlink(int kerze){
  static int brightness = 100;
  static int val = 100; //because of the timer()'s different between kerzeBlink() and KerzeSim(), val should be fine tune a little bit.
  val = (brightness == 0 || brightness == 4000) ? -val : val;
  brightness += val;
  pwm.setPWM((kerze-1),0,brightness); //set the acutal LED to do blink.
}

//----------------------------------
void kerzeSim(){
  double a[4];
  static double b[4];
  int outputTmp[4];
  int output[4];

  static  int brightOld[4] = {
    1500,1500,1500,1500                
  }; //default brightness 1500. Max. 4095

  b[0] += 0.07;

  for(int i=1; i<4; i++) {
    b[i] = b[i-1] + 0.01;
  }

  blink(25); //this delay() control the speed of the kertze Brighness variation speed. war 125ms //before sound(): 50

  a[0] = pnoise(b[0] + sin(b[0]),b[0] +cos(b[0]),b[0]);
  a[1] = pnoise(b[1] + sin(b[1]),b[1] +sin(b[1]),b[1]);
  a[2] = pnoise(b[2] + cos(b[2]),b[2] +cos(b[2]),b[2]);
  a[3] = pnoise(b[3] + cos(b[3]),b[3] +sin(b[3]),b[3]);

  for (int i=0; i<4; i++){
    if(abs(brightNow[i] - brightOld[i]) > brightBlend[i] + 5) {
      brightOld[i] = brightNow[i] > brightOld[i] ? (brightOld[i] + brightBlend[i]) : (brightOld[i] - brightBlend[i]);
    }
    else{
      brightOld[i] = brightNow[i];
    }

    outputTmp[i] = int(127 + (a[i] * 127)); //second 127 changed from 10
    output[i] = int(map(outputTmp[i], 0, 255, 5/*300 relative naturl*/, brightOld[i]/*brightNow[i]*/));
    
    if( kerzeBlinked[i] == false){ //added on 2015-2-27
      pwm.setPWM(i,0,output[i]); //TODO: Here should check which mode should this specific LED play. kerzeBlink() or KerzeSim()
    } else {
      kerzeBlink(i+1);
    }
  }
}

//----------------------------------
void setup(){
  Serial.begin(38400); //try the functionality so use a safer speed.
  Serial.println("\nHello, This is light & Sound Control Board");

  pinMode(ledPin,OUTPUT);
  digitalWrite(ledPin,LOW);

  pwm.begin();
  pwm.setPWMFreq(1600);  // This is the maximum PWM frequency

  // save I2C bitrate
  uint8_t twbrbackup = TWBR;
  TWBR = 12; // upgrade to 400KHz!
}

//----------------------------------
void loop() {

  // See: http://www.marginallyclever.com/2011/10/controlling-your-arduino-through-the-serial-monitor/

  while(Serial.available() > 0) {
    buffer[sofar++]=Serial.read(); //save the Serial.read() and then get the sofar++
    if(buffer[sofar-1]==';') break;  // in case there are multiple instructions, break out the while loop to excute the command
  }

  // if we hit a semi-colon, assume end of instruction.
  if(sofar>0 && buffer[sofar-1]==';') {
    buffer[sofar]=0; //because the last Serial.read() saved at buffer[sofar-1] and then sofar++, so there is nothing saved it buffer[sofar]

    // echo confirmation
    Serial.println(buffer);

    // do something with the command
    processCommand();

    // reset the buffer
    sofar=0;

    // echo completion
    Serial.print(F("> "));
  }

  if(LEDState == KerzeSim){
    kerzeSim();
  }
  else if(LEDState == Blink){
    kerzeBlink();
    for(int i=0; i<4; i++)  kerzeBlinked[i] = false; //reset all kerzeBlinked[i] flag.
  }
  else if(LEDState == ON){
    kerzeON();
  }
  else if(LEDState == OFF){
    kerzeOFF();
  }
  
  Serial.print("kerzeBlinked flags: ");
  for(int i=0; i<4; i++) 
  {
    Serial.print(" ");
    Serial.print(kerzeBlinked[i]);
  }
  Serial.println();
}






