#include "Adafruit_NeoPixel.h"

#define LEFT_PIN 5
#define RIGHT_PIN 6
#define PUSHBAR_PIN 7


#define LEFT_LENGTH 90
#define RIGHT_LENGTH 90
#define PUSHBAR_LENGTH 150

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel left = Adafruit_NeoPixel(90, LEFT_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel right = Adafruit_NeoPixel(90,RIGHT_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pushbar = Adafruit_NeoPixel(150,PUSHBAR_PIN, NEO_GRB + NEO_KHZ800);


int tick;

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.

void setup() {
  left.begin();
  right.begin();
  pushbar.begin();

  left.show(); // Initialize all pixels to 'off'
  right.show(); // Initialize all pixels to 'off'
  pushbar.show(); // Initialize all pixels to 'off'
  tick = 0;  
}

void loop() {
rightTurnSignal(true);
//leftTurnSignal(true);
//brakeLights(true);
//ESTOPLIGHTS();

//uint32_t colors[] = {
                     left.Color(  0,   0, 127),
                     left.Color(  0,   127, 0),
                     left.Color(  127,   0, 0)
                   };
  
//PUSHBAR_SIGNAL_LIGHTS(sizeof(colors)/4,colors);
//UNDERGLOW_SIGNAL_LIGHTS(sizeof(colors)/4,colors);
systemTest();
// drawPicture();
left.show();
right.show(); 
pushbar.show();
delay(50);
tick++;
}

void brakeLights(boolean on){
  if(on){
   for(int i = 0;i<3;i++){
      left.setPixelColor(i,left.Color(255,0,0)); //Yellow
      right.setPixelColor(i,left.Color(255,0,0)); //Yellow

    }
  }else{
    for(int i = 0;i<3;i++){
      left.setPixelColor(i,left.Color(0,0,0)); //Yellow
      right.setPixelColor(i,right.Color(0,0,0)); //Yellow

    }
  }
 
}

void ESTOPLIGHTS(){
  setALL(left.Color(255,0,0));
}


void setALL(uint32_t c){
    for(int i = 0; i< LEFT_LENGTH;i++){
      left.setPixelColor(i,c);
    }
    for(int i = 0; i< RIGHT_LENGTH;i++){
      right.setPixelColor(i,c); 
    }
    for(int i = 0; i< PUSHBAR_LENGTH;i++){
      pushbar.setPixelColor(i,c); 
    }
}
  
void PUSHBAR_SIGNAL_LIGHTS(int numLights,uint32_t* colors){
  int min = 10;
  int max = 74;
  int size = (max - min)/numLights;
  int k;
  for(int j = 0;j<numLights;j++){
    for(int i = 0;i<size;i++){
      k = min + j*size + i;
      pushbar.setPixelColor(k,colors[j]);
      pushbar.setPixelColor(PUSHBAR_LENGTH - k,colors[j]);      
    }
  }  
}

void UNDERGLOW_SIGNAL_LIGHTS(int numLights,uint32_t* colors){
  int min = 4;
  int max = 75;
  int size = (max - min)/numLights;
  int k;
  for(int j = 0;j<numLights;j++){
    for(int i = 0;i<size;i++){
      k = min + j*size + i;
      left.setPixelColor(k,colors[j]);
      right.setPixelColor(k,colors[j]);      
    }
  }  
  
}

void meatBall(int relativeHeading){
  
//    pushbar.setPixelColor(,colors[j]);
}

void drawPicture(){
  uint32_t image[2][64];
  
 /*
  
  for(int i = 0;i < 64; i++){
  image[0][i] = left.Color(255,0,0);
  }
  for(int i = 1;i < 64; i++){
    image[1][i] = left.Color(0,255,0);
  }
  
 int min = 10;
  int max = 74;
  int k;
  for(int j = 0;j<64;j++){
      k = min + j;
      pushbar.setPixelColor(k,image[tick % 2][j]);
      pushbar.setPixelColor(PUSHBAR_LENGTH - k,image[tick %2][j]);      
  }  
  
  delay(100);
*/  
  //  image[10][10][3];
  
   //TODO 
  
}

void systemTest(){
 // Some example procedures showing how to display to the pixels:
  colorWipe(left.Color(255, 0, 0), 25); // Red
  colorWipe(right.Color(255, 0, 0), 25); // Red
  colorWipe(pushbar.Color(255, 0, 0), 25); // Red

  colorWipe(left.Color(0, 255, 0), 25); // Green
  colorWipe(right.Color(0, 255, 0), 25); // Green
  colorWipe(pushbar.Color(0, 255, 0), 25); // Green


  colorWipe(left.Color(0, 0, 255), 25); // Blue
  colorWipe(right.Color(0, 0, 255), 25); // Blue
  colorWipe(pushbar.Color(0, 0, 255), 25); // Blue

  // Send a theater pixel chase in...
  theaterChase(left.Color(127, 127, 127), 50); // White
  theaterChase(right.Color(127, 127, 127), 50); // White
  theaterChase(pushbar.Color(127, 127, 127), 50); // White

  theaterChase(left.Color(127,   0,   0), 50); // Red
  theaterChase(right.Color(127,   0,   0), 50); // Red
  theaterChase(pushbar.Color(127,   0,   0), 50); // Red

  theaterChase(left.Color(  0,   0, 127), 50); // Blue
  theaterChase(right.Color(  0,   0, 127), 50); // Blue
  theaterChase(pushbar.Color(  0,   0, 127), 50); // Blue

  rainbow(20);
  rainbowCycle(20);
  theaterChaseRainbow(50);
 
}

void allOFF(){
  setALL(left.Color(0,0,0));
 
}  


void setBackRightTurnSignal(uint32_t c){
    for(int i = 0;i<10;i++){
      pushbar.setPixelColor(i,c);
    }  
}

void setFrontRightTurnSignal(uint32_t c){
      for(int i = 0;i<15;i++){
          right.setPixelColor(RIGHT_LENGTH - i,c);
    }
}

void setBackLeftTurnSignal(uint32_t c){
   for(int i = 0;i<10;i++){
      pushbar.setPixelColor(PUSHBAR_LENGTH-i,c);
    }
}

void rightTurnSignal(boolean on ){
  if(on){
  if(tick %40 < 20){
       setBackRightTurnSignal(right.Color(255,50,0));
       setFrontRightTurnSignal(right.Color(255,50,0));
  }else{
     setBackRightTurnSignal(right.Color(0,0,0));
    setFrontRightTurnSignal(right.Color(0,0,0));
}
  }else{
     setBackRightTurnSignal(right.Color(0,0,0));
    setFrontRightTurnSignal(right.Color(0,0,0));
  } 
  
}



void leftTurnSignal(boolean on){
  if(tick %40 < 20){
    //leftTurnSignal
  setBackLeftTurnSignal(left.Color(255,50,0));
    for(int i = 0;i<15;i++){
          left.setPixelColor(LEFT_LENGTH - i,left.Color(255,50,0));
    }
  }else{
      setBackLeftTurnSignal(left.Color(0,0,0));
        for(int i = 0;i<15;i++){
          left.setPixelColor(LEFT_LENGTH - i,right.Color(0,0,0));
    }
    }

  
    pushbar.show();
    left.show();
}


// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<left.numPixels(); i++) {
      left.setPixelColor(i, c);
      right.setPixelColor(i, c);
      pushbar.setPixelColor(i, c);
      pushbar.setPixelColor(PUSHBAR_LENGTH -i, c);
      

      left.show();
      right.show();
      pushbar.show();

      delay(wait);
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<left.numPixels(); i++) {
      left.setPixelColor(i, Wheel((i+j) & 255));
      right.setPixelColor(i, Wheel((i+j) & 255));
      pushbar.setPixelColor(i, Wheel((i+j) & 255));
      pushbar.setPixelColor(PUSHBAR_LENGTH - i, Wheel((i+j) & 255));


    }
    left.show();
    right.show();
    pushbar.show();

    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< left.numPixels(); i++) {
      left.setPixelColor(i, Wheel(((i * 256 / left.numPixels()) + j) & 255));
      right.setPixelColor(i, Wheel(((i * 256 / left.numPixels()) + j) & 255));
      pushbar.setPixelColor(i, Wheel(((i * 256 / left.numPixels()) + j) & 255));
      pushbar.setPixelColor(PUSHBAR_LENGTH - i, Wheel(((i * 256 / left.numPixels()) + j) & 255));


    }
    left.show();
    right.show();
    pushbar.show();

    delay(wait);
  }
}

//Theatre-style crawling lights.
void theaterChase(uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (int i=0; i < left.numPixels(); i=i+3) {
        left.setPixelColor(i+q, c);    //turn every third pixel on
        right.setPixelColor(i+q, c);    //turn every third pixel on
        pushbar.setPixelColor(i+q, c);    //turn every third pixel on
        pushbar.setPixelColor(PUSHBAR_LENGTH -(i+q), c);    //turn every third pixel on

      }
      left.show();
      right.show();
      pushbar.show();
     
      delay(wait);
     
      for (int i=0; i < left.numPixels(); i=i+3) {
        left.setPixelColor(i+q, 0);        //turn every third pixel off
        right.setPixelColor(i+q, 0);        //turn every third pixel off
        pushbar.setPixelColor(i+q, 0);        //turn every third pixel off
        pushbar.setPixelColor(PUSHBAR_LENGTH-(i+q), 0);        //turn every third pixel off

      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
        for (int i=0; i < left.numPixels(); i=i+3) {
          left.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
          right.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
          pushbar.setPixelColor(i+q, Wheel( (i+j) % 255));    //turn every third pixel on
          pushbar.setPixelColor(PUSHBAR_LENGTH -(i+q), Wheel( (i+j) % 255));    //turn every third pixel on

        }
        left.show();
        right.show();
        pushbar.show();
       
        delay(wait);
       
        for (int i=0; i < left.numPixels(); i=i+3) {
          left.setPixelColor(i+q, 0);        //turn every third pixel off
          right.setPixelColor(i+q, 0);        //turn every third pixel off
          pushbar.setPixelColor(i+q, 0);        //turn every third pixel off
          pushbar.setPixelColor(PUSHBAR_LENGTH -(i+q), 0);        //turn every third pixel off

          
        }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
   return left.Color(255 - WheelPos * 3, 0, WheelPos * 3);
   return right.Color(255 - WheelPos * 3, 0, WheelPos * 3);
   return pushbar.Color(255 - WheelPos * 3, 0, WheelPos * 3);
   return pushbar.Color(255 - WheelPos * 3, 0, WheelPos * 3);

  } else if(WheelPos < 170) {
    WheelPos -= 85;
   return left.Color(0, WheelPos * 3, 255 - WheelPos * 3);
   return right.Color(0, WheelPos * 3, 255 - WheelPos * 3);
   return pushbar.Color(0, WheelPos * 3, 255 - WheelPos * 3);
   return pushbar.Color(0, WheelPos * 3, 255 - WheelPos * 3);

  } else {
   WheelPos -= 170;
   return left.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
   return right.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
   return pushbar.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
   return pushbar.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
   
  }
}

