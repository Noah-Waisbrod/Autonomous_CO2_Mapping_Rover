
//-------------------------------------
//Names          : Noah Waisbrod, Mustafa Hasan, Matthew Murray
//Student Numbers:
//------------------------------------- 

//Imports---------------------------
#include <Adafruit_NeoPixel.h>

//pin assignments-------------------

//SHARPS
int sharpF = A5;
int sharpL = A4;
int sharpR = A3;

//LED
int PIN = 6;
int NUMPIXELS = 8;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
    
    //SHARP
    pinMode(sharpF, INPUT);
    pinMode(sharpL, INPUT);
    pinMode(sharpR, INPUT);

    //LED
    pixels.begin();

    Serial.begin(9600);     
    Serial.println(" "); 
    Serial.println("Program ready.");
}

void loop() {
  // put your main code here, to run repeatedly:
  pixels.setPixelColor(3,pixels.Color(255, 0, 0));
  pixels.show();
  delay(1000);
  
}

//---------------Functions------------------

//PID Motor
void PID(){
  
  }

//function to check safty distance
boolean SafteyDistance(double frontDist, double sideDist){
    double FD = map(analogRead(sharpF),0,1023,0,3300);
    double LD = map(analogRead(sharpL),0,1023,0,3300);
    double RD = map(analogRead(sharpR),0,1023,0,3300);
    if(FD <= frontDist && LD <= sideDist && RD <= sideDist){
       return true; 
    } else {
       return false;
    }
}

//Function to update the LED stick
void setNeoPixelColor(int color) {
  switch (color) {
    case 1:
      pixels.setPixelColor(0, pixels.Color(255, 0, 0)); // set first pixel to red
      break;
    case 2:
      pixels.setPixelColor(0, pixels.Color(0, 255, 0)); // set first pixel to green
      break;
    case 3:
      pixels.setPixelColor(0, pixels.Color(0, 0, 255)); // set first pixel to blue
      break;
    default:
      // do nothing if the input is not between 1 and 3
      break;
  }
  pixels.show(); // update the NeoPixel stick
}
