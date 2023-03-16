
//-------------------------------------
//Names          : Noah Waisbrod, Mustafa Hasan, Matthew Murray
//Student Numbers: 20274009
//------------------------------------- 

//---------------------------------Imports----------------------------------
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_SGP30.h>
//------------------------------pin assignments-----------------------------

//-SHARPS-
int sharpF = A5;
int sharpL = A4;
int sharpR = A3;

//-LED-
int PIN = 6;
int NUMPIXELS = 8;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGBW + NEO_KHZ800);

//-CO2-
Adafruit_SGP30 sgp;

//-wheels-


//-----------------------------------Setup-------------------------------
void setup() {
    //start
    Serial.begin(9600);     
    Serial.println(" ");
    
    //SHARP
    pinMode(sharpF, INPUT);
    pinMode(sharpL, INPUT);
    pinMode(sharpR, INPUT);

    //LED
    pixels.begin();
    pixels.setBrightness(50);

    //CO2 (sensor to warm up)
    if (! sgp.begin()) {
      Serial.println("SGP30 not found :(");
      while (1);
    }
    Serial.println("Found SGP30");
    for (int i = 0; i < 15; i++) {
      Serial.print(".");
      delay(1000);
    }
    //Start 
    Serial.println("Program ready.");
}

//----------------------------------main--------------------------------
void loop() {
  

  delay(1000);
  
}

//-------------------------------Functions------------------------------

//Function to get the CO2 reading
int getCO2(){
  if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }
  return sgp.eCO2;
  }

//Function to get the TVOC (Total Voletile Oganic Comounds) reading
int getTVOC(){
  if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }
  return sgp.TVOC;
  }

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
// 0 = OFF
// 1 = GREEN
// 2 = RED
// 3 = BLUE
void setNeoPixelColor(int colour) {
  for(int i = 0; i < 8; i++){
    if(colour == 1){
      pixels.setPixelColor(i, pixels.Color(255, 0, 0));
      } else if(colour == 2){
        pixels.setPixelColor(i, pixels.Color(0, 255, 0));
        } else if(colour == 0){
          pixels.setPixelColor(i, pixels.Color(0, 0, 0));
          } else {
            pixels.setPixelColor(i, pixels.Color(0, 0, 255));
            }
    }
  pixels.show(); // update the NeoPixel stick
}
