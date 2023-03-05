
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
// Wheel PWM pin (must be a PWM pin)
int EA = 6; // Right wheel
int EB = 5; // Left wheel

// Wheel direction digital pins
int I1 = 8; // Forwards (R)
int I2 = 7; // Backwards (R)

int I3 = 13; // Backwards (L)
int I4 = 12; // Forwards (L)

int LF = 100;
int LB = 200;
int RF = 300;
int RB = 400;

// Motor PWM command variable [0-255]
byte uL = 0;
byte uR = 0;

// Left wheel encoder digital pins
const byte SIGNAL_A_L = 3;
const byte SIGNAL_B_L = 2;

const byte SIGNAL_A_R = 9;
const byte SIGNAL_B_R = 10;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks_L = 0;
volatile long encoder_ticks_R = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 70;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;

#define MAX_PWM 255
#define MIN_PWM 0

double kp = 250;
double ki = 50;
double kd = 0.0;
double error_L, p_error_L, i_error_L, d_error_L, error_R, p_error_R, i_error_R, d_error_R;
double pwm_L, pwm_R;
double setpoint = 0.70; // m/s
double actual_speed_L,actual_speed_R;

//-----------------------------------Setup-------------------------------
void setup() {
    
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
    Serial.begin(9600);     
    Serial.println(" ");
    Serial.println("Program ready.");
}

//----------------------------------main--------------------------------
void loop() {
  if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }
  
  Serial.print("CO2: ");
  Serial.print(sgp.eCO2);
  Serial.print(" ppm\tTVOC: ");
  Serial.print(sgp.TVOC);
  Serial.println(" ppb");

  delay(1000);
  
}

//-------------------------------Functions------------------------------

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
