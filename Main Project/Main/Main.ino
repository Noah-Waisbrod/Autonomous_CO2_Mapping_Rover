
//-------------------------------------
//Names          : Noah Waisbrod, Mustafa Hasan, Matthew Murray
//Student Numbers: 20274009                      20269600
//Sources        : Thomas Sears (thomas.sears@queensu.ca)(PI-Controller)
//------------------------------------- 

//---------------------------------Imports----------------------------------
#include <ros.h> //this must be first import
#include <std_msgs/Int32.h> // pass CO2 out as float32
#include <geometry_msgs/Point.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <Adafruit_SGP30.h>


//------------------------------Location setups-----------------------------

int curPos[] = [0,0]; //x,y
int next[] = [0,0];//dist,angle


//------------------------------ros node setups-----------------------------
ros::NodeHandle nh;

std_msgs::Int32 CO2_out;//pass CO2 out as Float32 
ros::Publisher chatter("co2_topic", &CO2_out);
/**
 * @brief Callback for Ros drive_to_topic subscirber
 * 
 */
void positionCB( const geometry_msgs/Point.h & point){
  int x = int(point.x);   // x
  int y = int(point.y);   //y
  int dx = curPos[0]-x;
  int dy = curPos[1]-y;
  float dis = sqrt(pow(dx,2)+pow(dy,2));
  float angle = atan(dy/dx);
  next =[dis,angle];

}
/**
 * @brief kill callback if kill is pushed loop so robot stops
 * 
 */
void killCB(const std_msgs/Int32.h &kill);
ros::Subscriber<geometry_msgs/Point.h> sub("drive_to_topic", &positionCB );

ros::Subscriber<std_msgs/Int32.h> sub("Kill", &Kill );
//------------------------------pin assignments-----------------------------



//----SHARPS----
int sharpF = A5;
int sharpL = A4;
int sharpR = A3;

//----LED----
int PIN = 2;
int NUMPIXELS = 8;
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_RGBW + NEO_KHZ800);

//----CO2----
Adafruit_SGP30 sgp;

//----wheels----
int EB = 5; // Wheel PWM pin (must be a PWM pin)
int I3 = 3; // Wheel direction digital pin 1
int I4 = 4; // Wheel direction digital pin 2
int EA = 6; // Wheel PWM pin (must be a PWM pin)
int I1 = 8; // Wheel direction digital pin 1
int I2 = 7; // Wheel direction digital pin 2
const byte SIGNAL_AL = 11; // green wire
const byte SIGNAL_BL = 12; // yellow wire
const byte SIGNAL_AR = 9;  // green wire
const byte SIGNAL_BR = 10; // yellow wire

const int TPR = 3000;         // Encoder ticks per (motor) revolution (TPR)
const double RHO = 0.0625;    // Wheel radius [m]
const double ELL = 0.2775;    // Vehicle track [m]
const int T = 100;            // Sampling interval for measurements in milliseconds
const double KP = 200.0;      // Proportional gain
const double KI = 100.0;      // Integral gain

// Motor PWM command variables [0-255]
short u_L = 0;
short u_R = 0;
// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks_L = 0;
volatile long encoder_ticks_R = 0;
// Variables to store estimated angular rates of wheels [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;
// Variables to store estimated wheel speeds [m/s]
double v_L = 0.0;
double v_R = 0.0;
// Variables to store vehicle speed and turning rate
double v = 0.0;     // [m/s]
double omega = 0.0; // [rad/s]
// Variables to store desired vehicle speed and turning rate
double v_d = 0.0;     // [m/s]
double omega_d = 0.0; // [rad/s]
// Variable to store desired wheel speeds [m/s]
double v_Ld = 0.0;
double v_Rd = 0.0;
// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;
// Variables to store errors for controller
double e_L = 0.0;
double e_R = 0.0;
double e_Lint = 0.0;
double e_Rint = 0.0;

//-----helper function-------
void killCB(const std_msgs/Int32.h &kill){
  if (int(kill) = 1){
    digitalWrite(I1, LOW);//stop all motors
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, LOW);
  }
  else{
    return;
  }

}
// This function is called when SIGNAL_AL (left encoder) goes HIGH
void decodeEncoderTicks_L()
{
    if (digitalRead(SIGNAL_BL) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticks_L--;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticks_L++;
    }
}

// This function is called when SIGNAL_AR (right encoder) goes HIGH
void decodeEncoderTicks_R()
{
    if (digitalRead(SIGNAL_BR) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticks_R++;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticks_R--;
    }
}

//-----------------------------------Setup-------------------------------
void setup() {
    //start
    Serial.begin(9600);     
    Serial.println(" ");
    //ros
    nh.initNode();
    nh.advertise(chatter);


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

    //PID
    // Configure digital pins for output
    pinMode(EA, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);

    // Configure digital pins for output
    pinMode(EB, OUTPUT);
    pinMode(I3, OUTPUT);
    pinMode(I4, OUTPUT);

    // Set the pin modes for the encoders
    pinMode(SIGNAL_AL, INPUT);
    pinMode(SIGNAL_BL, INPUT);
    pinMode(SIGNAL_AR, INPUT);
    pinMode(SIGNAL_BR, INPUT);

    // Send 0 PWM commands
    analogWrite(EA, 0);
    analogWrite(EB, 0);

    // Send brake signals to motor driver
    digitalWrite(I1, LOW);
    digitalWrite(I2, LOW);
    digitalWrite(I3, LOW);
    digitalWrite(I4, LOW);

    // Every time the pin goes high, this is a pulse; enable the interrupts
    attachInterrupt(digitalPinToInterrupt(SIGNAL_AL), decodeEncoderTicks_L, RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_AR), decodeEncoderTicks_R, RISING);
    
    //Start 
    Serial.println("Program ready.");
}

//----------------------------------main--------------------------------
void loop() {
  
  PIControler(next[0],next[1]);
  chatter.publish(getCO2());
  nh.spinOnce();
}

//-------------------------------Functions------------------------------

//-----------CO2-----------
/**
 * @brief Function to get the CO2 reading
 *
 * 
 * @return int 
 */
int getCO2(){
  if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }
  return sgp.eCO2;
  }

/**
 * @brief  Function to get the TVOC (Total Voletile Oganic Comounds) reading
 * 
 */

int getTVOC(){
  if (! sgp.IAQmeasure()) {
    Serial.println("Measurement failed");
    return;
  }
  return sgp.TVOC;
  }
/**
 * 
 * @brief function to check safty distance
 * -----------SHARP-----------
 * @param frontDist 
 * @param sideDist 
 * @return boolean 
 */
boolean SafteyDistance(double frontDist, double sideDist){
      double FD = map(analogRead(sharpF),0,1023,0,3300);
      double LD = map(analogRead(sharpL),0,1023,0,3300);
      double RD = map(analogRead(sharpR),0,1023,0,3300);
    
    for(int i = 0; i < 10; i++){
      FD = (FD + map(analogRead(sharpF),0,1023,0,3300))/2;
      LD = (LD + map(analogRead(sharpL),0,1023,0,3300))/2;
      RD = (RD + map(analogRead(sharpR),0,1023,0,3300))/2;
      delay(10);
    }
    if(FD <= frontDist && LD <= sideDist && RD <= sideDist){
       return true; 
    } else {
       return false;
    }
}

//-----------LED-----------
//
//
/**
 * @brief Function to update the LED stick
 * 
 * @param colour 
 *      0 = OFF;
 *      1 = GREEN;
 *      2 = RED;
 *      3 = BLUE;
 */
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

//-------------------------------PID------------------------------
//-----------Helper Functions-----------

/**
 * @brief This function applies PWM inputs (u_L and u_R) to the right and left wheels
 * 
 * @param u_L 
 * @param u_R 
 */
void driveVehicle(short u_L, short u_R)
{
    // LEFT WHEEL
    if (u_L < 0) // If the controller calculated a negative input...
    {
        digitalWrite(I3, HIGH); // Drive backward (left wheels)
        digitalWrite(I4, LOW);  // Drive backward (left wheels)

        analogWrite(EB, -u_L); // Write left motors command
    }
    else // the controller calculated a positive input
    {
        digitalWrite(I3, LOW);  // Drive forward (left wheels)
        digitalWrite(I4, HIGH); // Drive forward (left wheels)

        analogWrite(EB, u_L); // Write left motors command
    }

    // RIGHT WHEEL
    if (u_R < 0) // If the controller calculated a negative input...
    {
        digitalWrite(I1, LOW);  // Drive backward (right wheels)
        digitalWrite(I2, HIGH); // Drive backward (right wheels)

        analogWrite(EA, -u_R); // Write right motors command
    }
    else // the controller calculated a positive input
    {
        digitalWrite(I1, HIGH); // Drive forward (right wheels)
        digitalWrite(I2, LOW);  // Drive forward (right wheels)

        analogWrite(EA, u_R); // Write right motors command
    }
}


/**
 * @brief Compute the wheel rate from elapsed time and encoder ticks [rad/s]
 * 
 * @param encoder_ticks 
 * @param delta_t 
 * @return double 
 */
double compute_wheel_rate(long encoder_ticks, double delta_t)
{
    double omega;
    omega = 2.0 * PI * ((double)encoder_ticks / (double)TPR) * 1000.0 / delta_t;
    return omega;
}


/**
 * @brief Compute wheel speed [m/s]
 * 
 * @param omega_wheel 
 * @return double 
 */
double compute_wheel_speed(double omega_wheel)
{
    double v_wheel;
    v_wheel = omega_wheel * RHO;
    return v_wheel;
}

/**
 * @brief Compute vehicle speed [m/s]
 * 
 * @param  v_L 
 * @param  v_R 
 * @return double 
 */
double compute_vehicle_speed(double v_L, double v_R)
{
    double v;
    v = 0.5 * (v_L + v_R);
    return v;
}


/**
 * @brief Compute vehicle turning rate [rad/s]
 * 
 * @param v_L 
 * @param v_R 
 * @return double 
 */
double compute_vehicle_rate(double v_L, double v_R)
{
    double omega;
    omega = 1.0 / ELL * (v_R - v_L);
    return omega;
}


/**
 * @brief Compute v_L from v and omega
 * 
 * @param v 
 * @param omega 
 * @return double 
 */
double compute_L_wheel_speed(double v, double omega)
{
    double v_wheel = 0.0;
    v_wheel = v - ELL / 2.0 * omega;
    return v_wheel;
}


/**
 * @brief Compute v_R from v and omega
 * 
 * @param v 
 * @param omega 
 * @return double 
 */
double compute_R_wheel_speed(double v, double omega)
{
    double v_wheel = 0.0;
    v_wheel = v + ELL / 2.0 * omega;
    return v_wheel;
}


/**
 * @brief Wheel speed PI controller function
 * 
 * @param e_now 
 * @param e_int 
 * @param k_P 
 * @param k_I 
 * @return short 
 */
short PI_controller(double e_now, double e_int, double k_P, double k_I)
{
    short u;
    u = (short)(k_P * e_now + k_I * e_int);

    // Saturation (i.e., maximum input) detection
    if (u > 255)
    {
        u = 255;
    }
    else if (u < -255)
    {
        u = -255;
    }
    return u;
}


//-------------control-------------
/**
 * @brief PIControler
 * 
 * @param vd 
 * @param od 
 */
void PIControler(double vd, double od){
    // Get the elapsed time [ms]
    t_now = millis();

    // Perform control update every T milliseconds
    if (t_now - t_last >= T)
    {

        // Set the desired vehicle speed and turning rate
        v_d = vd;     // [m/s]
        omega_d = od; // [rad/s]

        // Estimate the rotational speed of each wheel [rad/s]
        omega_L = compute_wheel_rate(encoder_ticks_L, (double)(t_now - t_last));
        omega_R = compute_wheel_rate(encoder_ticks_R, (double)(t_now - t_last));

        // Compute the speed of each wheel [m/s]
        v_L = compute_wheel_speed(omega_L);
        v_R = compute_wheel_speed(omega_R);

        // Compute the speed of the vehicle [m/s]
        v = compute_vehicle_speed(v_L, v_R);

        // Compute the turning rate of the vehicle [rad/s]
        omega = compute_vehicle_rate(v_L, v_R);

        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        encoder_ticks_L = 0;
        encoder_ticks_R = 0;

        // Compute the desired wheel speeds from v_d and omega_d
        v_Ld = compute_L_wheel_speed(v_d, omega_d);
        v_Rd = compute_R_wheel_speed(v_d, omega_d);

        // Compute errors
        e_L = v_Ld - v_L;
        e_R = v_Rd - v_R;

        // Integrate errors with anti-windup
        if (abs(u_L) < 255)
        {
            e_Lint += e_L;
        }
        if (abs(u_R) < 255)
        {
            e_Rint += e_R;
        }

        // Compute control signals using PI controller
        u_L = PI_controller(e_L, e_Lint, KP, KI);
        u_R = PI_controller(e_R, e_Rint, KP, KI);

        // Drive the vehicle
        driveVehicle(u_L, u_R);

    }
}

/**
 * @brief driveDistance
 * 
 * @param distance 
 */
//input distance in [m]
void driveDistance(double distance){
  int t1 = distance*0.8/0.8;
  int t2 = distance*0.2/0.4;
  PIControler(0.8,0);
  delay(t1*1000);
  PIControler(0.4,0);
  delay(t2*1000);
  PIControler(0,0);
  delay(1000);
  }

/**
 * @brief turnAngle
 * 
 * @param ang 
 */
//input angle in [deg]
void turnAngle(double ang){
  double rad = ang*3.14/180
  int t1 = rad*0.8/4;
  int t2 = distance*0.2/2;
  PIControler(4,0);
  delay(t1*1000);
  PIControler(2,0);
  delay(t2*1000);
  PIControler(0,0);
  delay(1000);
  }