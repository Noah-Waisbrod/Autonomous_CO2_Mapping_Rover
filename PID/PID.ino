/*

   @file PID.ino
   @author MATTHEW MURRAY (20mdre@queensu.ca)
   @brief PID time
   @version 1
   @date 2023-01-25

*/



// Wheel PWM pin (must be a PWM pin)
int EA = 11;
int EB = 9;

// Wheel direction digital pins
int I1 = 12;
int I2 = 10;
int I3 = 8;
int I4 = 7;


// Left wheel encoder digital pins
const byte SIGNAL_A_L = 5;
const byte SIGNAL_B_L = 6;
// Left wheel encoder digital pins
const byte SIGNAL_A_R = 2;
const byte SIGNAL_B_R = 3;
// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

//Wheel track length
const double ELL = 0.2775;
// Wheel radius [m]
const double RHO = 6.25;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks_L = 0;
volatile long encoder_ticks_R = 0;


// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 1000;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;



// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicks()
{
  if (digitalRead(SIGNAL_B_L) == LOW)
  {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks_L--;

  }
  else
  {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks_L++;
  }
  if (digitalRead(SIGNAL_B_R) == LOW)
  {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticks_R--;

  }
  else
  {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticks_R++;
  }
}
void PID(double setV, double currentV) {
  
  double errorV = setV - currentV;
  double dt = 0.01;

  double kpV = 400;
  double kiV = 0.0;
  double kdV = 0.0;

  double addV = 0;
 
  double last_errorV = errorV;
  double pwmR = 0;
  double pwmL = 0;
  //    Serial.print("errorW\t");
  //    Serial.print(errorW);
  //    Serial.print("\n errorV \t ");
  //    Serial.print(erorV);
  double t_last =0;
  double t_now =0;
  while (errorV > 0.0001||errorV < -0.0001) {
    t_now = millis();

    
      
    addV += errorV * dt;
    Serial.print("errorV:");
    Serial.print(errorV);
    Serial.print(",");
    Serial.print("addV:");
    Serial.print(addV);
    Serial.print(",");
    Serial.print("DTv:");
    Serial.println((errorV - last_errorV) * dt);
    double uV = errorV * kpV + (errorV - last_errorV) * dt * kdV + addV * kiV;
    pwmL = uV;
    pwmR = uV;
//    Serial.print(",");
//    Serial.print("pwmR:");
//    Serial.print(pwmR);
//    Serial.print(",");
//    Serial.print("pwmL:");
//    Serial.println(pwmL);
    drive(pwmL, pwmR);
    omega_L = 2.0 * PI * ((double)encoder_ticks_L / (double)TPR)/(t_now-t_last) * 1000.0;
    omega_R = 2.0 * PI * ((double)encoder_ticks_R / (double)TPR)/(t_now-t_last) * 1000.0;
    
    last_errorV = errorV;
    errorV = errorV - 0.5 * (omega_L * RHO + omega_R * RHO);
    
    
    encoder_ticks_L = 0;
    encoder_ticks_R = 0;
    t_last = t_now;

  }

}
void drive(double pwmL, double pwmR) {


  if (pwmL > 255) {
    pwmL = 255;
  }
  if (pwmR > 255) {
    pwmR = 255;
  }
  if (pwmL < -255) {
    pwmL = -255;
  }
  if (pwmR < -255) {
    pwmR = -255;
  }
  
  if (pwmR > 0) {
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);

  }
  if (pwmL > 0) {
    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
  }
  if (pwmR <= 0) {
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);
    pwmR = -pwmR;

  }
  if (pwmL <= 0) {
    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    pwmL = -pwmL;

  }

  // PWM command to the motor driver
  analogWrite(EA, pwmL);
  analogWrite(EB, pwmR);



}
void setup()
{
  // Open the serial port at 9600 bps
  Serial.begin(9600);

  // Set the pin modes for the motor driver
  pinMode(EA, OUTPUT);
  pinMode(I1, OUTPUT);
  pinMode(I2, OUTPUT);


  // Set the pin modes for the motor driver
  pinMode(EB, OUTPUT);
  pinMode(I3, OUTPUT);
  pinMode(I4, OUTPUT);

  // Set the pin modes for the encoders
  pinMode(SIGNAL_A_L, INPUT);
  pinMode(SIGNAL_B_L, INPUT);
  pinMode(SIGNAL_A_R, INPUT);
  pinMode(SIGNAL_B_R, INPUT);
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A_L), decodeEncoderTicks, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A_R), decodeEncoderTicks, RISING);

}

void loop()
{
  PID(0.5, 0);
  delay(10000);
  PID(0, 0.5);




}
// PID loop to set position
