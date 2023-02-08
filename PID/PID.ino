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

/* pass through set speed, current speed, set turn rate current turn rate */
void PID(double setV, double currentV,double setW, double currentW)
{
  double errorL = setV + setW - (currentV+currentW) ; //find speed error
  double errorR = setV - setW - (currentV-currentW) ; //find turn rate error
  double dt = 0.01;// set some time delta for dt term

  double kpL = 200;// set kp for speed
  double kiL = 0.0;// set ki for speed
  double kdL = 0.0;// set kd for speed
  double kpR = 200;// set kp for Turn rate
  double kiR = 0.0;// set ki for Turn rate
  double kdR = 0.0;// set kd for Turn rate

  double addL = 0;//set integrating term for speed
  double addR = 0;//set integrating term for Turn rate

  double last_errorL = errorL;
  double last_errorR= errorR;

  double pwmR = 0;
  double pwmL = 0;
  
  double t_last =0;
  double t_now =0;
  double uL = 0;
  float uR = 0;
  while (true) {
    t_now = millis();
    dt =t_now-t_last;
    
      
    addL += errorL * dt;
    addR += errorR * dt;

    // Serial.print("errorL:");
    // Serial.print(errorL);
    // Serial.print(",");
    // Serial.print("addL:");
    // Serial.print(addL);
    // Serial.print(",");
    // Serial.print("DTL:");
    // Serial.println((errorL - last_errorL) * dt);
    uL = errorL * kpL + (errorL - last_errorL) * dt * kdL + addL * kiL; //pid calc for speed
    uR = errorR * kpR + (errorR - last_errorR) * dt * kiR + addR * kiR; //pid calc for turn rate

    pwmL = uL;
    pwmR = uR;
//    Serial.print(",");
//    Serial.print("pwmR:");
//    Serial.print(pwmR);
//    Serial.print(",");
//    Serial.print("pwmL:");
//    Serial.println(pwmL);
    drive(pwmL, pwmR);
    omega_L = 2.0 * PI * ((double)encoder_ticks_L / (double)TPR)/(t_now-t_last) * 1000.0;
    omega_R = 2.0 * PI * ((double)encoder_ticks_R / (double)TPR)/(t_now-t_last) * 1000.0;
    
    last_errorL = errorL;
    last_errorR = errorR;

    errorL= errorL - omega_L * RHO ; //todo check if one of these should go backwards becasue of encoder
    errorL= errorL - omega_R * RHO ;

    
    
    encoder_ticks_L = 0;
    encoder_ticks_R = 0;
    t_last = t_now;
    if(t_now>10000){ //after ten seconds leave pid loop
      break;
    }
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
  delay(600000);
  PID(0, 0.5);




}
// PID loop to set position
