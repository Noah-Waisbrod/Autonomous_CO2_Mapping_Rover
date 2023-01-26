/**
 * @file PID.ino
 * @author MATTHEW MURRAY (20mdre@queensu.ca)
 * @brief PID time
 * @version 1
 * @date 2023-01-25
 *
 */


//!NEED TO BE TESTED
// Wheel PWM pin (must be a PWM pin)
int EA = 11;
int EB = 9;

// Wheel direction digital pins
int I1 = 12;
int I2 = 10;
int I3 = 8;
int I4 = 7;


const PI = 3.1415;
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
const double RHO =0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks_L = 0;
volatile long encoder_ticks_R = 0;


// Variable to store estimated angular rate of left wheel [rad/s]
double theta_L = 0.0;
double theta_R = 0.0;

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
        encoder_ticks--;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticks++;
    }
}
void PID(double set, double current){
    double error = set-current;

    while( error > 0.01){

        double dt =0.01;
        int kp =0.01;
        int ki =0.01;
        int kd =0.01;
        double add = add+ error;
        double new_error = error*kp+ error*dt*kd+ add*ki;
        if(new_error>255){
            new_error = 255;
        }
        else if(new_error<-255){
            new_error = -255;
        }
        drive(new_error);
        theta_L = 2.0 * PI * ((double)encoder_ticks_L / (double)TPR) * 1000.0;
        theta_R = 2.0 * PI * ((double)encoder_ticks_R / (double)TPR) * 1000.0;
        error = error- 0.5*(theta_L*RHO+theta_R*RHO);
        encoder_ticks_L = 0;
        encoder_ticks_R = 0;

    }

}
void drive(double pwm){

    if(pwm>0){
        digitalWrite(I1, LOW);
        digitalWrite(I2, HIGH);
        digitalWrite(I3, HIGH);
        digitalWrite(I4, LOW);
    }
    else{
        digitalWrite(I1, HIGH);
        digitalWrite(I2, LOW);
        digitalWrite(I3, LOW);
        digitalWrite(I4, HIGH);
        pwm =-pwm;
    }
    // PWM command to the motor driver
    analogWrite(EA, pwm);
    analogWrite(EB,pwm);



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
    attachInterrupt(digitalPinToInterrupt(SIGNAL_A), decodeEncoderTicks, RISING);

    // Print a message
    Serial.print("Program initialized.");
    Serial.print("\n");
}

void loop()
{
    pid(1,0);

    delay(10000);

    
    
}
// PID loop to set position 
