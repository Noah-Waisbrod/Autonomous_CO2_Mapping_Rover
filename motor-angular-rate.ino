/**
 * @file motor-angular-rate.ino
 * @author MATTHEW MURRAY (20mdre@queensu.ca)
 * @brief Making it drive
 * @version 1
 * @date 2023-01-25
 *
 * 
 *
 */

// Wheel PWM pin (must be a PWM pin)
int EA = 11;
int EB = 9;

// Wheel direction digital pins
int I1 = 12;
int I2 = 10;
int I3 = 8;
int I4 = 7;

// Motor PWM command variable [0-255]
byte u = 0;

// Left wheel encoder digital pins
const byte SIGNAL_A_L = 5;
const byte SIGNAL_B_L = 6;
// Left wheel encoder digital pins
const byte SIGNAL_A_R = 2;
const byte SIGNAL_B_R = 3;
// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;
double omega_R = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 1000;

// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;
short speed = 0;
short turn = 0;


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
    // Get the elapsed time [ms]
    t_now = millis();

    if (t_now - t_last >= T)
    {
        // Estimate the rotational speed [rad/s]
        omega_L = 2.0 * PI * ((double)encoder_ticks_L / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        omega_R = 2.0 * PI * ((double)encoder_ticks_R / (double)TPR) * 1000.0 / (double)(t_now - t_last);

        // Print some stuff to the serial monitor
        Serial.print("Encoder ticks: ");
        Serial.print(encoder_ticks);
        Serial.print("\t");
        Serial.print("Estimated left wheel speed: ");
        Serial.print(omega_L);
        Serial.print(" rad/s");
        Serial.print("\n");
        Serial.print(omega_L*RHO);
        Serial.print(" m/s");
        Serial.print("\n");
        // Record the current time [ms]
        speed = 0.5*(omega_L*RHO+omega_R*RHO);
        turn = 0.5*(omega_L*RHO-omega_R*RHO);
        Serial.print(speed);
        Serial.print(" m/s");
        Serial.print("\n");
        Serial.print(turn);
        Serial.print(" m/s");
        Serial.print("\n");
        t_last = t_now;

        // Reset the encoder ticks counter
        encoder_ticks = 0;
    }

    // Set the wheel motor PWM command [0-255]
    u = 128;
    
    // Select a direction
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);

    // PWM command to the motor driver
    analogWrite(EA, u);
}
