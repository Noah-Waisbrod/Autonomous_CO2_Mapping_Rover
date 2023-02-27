//-------------------------------------
//Names          : Noah Waisbrod, Mustafa Hasan, Matthew Murray
//Student Numbers:
//------------------------------------- 

//Imports---------------------------

//pin assignments-------------------

//SHARPS
int sharpF = A5;
int sharpL = A4;
int sharpR = A3;

void setup() {

    //SHARP
    pinMode(sharpF, INPUT);
    pinMode(sharpL, INPUT);
    pinMode(sharpR, INPUT);

    Serial.begin(9600);     
    Serial.println(" "); 
    Serial.println("Program ready.");
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(SafteyDistance(900.0,900.0));
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
