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
  
}

//---------------Functions------------------
//function to check safty distance
boolean SafteyDistance(double frontDist, double sideDist){
    double FD = map(analogRead(sharpF),0,1023,0,3300);
    double LD = map(analogRead(sharpF),0,1023,0,3300);
    double RD = map(analogRead(sharpF),0,1023,0,3300);
    if(FD <= frontDist || LD <= sideDist || RD <= sideDist){
       return true; 
    } else {
        return false;
    }
}


