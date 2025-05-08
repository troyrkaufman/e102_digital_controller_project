#include <BasicLinearAlgebra.h>
#include <ElementStorage.h>
// optimal_observer_control.ino

//PIN SETTINGS
const int yPin = A0; // Analog read pin
const int uPin = 9; // Analog write pin (PWM)
const int switchPin = 2; // input pin for the switch
boolean switchVal = HIGH; // declare initial switch pin state

// Parameter Settings
int time = 0; // initialize time
int uVal = 0; // initialize control input
const float Kp = 2.3492;
const float Ki = 0.8551;
const float ref = 2.5; //2.5V reference input
const float Kr = 0.974;

// feedback gain and observable gain vectors
BLA::Matrix<1,2> K = {4.6334, -0.026};
BLA::Matrix<2,1> L = {1.7829, 1.0896};//{-0.4331, -0.9516};

// Observable matrix values
BLA::Matrix<2,2> Ad = {0.7746, -0.08833, 0.08833, 0.9954};
BLA::Matrix<2,1> Bd = {0.08833, 0.004604};
BLA::Matrix<1,2> Cd = {0, 1};
BLA::Matrix<1,1> Dd = {0};
BLA::Matrix<2,1> prev_xhat = 0;
BLA::Matrix<2,1> xhat = 0;


BLA::Matrix<1,2> C = {0, 1};

// Initial Values for internal signals
float y=0;
//float u=0;
//float u_prev=0;
//float e=0;
//float e_prev=0;
float T = 0.1;

BLA::Matrix<2,1> approxStateVector(float u_val, float y_val, BLA::Matrix<2,1> prev_xhat){
    BLA::Matrix<1,1> u = u_val;
    BLA::Matrix<1,1> y = y_val;
    BLA::Matrix<2,1> entry1 = Bd * u;
    BLA::Matrix<2,1> entry2 = (L) * (y - (C * prev_xhat));
    BLA::Matrix<2,1> entry3 = (u_val * Ad) * prev_xhat; 
    BLA::Matrix<2,1> xhat = entry1 + entry2 + entry3;
    return xhat;
}

float controller(float step_val, float Kr_val, BLA::Matrix<1,2> K_fb_val, float y_val){
    
    float u = (step_val * Kr_val) + (-K_fb_val * xhat)(0,0);
    xhat = approxStateVector(u, y_val, prev_xhat);
    //BLA::Matrix<2,1> xhat = prev_xhat;
    // Cap u
    if (u > 5) u = 5;
    if (u < 0) u = 0;
    prev_xhat = xhat;
    return u;
}

void setup() {
    pinMode(switchPin, INPUT); //set switch pin to input mode
    digitalWrite(switchPin,HIGH); //initialize to start with pull up voltage
    Serial.begin(9600);
}

void loop() {
    //WAIT FOR SWITCH
    while(switchVal != HIGH) // repeat this loop until switch is turned on
    // (switchVal will go LOW when switch is turned on)
    {
        analogWrite(uPin,0);
        switchVal = digitalRead(switchPin); // read switch state
    }
    // READ CIRCUIT OUTPUT
    int sensorVal = analogRead(yPin);
    
    // convert to volts
    y=sensorVal*(5.0/1023.0);

    // observer design within controller design
    float u = controller(ref, Kr, K, y);

    // WRITE CIRCUIT INPUT
    uVal=u*(255/5);
    analogWrite(uPin,uVal);

    // print the results to the serial monitor:
    Serial.print(time++);
    Serial.print (" ");
    Serial.print (y);
    Serial.print (" ");
    Serial.println(u);
 

    // WAIT FOR NEXT SAMPLE
    delay(100); //sample frequency 10Hz
    switchVal = digitalRead(switchPin); // read switch state
}
