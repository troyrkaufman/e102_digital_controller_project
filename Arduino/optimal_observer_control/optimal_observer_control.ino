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
const float ref = 2.5; //2.5V reference input
const float kr = 0.9740;

// feedback gain and observable gain vectors
const float k1 = -4.6334;
const float k2 = -0.0260;

const float l1 = 1.7829;
const float l2 = 1.0896;

const float ad1 = 0.7746;
const float ad2 = -0.08833;
const float ad3 = 0.08833;
const float ad4 = 0.9954;

const float bd1 = 0.08833;
const float bd2 = 0.004604;

const float cd1 = 0;
const float cd2 = 1;

const float dd1 = 0;

float xhat1 = 0;
float xhat2 = 0;

float xhat1_prev = 0;
float xhat2_prev = 0;

// Initial Values for internal signals
float y=0;
const float T = 0.1;

float u = 0;


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

    u = kr*ref -k1*xhat1 - k2*xhat2;

    xhat1 = ad1*xhat1_prev + ad3*xhat2_prev + l1*(y-xhat2_prev) + bd1*u;
    xhat2 = ad2*xhat1_prev + ad4*xhat2_prev + l2*(y-xhat2_prev) + bd2*u;

    xhat1_prev = xhat1;
    xhat2_prev = xhat2; 



    if (u > 5) u = 5;
    if (u < 0) u = 0;

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
