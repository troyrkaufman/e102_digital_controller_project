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
BLA::Matrix<2,1> K = {4.6334, -0.026};
BLA::Matrix<2,1> L = {-0.4331, -0.9516};

// Observable matrix values
BLA::Matrix<2,1> Ad = {0.7746, -0.08833, 0.08833, 0.9954};
BLA::Matrix<2,1> Bd = {0.08833, 0.004604};
BLA::Matrix<2,1> Cd = {0, 1};
BLA::Matrix<1,1> Dd = {0};

// Initial Values for internal signals
float y=0;
float u=0;
float u_prev=0;
float e=0;
float e_prev=0;
float T = 0.1;

float approxStateVector(float u_val, float y_val, float [2][1] prev_xhat){
    BLA::Matrix<1,1> u = u_val;
    BLA::Matrix<1,1> u = y_val;
    BLA::Matrix<2,1> entry1 = Bd * u;
    BLA::Matrix<2,1> entry2 = (L * u) * (y - ((Cd * u) * prev_xhat));
    BLA::Matrix<2,1> entry3 = (Ad * u) * prev_xhat; 
    BLA::Matrix<2,1> xhat = entry1 + entry2 + entry3;
    return xhat;
}

float controller(float step_val, float Kr_val, float [2][1] control_gain_val, float y_val){


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

    // calculate error for p control
    // e=ref-y;

    // u = Kp*(e-e_prev) + Ki*e*T + u_prev;

    // u_prev = u; // store previous control signal
    // e_prev = e; // store previous error

    // check that control signal is in range
    if (u>5) {
        u=5;
    }
    if (u<0){
        u=0;
    }

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
