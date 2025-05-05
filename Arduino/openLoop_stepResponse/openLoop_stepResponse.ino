/*
Step response of RC circuit after
a switched 2.5V step reference input.
The circuit:
* switch connected from pin 2 to GND
* input Vin from pin 9
* output of circuit read at analog pin A0
*/
//PIN SETTINGS
const int yPin = A0; // Analog read pin
const int uPin = 9; // Analog write pin (PWM)
const int switchPin = 2; // input pin for the switch
boolean switchVal = HIGH; // declare initial switch pin state
// Parameter Settings
int time = 0; // initialize time
int uVal = 0; // initialize control input
const float Kp = 4.0;
const float ref = 2.5; //2.5V reference input
// Initial Values for internal signals
float y=0;
float u=0;
float e=0;
float sumY = 0;       // running sum of y
float sumU = 0;       // running sum of u
float avgY = 0;       // average of y
float avgU = 0;       // average of u

void setup() {
pinMode(switchPin, INPUT); //set switch pin to input mode
digitalWrite(switchPin,HIGH); //initialize to start with pull up voltage
Serial.begin(9600);
}
void loop() {
//WAIT FOR SWITCH
while(switchVal == HIGH) // repeat this loop until switch is turned on
// (switchVal will go LOW when switch is turned on)
{
analogWrite(uPin,0);
switchVal = digitalRead(switchPin); // read switch state
}
// READ CIRCUIT OUTPUT
int sensorVal = analogRead(yPin);
// convert to volts
y=sensorVal*(5.0/1023.0);
// calculate error
e=ref-y;
// CONTROLLER
u=Kp*e;
// check that control signal is in range
if (u>5)
{
u=5;
}
if (u<0)
{
u=0;
}
// WRITE CIRCUIT INPUT
uVal=u*(255/5);

// Update running sums and averages
sumY += y;
sumU += u;
avgY = sumY / (time + 1);
avgU = sumU / (time + 1);

analogWrite(uPin,uVal);
// print the results to the serial monitor:
Serial.print(time++);
Serial.print (" ");
Serial.print (y);
Serial.print (" ");
Serial.println(u);
//Serial.print(time++, y, u);
// Serial.print("Sample #: ");
// Serial.print(time++);
// Serial.print(", Output: ");
// Serial.print(y);
// Serial.print(", Input: ");
// Serial.print(u);  // use println() to add a newline at the end
// Serial.print(", Avg Output: ");
// Serial.print(avgY);
// Serial.print(", Avg Input: ");
// Serial.println(avgU);


// WAIT FOR NEXT SAMPLE
delay(100); //sample frequency 10Hz
switchVal = digitalRead(switchPin); // read switch state
}

