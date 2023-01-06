/* 
George Chen, Ryken Santillan
2021/12/14
Traffic Light System Code
*/

// Libraries for Servo and IR Remote
#include <Servo.h> 
#include <IRremote.h> 

// Timer Variables
int time = 0; //running time
int pos = 0; // to assist with modulo algorithm
int d1 = 1500; //specific light timers
int d2 = 1500;
int d3 = 1500;
int d4 = 1500;
int wait = 0; // used for servo timing
bool start = false; // used for servo timing

// Traffic / Pedestrian Light Pins
int greenIntersection1 = 8;
int yellowIntersection1 = 7;
int redIntersection1 = 6;
int greenIntersection2 = 5;
int yellowIntersection2 = 4;
int redIntersection2 = 3;

// Photoresistor Variables
int testLED = 2;
int sensor = 0;

// Button Variables
int button = 9;
int buttonState = 0;
int green = 1500;
int yellow = 1500;
int red = 1500;
bool notPressed = true;
bool notGreen = true;

// Ultrasonic Sensor Variables
int trigPin = 10;
int echoPin = 11;
int trigState = LOW; // trig pin state
int interval = 1; // millis interval for input
unsigned long prevMillisUS;
long dur; // sound wave duration
int dis; // sound wave distance

// Servo Variables
int servoPin = 12;
Servo servo;

// IR Sensor Variables
int receivePin = A1;
IRrecv irrecv(receivePin);
decode_results results;

void setup()
{
  // Initializing Baud Rate
  Serial.begin(9600);
  
  // Initializing LED Pins
  pinMode(greenIntersection1, OUTPUT);
  pinMode(yellowIntersection1, OUTPUT);
  pinMode(redIntersection1, OUTPUT);
  pinMode(greenIntersection2, OUTPUT);
  pinMode(yellowIntersection2, OUTPUT);
  pinMode(redIntersection2, OUTPUT);
 
  // Initializing Ultrasonic Sensor Pins
  pinMode(trigPin, OUTPUT);
  pinMode (echoPin, INPUT);
    
  // GL I1 ON, RL I2 ON, YL I2 OFF, RL I1 OFF
  // initial lighting states: I = intersection G,I,Y = color
  digitalWrite(yellowIntersection2, LOW);
  digitalWrite(redIntersection1, LOW);
  digitalWrite(greenIntersection1, HIGH);
  digitalWrite(redIntersection2, HIGH);
  pos++;
  
  // Initializing Servo Pin and Position
  servo.attach(12);
  servo.write(0); // starts horizontally
  
  //IR SENSOR
  irrecv.enableIRIn();
  
}


void loop()
{
  // Simplified void loop, calls each subsystem
  trafficUpdate();
  photoresUpdate();
  buttonUpdate();
  ultrasonicUpdate();
  servoUpdate();
 // IRUpdate();
  delay(10); // overall delay for synchronous performance
  time += 10; // keeping track of time through delay
}



//-------------- TRAFFIC / PEDESTRIAN LIGHTS ---------------//

void trafficUpdate(){
  //modulo algorithm: uses pos and time var for light timing
  if (time%d1 == 0 && pos%4 == 1){ 
  	// GL1 I1 OFF, YL I1 ON
  	digitalWrite(greenIntersection1, LOW);
  	digitalWrite(yellowIntersection1, HIGH);
    pos++; 
    if (notGreen == true){ // reset traffic timings
    	d1 = 1500;
		d2 = 1500;
		d3 = 1500;
		d4 = 1500;
        notPressed = true;
    }
  }
  else if (time%d2 == 0 && pos%4 == 2){
  	// YL I1, RL I2 OFF, RL I1 and GL I2 ON
  	digitalWrite(yellowIntersection1, LOW);
  	digitalWrite(redIntersection1, HIGH);
  	digitalWrite(redIntersection2, LOW);
  	digitalWrite(greenIntersection2, HIGH);
    pos++;
  }
  else if (time%d3 == 0 && pos%4 == 3){
  	// GL I2 OFF, YL I2 ON
  	digitalWrite(greenIntersection2, LOW);
  	digitalWrite(yellowIntersection2, HIGH);
    pos++;
  }
  else if (time%d4 == 0 && pos%4 == 0){
  	// GL I1 ON, RL I2 ON, YL I2 OFF, RL I1 OFF
  	digitalWrite(yellowIntersection2, LOW);
  	digitalWrite(redIntersection1, LOW);
  	digitalWrite(greenIntersection1, HIGH);
  	digitalWrite(redIntersection2, HIGH);
    pos++;
    time = 0;
    notGreen = true;
  }
}
//----------------------------------------------------------//



//---------------------- PHOTORESISTOR ---------------------//

void photoresUpdate(){
  sensor = analogRead(A0); //reads photoresistor input
  if (sensor < 535)//sensor is past halfway
    digitalWrite(testLED, HIGH);
  else 
    digitalWrite(testLED, LOW);
}
//----------------------------------------------------------//


//------------------------- BUTTON -------------------------//

void buttonUpdate(){
  buttonState = digitalRead(button); // reads button
  
  if (buttonState == HIGH && notPressed == true){ 
    d2 = d2/2; //yellow & red light durations are halved
    d3 = d3/2;
    d4 = d4/2;
    d1 = d1*1.5; // green light remains on 50% longer
    if (digitalRead(greenIntersection1 == HIGH)) { 
    	notGreen = false;
    }
    notPressed = false;
  }
}
//---------------------------------------------------------//



//---------------------- ULTRASONIC -----------------------//

void ultrasonicUpdate(){
  	if(time%500 == 0){ // reads every 500 ms
    	digitalWrite(trigPin, LOW);
		digitalWrite(trigPin, HIGH);
    	digitalWrite(trigPin, LOW);
		dur = pulseIn(echoPin, HIGH); // wave duration as input
		dis = (dur*0.03); // converts duration to distance
    }
}

//--------------------------------------------------------//



//------------------------ SERVO -------------------------//

void servoUpdate(){
    // if it detects an object within required distance
	if ((dis >= 150 && dis <= 200) && start == false){
      start = true; // start the servo waiting time
    }
  
    // servo waits
    if (start == true){
      wait++;
    }
  
  	if (wait == 200){
      servo.write(60); // after waiting 2s, servo opens
    } else if (wait == 350){ // after waiting 1.5s, servo closes
      servo.write(0); // servo is back to horizontal position
      start = false; // resets bool for multiple operations
      wait = 0; // resets wait for multiple operations
    }
}

//--------------------------------------------------------//


//----------------------- IR SENSOR ----------------------//

void IRUpdate(){
	if(irrecv.decode(&results)){ // decodes the pressed button
    	if (results.value == 0xFD00FF){ // if power is pressed
         	servoUpdate(); 
      		irrecv.resume(); // continues reading/decoding
    	}  
    }
}
//--------------------------------------------------------//
