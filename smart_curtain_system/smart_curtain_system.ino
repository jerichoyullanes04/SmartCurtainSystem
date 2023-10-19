#include <Arduino.h> 

// WIFI
#include <WiFi.h> 
const char* ssid = "Smart Curtain";
const char* password = "123456789";

// PHOTORESISTOR CODE (Light Sensor)
const int LDRPin = 32; // The used pin for photoresistor
int LDRValue; // To store the value of light resistance

// LED CODE
const int led = 2; // Set LED to Digital Pin 13

// DHT11 CODE (Temperature and Humidity Sensor)
#include <dht11.h> // Include the dht11 library
const int DHT11PIN = 33; // Set DHT11 to Analog Pin 1
dht11 DHT11; // variable for dht11

// DC MOTOR CODE
// Motor A
int motor1Pin1 = 0; // Replace with the GPIO pin number connected to IN1
int motor1Pin2 = 2; // Replace with the GPIO pin number connected to IN2
int enable1Pin = 4; // Replace with the GPIO pin number connected to ENA
// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;

// (HC-SR04 MODULE) ULTRASONIC DISTANCE SENSOR CODE 
int trigPin = 13; // Set Trigger to Digital Pin 6
int echoPin = 14; // Set Echo to Digital Pin 5
int distance; // to store the value of distance

// LIMIT SWITCH CODE
int limitSwitch1 = 34;  
int limitSwitch2 = 35;
int switchValue1;
int switchValue2;

// PUSH BUTTON CODE
#include "OneButton.h" // Library for single button multi function
// Goal to achieve: Click (Full Open and Full Close), Double Click (Half Open and Half CLose ), and Long Press (Custom Open and Close) Functions
//const int openButtonPin = 25;
OneButton openButton(25,true);
//const int closeButtonPin = 26; 
OneButton closeButton(26,true); 


// Automatic to Manual Switch Button
const int switchButtonPin = 27;
const int ledPin = 23;   // Replace with the GPIO pin number connected to your LED
const unsigned long debounceDelay = 50; // Debounce delay (in milliseconds)
enum Mode { MANUAL, AUTOMATIC };
Mode currentMode = MANUAL;
unsigned long lastDebounceTime = 0;
bool buttonState = HIGH;
bool lastButtonState = HIGH;

void setup() {
  // put your setup code here, to run once:
  
  //WIFI
  WiFi.softAP(ssid, password);

  // (HC-SR04 MODULE) ULTRASONIC DISTANCE SENSOR CODE 
  pinMode(trigPin, OUTPUT); //sets pin as OUTPUT
  pinMode(echoPin, INPUT); //sets pin as INPUT

  // LIMIT SWITCH
  pinMode(limitSwitch1, INPUT); //set pin as INPUT
  pinMode(limitSwitch2, INPUT); //set pin as INPUT

  // LED Indicator when the System is Turn On
  pinMode(led, OUTPUT); // onboard LED
  digitalWrite(led, HIGH); // Switch ON LED in Pin 13

  pinMode(LDRPin, INPUT); // Set Photoresistor analog pin as INPUT

// PUSH BUTTONS
  openButton.attachClick(openCurtainButton);
  closeButton.attachClick(closeCurtainButton);

// FOR DC MOTOR
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);

  pinMode(switchButtonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);

  Serial.begin(921600); // Enable serial and sets baud rate to 115200
}

void loop() {
  // put your main code here, to run repeatedly:

  // SETUP
  int reading = digitalRead(switchButtonPin);
  if (reading != lastButtonState) {
  lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != buttonState) {
      buttonState = reading;
      if (buttonState == LOW) {
        // Button was pressed
        if (currentMode == MANUAL) {
          currentMode = AUTOMATIC;
          digitalWrite(ledPin, HIGH); // Turn on the LED to indicate automatic mode
          Serial.println("Switched to AUTOMATIC mode");
          // Perform your automatic mode actions here
        } else {
          currentMode = MANUAL;
          digitalWrite(ledPin, LOW); // Turn off the LED to indicate manual mode
          Serial.println("Switched to MANUAL mode");
          // Perform your manual mode actions here
        }
      }
    }
  }
  lastButtonState = reading;

  // LOOP
  // Your code for MANUAL and AUTOMATIC modes goes here
  if (currentMode == MANUAL) {
    // Code for MANUAL mode
    
    Serial.println("Motor stopped");
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    ledcWrite(pwmChannel, 0);  // Turn off PWM

    // Handle the Open Button
    openButton.tick();
    // Handle the Close Button
    closeButton.tick();
    
  } else {
    // Code for AUTOMATIC mode
    //read and stores the Light Resistance value, then prints it
    LDRValue = analogRead(LDRPin);
    Serial.print("Light Resistance: ");
    Serial.println(LDRValue);
    // read and stores temperature and humidity value, then prints it
    Serial.println("\n");
    int checkDht11Value = DHT11.read(DHT11PIN);
    Serial.print("Humidity (%): ");
    Serial.println((float)DHT11.humidity, 2);
    Serial.print("Temperature  (C): ");
    Serial.println((float)DHT11.temperature, 2);
    if(LDRValue > 3000) { 
      openCurtain();
    } else { 
      closeCurtain();
    }
  }
}

// MANUAL OPERATION FUNCTIONS
void openCurtainButton () {
  openCurtain();
}

void closeCurtainButton () {
  closeCurtain();
}

// AUTOMATIC OPERATION FUNCTIONS
void openCurtain () {
    if(digitalRead(limitSwitch1) == LOW && digitalRead(limitSwitch2) == LOW) {
        // Stop the DC motor
        Serial.println("Motor stopped");
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, LOW);
        ledcWrite(pwmChannel, 0);  // Turn off PWM
    } else {
      Serial.println("Opening Curtain...");
      // Move the DC motor forward at maximum speed
      Serial.println("Moving Forward");
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH); 
      ledcWrite(pwmChannel, 255);  // Set duty cycle to 255 (maximum speed)
    }
}
void closeCurtain () {
    distance = getDistance();  //stores the returned value from the function
    Serial.print("Distance: ");
    Serial.println(distance);  //prints the stored value
    if (distance < 20) {
        // Stop the DC motor
        Serial.println("Motor stopped");
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, LOW);
        ledcWrite(pwmChannel, 0);  // Turn off PWM
    } else {
        Serial.println("Closing Curtain...");
        // Move DC motor backwards at maximum speed
        Serial.println("Moving Backwards");
        digitalWrite(motor1Pin1, HIGH);
        digitalWrite(motor1Pin2, LOW); 
        ledcWrite(pwmChannel, 255);  // Set duty cycle to 255 (maximum speed)
    }
}
// HC-SR04 Ultrasonic Distance Sensor Function
int getDistance() {
  //sends out a trigger sound
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  //returns the received echo in centimeter
  return pulseIn(echoPin, HIGH)*0.034/2;
}

