#include <Arduino.h>
#include <WiFi.h> 
#include <WiFiClient.h>
#include <String.h>

// Replace with your network credentials
const char* ssid     = "SmartCurtain";
const char* password = "12345678";
// Set web server port number to 80
WiFiServer server(80);

const int LDRPin = 32; 
int LDRValue;

// DHT11 CODE (Temperature and Humidity Sensor)
#include <dht11.h> // Include the dht11 library
const int DHT11PIN = 33; // Set DHT11 to Analog Pin 1
dht11 DHT11; // variable for dht11

// DC MOTOR CODE
// Motor A
int motor1Pin1 = 0;  
int motor1Pin2 = 2;
int enable1Pin = 4; // Replace with the GPIO pin number connected to ENA
// Setting PWM properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8;
// Define a variable to track the motor state
bool motorRunning = false;

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
const int openButtonPin = 25; // Blue Button
//OneButton openButton(25,true);
const int closeButtonPin = 26; // Yellow Button
//OneButton closeButton(26,true); 
int openButtonState = 0;
int closeButtonState = 0;

// Automatic to Manual Switch Button
const int switchButtonPin = 27; // Red Button
const int manualLED = 16;   // Red LED
const int automaticLED = 17;   // Green LED
const unsigned long debounceDelay = 50; // Debounce delay (in milliseconds) 
enum Mode { MANUAL, AUTOMATIC };
Mode currentMode = MANUAL;
unsigned long lastDebounceTime = 0;
bool buttonState = HIGH;
bool lastButtonState = HIGH;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(921600);
  WiFi.softAP(ssid, password); 
  server.begin();
  Serial.println("Access Point started");
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());

  // (HC-SR04 MODULE) ULTRASONIC DISTANCE SENSOR CODE 
  pinMode(trigPin, OUTPUT); //sets pin as OUTPUT
  pinMode(echoPin, INPUT); //sets pin as INPUT

  // LIMIT SWITCH
  pinMode(limitSwitch1, INPUT); //set pin as INPUT
  pinMode(limitSwitch2, INPUT); //set pin as INPUT

  // LED Indicator when the System is Turn On
  pinMode(manualLED, OUTPUT); // onboard LED
  pinMode(automaticLED, OUTPUT); // onboard LED

  pinMode(LDRPin, INPUT); // Set Photoresistor analog pin as INPUT

// FOR DC MOTOR
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  // configure LED PWM functionalitites
  ledcSetup(pwmChannel, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(enable1Pin, pwmChannel);

  pinMode(openButtonPin, INPUT_PULLUP);
  pinMode(closeButtonPin, INPUT_PULLUP);
  pinMode(switchButtonPin, INPUT_PULLUP);

}

void loop() {

  // ANDRID APP CONNECTION AND CONTROL
  String all_command = "";
  WiFiClient client = server.available();
  if (client) {
    String request = "";
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        request += c;
        if (c == '\r') {
          // End of line reached, check if next character is newline

          Serial.println(request);  // full HTTP command line including GET  and HTTP 1

          // Extract command from request string
          int start = request.indexOf("GET /") + 5;
          int end = request.indexOf("HTTP/");
          String command = request.substring(start, end);

          //Purify the command
          command.replace("\n", "");
          command.replace("\r", "");
          command.replace(" ", ""); // removes all space characters
          command.replace("\t", ""); // removes all tab characters
          command.trim();

          // client.print(responseMessage);
          // // Read temperature, humidity, and light values
          // int temperature = DHT11.temperature;
          // int humidity = DHT11.humidity;
          // int lightValue = 0;
          // client.println(temperature);
          // client.println(humidity);
          // client.println(lightValue);
          // // Create a response message that includes the sensor data
          // String responseMessage = "Temperature: " + String(temperature) + " &deg;C<br>";
          // responseMessage += "Humidity: " + String(humidity) + " %<br>";
          // responseMessage += "Light Value: " + String(lightValue) + "<br>";
          
          Serial.println(command);
          all_command =  command + "curtain";
         
          // ONE CLICK OPEN FUNCTION
          // while (command.equals("open")) {
          //     openCurtain();
          //     if(digitalRead(limitSwitch1) == LOW && digitalRead(limitSwitch2) == LOW){
          //       break;
          //     }
          // }

          // // HOLD BUTTON OPEN FUNCTION
          // if (command.equals("open")) {
          //   openCurtain();
          // }

          // HOLD BUTTON OPEN FUNCTION
          if (command.equals("open")) {
            openCurtain();
          }

          if (command.equals("close")) {
            closeCurtain();    
          }

          if (command.equals("switch")) {
            if (currentMode == AUTOMATIC) {
                currentMode = MANUAL;
                digitalWrite(manualLED, HIGH); // Turn off the LED to indicate manual mode
                digitalWrite(automaticLED, LOW); // Turn on the LED to indicate automatic mode
                Serial.println("Set to Manual Mode");
            } else {
                currentMode = AUTOMATIC;
                digitalWrite(automaticLED, HIGH); // Turn on the LED to indicate automatic mode
                digitalWrite(manualLED, LOW); // Turn off the LED to indicate manual mode
                Serial.println("Set to Automatic Mode");
            }
          }

          if (client.peek() == '\n') {
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            
            String commandWithTags = "<html><body>" + all_command + "</body></html>";

            client.println(commandWithTags);
            break;
          }
        }
      }
    }
  }

  // LIGHT AND TEMPERATURE SENSOR FUNCTIONS
  //read and stores the Light Resistance value, then prints it
  LDRValue = analogRead(LDRPin);
  Serial.print("Light Resistance: "); Serial.println(LDRValue);
  Serial.println("\n");
  // read and stores temperature and humidity value, then prints it
  int checkDht11Value = DHT11.read(DHT11PIN);
  Serial.print("Humidity (%): "); Serial.println((float)DHT11.humidity, 2);
  Serial.print("Temperature  (C): "); Serial.println((float)DHT11.temperature, 2);

  /* SETUP of Manual|Automatic Switch */
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
          // Perform your automatic mode actions here
          digitalWrite(automaticLED, HIGH); // Turn on the LED to indicate automatic mode
          digitalWrite(manualLED, LOW); // Turn off the LED to indicate manual mode
          Serial.println("Set to Automatic Mode");
        } else {
          currentMode = MANUAL;
          // Perform your manual mode actions here
          digitalWrite(manualLED, HIGH); // Turn off the LED to indicate manual mode
          digitalWrite(automaticLED, LOW); // Turn on the LED to indicate automatic mode
          Serial.println("Set to Manual Mode");
        }
      }
    }
  }
  lastButtonState = reading;

  // /* LOOP of Manual|Automatic Switch */
  // Your code for MANUAL and AUTOMATIC modes goes here
  if (currentMode == MANUAL) {
      // Code for MANUAL mode
      // read the state of the pushbutton value:
    
    openButtonState = digitalRead(closeButtonPin);
    closeButtonState = digitalRead(openButtonPin);

    // Serial.println("MODE: MANUAL");
    // if (openButton.isIdle()) {  // Handle the Open Button
    //   openCurtain();
    // }else if(closeButton.isIdle()) { // Handle the Close Button
    //   closeCurtain ();
    
    Serial.println("MODE: MANUAL");
    if (openButtonState == LOW) {  // Handle the Open Button
      openCurtain();
    }else if(closeButtonState == LOW) { // Handle the Close Button
      closeCurtain ();
    }else {
      Serial.println("Motor stopped"); 
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, LOW);
      ledcWrite(pwmChannel, 0);  // Turn off PWM
    }  
  } else {
    // Code for AUTOMATIC mode
    Serial.println("MODE: AUTOMATIC");
    int checkDht11Value = DHT11.read(DHT11PIN);
    if(LDRValue > 3000) { 
      openCurtain();
    } else { 
      closeCurtain();
    }
  }
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
      // Move the DC motor forward at maximum speed
      Serial.println("Opening Curtain...");
      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH); 
      ledcWrite(pwmChannel, 255);  // Set duty cycle to 255 (maximum speed)
    }
}
void closeCurtain () {
    distance = getDistance();  //stores the returned value from the function
    if (distance < 20) {
        // Stop the DC motor
        Serial.println("Motor stopped");
        digitalWrite(motor1Pin1, LOW);
        digitalWrite(motor1Pin2, LOW);
        ledcWrite(pwmChannel, 0);  // Turn off PWM
    } else {
        // Move DC motor backwards at maximum speed
        Serial.println("Closing Curtain...");
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


