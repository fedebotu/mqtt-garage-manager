#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266HTTPUpdateServerMod.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <RemoteDebug.h>
#include <string.h>

#define reedSensor D0 // Sensor for closure detection
#define echoPin1 D1 // Ultrasonic sensor input
#define echoPin2 D2 // Ultrasonic sensor input
#define trigPin1 D3 // Ultrasonic sensor output
#define trigPin2 D4 // Ultrasonic sensor output
#define presenceSensor D5 // Check body movements
#define button D6 // Manually send command / enable and disable auto mode
#define led D7 // LED for status
#define rfAntenna D8 // RF 433MHz sender

#define HOST_NAME "Garage_Module"
#define DEBUG_MODE 1
#define BUFFER_SIZE_CAR 5 // Car detection buffer array, useful for avoiding fake detections
#define BUFFER_SIZE_DISTANCE 5 // Array for averaging last n values
#define BUFFER_SIZE_DOOR 5 // Avoids single bit error in reading door closed
#define MAX_DISTANCE 300.00 // Maximum distance for error reporting, since the ceiling is ~3m distances greater than that should be regarded as errors
#define PULSE_DELAY 700 // microseconds between pulses
#define THRESHOLD_DISTANCE 150.00 // in cm, if distance is less than this the car is detected
#define OPENING_TIME 15000 // Time in ms for the motor to open/close the door (we consider the variation to be linear. Best practice would be a feedback such as a tilt sensor to know the motor state
#define CLOSING_FAIL_TIME 30000 // Time in ms after which retry sending command if not open
#define THRESHOLD_DOOR 4 // number of "1" readings over BUFFER_SIZE_DOOR needed for detecting closed door
#define THRESHOLD_carPresence 4 //number of "1" reading over BUFFER_SIZE_CAR needed for detecting car
#define MAX_TRIES 3 // Maximum tries for closing
#define LONG_PRESS 1000 // Time in ms for detecting long press on button
#define BLINK_TIMES 10 // Number of times to blink
#define BLINK_SPEED 300 // Time in ms for changing led state when blinking
//#define PRESS_TIME_REJECT 1000 // Time in ms for detecting multiple presses of the pushbutton
bool ENABLE_LED = 1;
bool AUTO_MODE = 1;
bool ENABLE_DETECT_CAR = 1;

// Update these with values suitable for your network.
const char* ssid = "Berto's 🏡 Garage 📡2.4G";
const char* password = "Alberello";
const char* mqtt_server = "192.168.1.183";
const char* host = "esp8266-webupdate";
const char* update_path = "/firmware";
const char* update_username = "admin";
const char* update_password = "admin";

// Topic list, will be sent over MQTT
const char* door_state_topic = "garage-module/door/state";
const char* door_command_topic = "garage-module/door/set";
const char* carPresence_topic = "garage-module/car/state";
const char* movement_detection_topic = "garage-module/movement";

/*
// Debug through MQTT
const char* ultrasound_sensor_debug1 = "garage-module/car/debug/1";
const char* ultrasound_sensor_debug2 = "garage-module/car/debug/2";
*/


// Code to send to garage door 
boolean garageCode[] = {1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,1,1,
0,0,0,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,1,1,0,0,0,0,
1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,1,1,0,0,0,0,
1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,1,1,0,0,0,0,
1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,1,1,0,0,0,0,
1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,1,1,0,0,0,0,
1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,1,1,0,0,0,0,
1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,1,1,0,0,0,0,
1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,0,0,1,0,1,0,1,0,1,0,1,1,1,1,0,0,0,0};

int i; //counter
int temp; // temporary variable
bool carPresence[BUFFER_SIZE_CAR] = {0}; // Initialize to 0
bool doorOpen[BUFFER_SIZE_DOOR] = {0}; // Initialize to 0
float distance[BUFFER_SIZE_DISTANCE];
long duration;
float finalDistance = 0; // Filtered out distance
int timerStart;
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
int numberOfTries = 0;
int garageCodeSize = sizeof(garageCode);
int buttonTimerStart;
//int buttonLastChangedTime = 0;

struct GarageState{
  bool doorOpening = 0;
  bool doorClosing = 0;
  bool doorClosed = 0; // door is open if = 0
  bool carPresence = 0;
  bool movementDetected = 0;
  float distance1;
  float distance2;
} GarageState;

RemoteDebug Debug;

void DebugSend(String message, String variable, int debugLevel, bool newline){ // sends to debug (both serial and telnet)
  if(DEBUG_MODE){
    if(variable != "null"){
      message = message + variable;
    }
    if(newline){
      Serial.println(message);
    }
    else{
      Serial.print(message);
    } 
  // Send debug info to telnet with different colors
    switch(debugLevel){
      case 0:
        debugV("[VERBOSE] %s", message.c_str()); 
        break;
      case 1:
        debugD("[DEBUG] %s", message.c_str());
        break;
      case 2:
        debugI("[INFO] %s", message.c_str());
        break;
      case 3:
        debugW("[WARNING] %s", message.c_str());
        break;
      case 4:
        debugE("[ERROR] %s", message.c_str());
        break;
      default:
        debugW("[DEFAULT WARNING] %s", message.c_str());
        break;
    } 
  }
}

void DebugUpdateState(){
  DebugSend("[DOOR STATE] Closed: ", (String)GarageState.doorClosed, 1, 1);
  DebugSend("[DOOR STATE] Opening: ", (String)GarageState.doorOpening, 1, 1);
  DebugSend("[DOOR STATE] Closing: ", (String)GarageState.doorClosing, 1, 1);
  DebugSend("[CAR STATE] Presence: ", (String)GarageState.carPresence, 1, 1);
  DebugSend("[MOVEMENT SENSOR] Presence:", (String)GarageState.movementDetected, 1, 1);
  DebugSend("[ULTRASONIC SENSOR 1] Distance read in cm = ", String(GarageState.distance1), 1, 1);
  DebugSend("[ULTRASONIC SENSOR 2] Distance read in cm = ", String(GarageState.distance2), 1, 1);
}

class Leds {
public:
    void turnOn() {
        digitalWrite(led, HIGH);
    }
    void turnOff() {
        digitalWrite(led, LOW);
    }
    void control() {
        if (_blinking) {
            blink();
        }
    }
    void stopBlinking(){
      _blinking = 0;
    }
    void blink() {
        if (!_blinking) { // Initialize variables
            _blinking = 1;
            _counter = 0;
            turnOff();
            _blinkingHighState = 0;
            _ledTimer = millis();
        }
        else {
            if (_counter > BLINK_TIMES) { // Stop blinking
                _blinking = 0;
                _blinkingHighState = 0;
                _counter = 0;
                // Restore led state
                if(AUTO_MODE) digitalWrite(led, HIGH);
                else digitalWrite(led, LOW);
            }
            else {
                if ((millis() - _ledTimer) > BLINK_SPEED) { // Change led state
                    if (_blinkingHighState) {
                      turnOff();
                      _blinkingHighState = 0;
                    }
                    else {
                      turnOn();
                      _blinkingHighState = 1;
                    }
                    _counter++;
                }
            }
        }
    }
private:
    bool _blinkingHighState = 0;
    bool _blinking = 0;
    int _ledTimer = 0;
    int _counter = 0;
};
Leds Led; // Led declaration

class GarageDoor {
public:
    bool checkClosed() {
        temp = 0;
        for (i = 0; i < BUFFER_SIZE_DOOR - 1; i++) {
            doorOpen[i] = doorOpen[i + 1];
            temp += doorOpen[i];
        }
        doorOpen[BUFFER_SIZE_DOOR - 1] = digitalRead(reedSensor);
        temp = temp + doorOpen[BUFFER_SIZE_DOOR - 1];
        if (temp < THRESHOLD_DOOR) {
            if ((!GarageState.doorOpening) and (!GarageState.doorClosing)) {
                client.publish(door_state_topic, "open");
                GarageState.doorClosed = 0;
            }
            return 1;
        }
        else {
            GarageState.doorClosed = 1;
            if (!GarageState.doorOpening) {
                client.publish(door_state_topic, "closed");
            }
            if (GarageState.doorClosing) {
                GarageState.doorClosing = 0;
            }
            return 0;
        }
    }

    void open() {
        if ((checkClosed()) and (!GarageState.doorOpening) and (!GarageState.doorClosing)) {
            sendSignal();
            client.publish(door_state_topic, "opening");
            GarageState.doorOpening = 1;
            timerStart = millis();
        }
        else DebugSend("Door is already open", "null", 4, 1);
    }

    void close() {
        if ((!checkClosed()) and !GarageState.doorOpening and !GarageState.doorClosing) {
            sendSignal();
            client.publish(door_state_topic, "closing");
            GarageState.doorClosing = 1;
            timerStart = millis();
        }
        else DebugSend("Door is already closed", "null", 4, 1);
    }
// MODIFY
    void stop() {
        if (1 == 1){//GarageState.doorClosing) or GarageState.doorOpening) 
            sendSignal();
            checkClosed();
            GarageState.doorClosing = 0;
            GarageState.doorOpening = 0;
        }
        else DebugSend("Door not opening or closing; cannot stop", "null", 4, 1);
    }

    void actions() {
        // We say the door is open after a certain time since it started opening. This is because we have no feedback over total opening of it
        if ((GarageState.doorOpening) and ((millis() - timerStart) > OPENING_TIME)) {
            DebugSend("Door has fully opened", "null", 2, 1);
            client.publish(door_state_topic, "open");
            GarageState.doorOpening = 0;
            GarageState.doorClosed = 0;
        }
        // If the door hasn't closed after a threshold time, then try closing again
        if ((GarageState.doorClosing) and ((millis() - timerStart) > CLOSING_FAIL_TIME)) {
            if (AUTO_MODE) {
                numberOfTries = numberOfTries + 1;
                if (numberOfTries > MAX_TRIES) {
                    DebugSend("An error has occurred, a manual check is required", "null", 4, 1); // Time to check the damn issue
                    numberOfTries = 0; // Reset
                    GarageState.doorClosing = 0; // Change state so we can try other ways
                }
                else {
                    DebugSend("Retrying to close door...", "null", 3, 1);
                    sendSignal(); // Send signal to motor to retry
                    timerStart = millis(); // Resets timer. Door will retry closing indefinetly if it doesn't manage it
                }
            }
            else DebugSend("Auto mode is off. Cannot retry closing automatically", "null", 3, 1);
        }
    }
   // Send signal via 433 MHz antenna
    void sendSignal(){
        for(int i = 0; i < garageCodeSize-1; i++){
          digitalWrite(rfAntenna, garageCode[i]);
          delayMicroseconds(PULSE_DELAY);
        }
        Led.blink();
    }
};
GarageDoor Door;

ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;

void setupWiFi() {
  delay(10);
  // We start by connecting to a WiFi network
  DebugSend("Connecting to ", (String)ssid, 2, 1);
  WiFi.mode(WIFI_AP_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    DebugSend(".", "null", 2, 0);
  }
  randomSeed(micros());
  Serial.println("");
  DebugSend("WiFi connected", "null", 2, 1);
  IPAddress localIp = WiFi.localIP();
  String localIpString = localIp.toString();
  DebugSend("IP address: ", localIpString, 2, 1);
}


void callback(char* topic, byte* payload_byte, unsigned int length) {
  String payload;
  for (int i = 0; i < length; i++) {
    payload = payload + (char)payload_byte[i];
  }
  DebugSend("[MQTT] Message arrived at topic: ", topic, 2, 1);
  DebugSend("[MQTT] Payload: ", payload, 2, 1);

  if(payload == "OPEN"){
    Door.open();
  }
  if(payload == "CLOSE"){
    Door.close();
  }
  if(payload == "STOP"){
    Door.stop();
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    DebugSend("Attempting MQTT connection...", "null", 2, 1);
    String clientId = HOST_NAME;
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      DebugSend("Connected", "null", 2, 1);
      // ... and resubscribe
      client.subscribe(door_command_topic);
      } else {
      DebugSend("Failed, rc = ", String(client.state()), 4, 1);
      DebugSend("Try again in 5 seconds", "null", 3, 1);
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// Measure distance first ultrasonic sensor
void MeasureDistance1(){
  finalDistance = 0;
  for(i = 0; i < BUFFER_SIZE_DISTANCE; i++){
    digitalWrite(trigPin1, LOW);   // Makes trigPin low
    delayMicroseconds(2);       // 2 micro second delay 
    digitalWrite(trigPin1, HIGH);  // tigPin high
    delayMicroseconds(10);      // trigPin high for 10 micro seconds
    digitalWrite(trigPin1, LOW);   // trigPin low
    duration = pulseIn(echoPin1, HIGH);   //Read echo pin, time in microseconds
    //Calculating actual/real distance in cm
    distance[i] = duration*0.034/2;
    if(distance[i] > MAX_DISTANCE){
      // Flatten out output for not messing everything up
      distance[i] = MAX_DISTANCE;
    }
    finalDistance += distance[i]/BUFFER_SIZE_DISTANCE; // calculate average
  }
  GarageState.distance1 = finalDistance;  
}

void MeasureDistance2(){
  finalDistance = 0;
  for(i = 0; i < BUFFER_SIZE_DISTANCE; i++){
    digitalWrite(trigPin2, LOW);   // Makes trigPin low
    delayMicroseconds(2);       // 2 micro second delay 
    digitalWrite(trigPin2, HIGH);  // tigPin high
    delayMicroseconds(10);      // trigPin high for 10 micro seconds
    digitalWrite(trigPin2, LOW);   // trigPin low
    duration = pulseIn(echoPin2, HIGH);   //Read echo pin, time in microseconds
    //Calculating actual/real distance in cm
    distance[i] = duration*0.034/2;
    if(distance[i] > MAX_DISTANCE){
      // Flatten out output for not messing everything up
      distance[i] = MAX_DISTANCE;
    }
    finalDistance += distance[i]/BUFFER_SIZE_DISTANCE; // calculate average
  }
  GarageState.distance2 = finalDistance;  
}

void DetectCar(){
  MeasureDistance1();
  MeasureDistance1();
  temp = 0;
  for(i = 0; i < BUFFER_SIZE_CAR-1; i++){
    carPresence[i] = carPresence[i+1];
    temp += carPresence[i];
  }
  if(GarageState.distance1 < THRESHOLD_DISTANCE && GarageState.distance2 < THRESHOLD_DISTANCE){
    carPresence[BUFFER_SIZE_CAR-1] = 1;
  }
  else {
    carPresence[BUFFER_SIZE_CAR-1] = 0;
  }
  temp = temp + carPresence[BUFFER_SIZE_CAR-1];
  if(temp > THRESHOLD_carPresence){
    client.publish(carPresence_topic, "on");
    GarageState.carPresence = 1;
  }
  else{
    client.publish(carPresence_topic, "off");
    GarageState.carPresence = 0;
  }
  GarageState.carPresence = 0;

}

/* ADD COMMAND TO CHECK GARAGE'S SHELLY CONTROLLER TO BE TURNED ON*/

// Dealing with the interrupt
ICACHE_RAM_ATTR void pushButtonPressed() {
    // Button is pressed
    Led.stopBlinking();
    if (digitalRead(button) == HIGH) {
        buttonTimerStart = millis();
        DebugSend("[BUTTON] pressed", "null", 3, 1);
        // Led.turnOff();
    }
    // Button is released
    else {
        if ((millis() - buttonTimerStart) > LONG_PRESS) {
            if (AUTO_MODE) {
                DebugSend("[BUTTON] Automatic mode is OFF", "null", 3, 1);
                AUTO_MODE = 0;
                Led.turnOff();
            }
            else {
                DebugSend("[BUTTON] Automatic mode is ON", "null", 3, 1);
                AUTO_MODE = 1;
                Led.turnOn();
            }
        }
        // Detect short press and send command to the garage door right away
        else {
            Door.sendSignal(); // Skip the open or closed door. Just send command
            DebugSend("[BUTTON] Sent RF command", "null", 3, 1);
            // Led Blinks already withing SendSignal function
        }
    }
    delay(500);
}

void DetectPresence() {
    if (digitalRead(presenceSensor)) {
        GarageState.movementDetected = 1;
        client.publish(movement_detection_topic, "on");
    }
    else {
        GarageState.movementDetected = 0;
        client.publish(movement_detection_topic, "on");
    }
}


void setup() {
  pinMode(BUILTIN_LED, OUTPUT); // Initialize the BUILTIN_LED pin as an output
  pinMode(presenceSensor, INPUT);
  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);
  pinMode(rfAntenna, OUTPUT);
  pinMode(reedSensor, INPUT);
  pinMode(button, INPUT);
  pinMode(led, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(button), pushButtonPressed, CHANGE);

  Serial.begin(115200);
  setupWiFi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  
  MDNS.begin(host);
  httpUpdater.setup(&httpServer, update_path, update_username, update_password);
  httpServer.begin();
  IPAddress localIp = WiFi.localIP();
  String localIpString = localIp.toString();
  MDNS.addService("http", "tcp", 80);
  DebugSend("[OTA] Ready! Login in your browser at ", localIpString, 2, 1);
  
  // Initialize RemoteDebug
  Debug.begin(HOST_NAME); // Initialize the WiFi server
  Debug.setResetCmdEnabled(true); // Enable the reset command
  Debug.showProfiler(true); // Profiler (Good to measure times, to optimize codes)
  Debug.showColors(true); // Colors

  // Initialize LED
  if (AUTO_MODE) {
      digitalWrite(led, HIGH);
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  DebugUpdateState();
  Debug.handle();
  httpServer.handleClient();
  MDNS.update();
  if(ENABLE_DETECT_CAR){
      DetectCar();
  }
  DetectPresence();
  Door.checkClosed();
  Door.actions();
  Led.control();
}
