#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <Adafruit_AHTX0.h>
#include <SPI.h>
#include <SD.h>
#include <RH_RF95.h>
#include <Adafruit_GPS.h>

// Timing Gate Setup
const int VOLTAGE_PIN = A2;
const float VOLTAGE_THRESHOLD = 1.4;
const float VOLTAGE_UPTHRESHOLD = 0.59;

// OLED setup
Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5

const int buttonAPin = 9;
// Button state tracking
bool lastButtonState = LOW;
bool currentButtonState = LOW;

const int buttonCPin = 5;
// Button state tracking
bool lastButtonCState = HIGH; // Assuming the button is unpressed at startup
bool currentButtonCState = HIGH;


// SSD Card
const int SD_CARD_CS_PIN = 4;

// LoRa setup
// Feather m0 w/wing:
#define RFM95_RST 11  // "A"
#define RFM95_CS  10  // "B"
#define RFM95_IRQ    6    // "D"
#define RFM95_IRQN   digitalPinToInterrupt(RFM69_IRQ )
#define RF95_FREQ 434.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_IRQ);

// Humidity and Temperature Sensor
Adafruit_AHTX0 aht;

// GPS setup
#define GPSSerial Serial1
Adafruit_GPS GPS(&GPSSerial);

#ifdef NMEA_EXTENSIONS
// Create another GPS object to hold the state of the boat, with no
// communications, so you don't need to call Boat.begin() in setup. 
// We will build some fake sentences from the Boat data to feed to 
// GPS for testing.
Adafruit_GPS Boat;
#endif

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO true

uint32_t timer = millis();


// Global Variables
unsigned long objectDetectedTime1 = 0;
unsigned long objectDetectedTime2 = 0;
File logfile;

// Definition of a structure to hold meteorological data
struct MeteoData {
  float temperature;
  float humidity;
  bool isValid; // Indicates if the data is valid (sensor read successfully)
};


// Global variable to track the last update time
unsigned long lastMeteoDataUpdateTime = 0;
MeteoData currentMeteoData;



void setup() {
  delay(1000);

  Serial.begin(115200);
  Serial.println("GATE 2 Setup");

  // Initialize SD Card
  if (!SD.begin(SD_CARD_CS_PIN)) {
    Serial.println("SD Card initialization failed!");
    while (1);
  }

  // Initialize the button pin as input
  pinMode(buttonAPin, INPUT_PULLUP); // Use INPUT if using an external pull-up resistor
  pinMode(buttonCPin, INPUT_PULLUP);

  

  // Initialize OLED
  delay(500);
  if(!display.begin(0x3C, true)) {
    Serial.println("OLED initialization failed");
    while(1);
  }
  display.display(); // Show initial screen
  delay(2000); // Delay 2 seconds

  if (!initLoRa()) {
    Serial.println("LoRa initialization failed!");
    while (1);
  }

  Serial.println("LoRa Initialization OK!");
  

   // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setRotation(1);

  display.setCursor(0,0);
  display.println("LoRa Init OK");
  display.display();
  delay(2000); // Show LoRa init status

  // Initialize Timing Gate Pin
  pinMode(VOLTAGE_PIN, INPUT);

  // Display System Ready Message
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("GATE 2 System Ready");
  display.display();
  delay(1000);

// Initialize the button pin as input
  pinMode(buttonAPin, INPUT_PULLUP); // Use INPUT if using an external pull-up resistor
  pinMode(buttonCPin, INPUT_PULLUP);

  // Custom splash screen
  display.setTextSize(2); // Increase text size for larger font
  display.setTextColor(SH110X_WHITE);
  display.setCursor(10, 28); // Center the text (adjust as needed)
  display.print("GATE 2");
  display.display(); // Display the custom splash screen
  delay(500); // Display for 0.5 seconds

  display.clearDisplay(); // Clear the screen before displaying new text

  // Initialize and read from the AHT20 sensor
  sensors_event_t humidity, temp;
  if (!aht.begin()) {
    Serial.println("Failed to find AHT sensor!");
    while (1) delay(10); // If sensor not found, halt the program
  }
  aht.getEvent(&humidity, &temp); // Get initial data

  // Round temperature and humidity to one decimal place
  float roundedTemp = round(temp.temperature * 10) / 10.0;
  float roundedHumidity = round(humidity.relative_humidity * 10) / 10.0;

  // Display welcome message on OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Welcome to the track!");
  display.print("Meteo-Data: ");
  display.print(roundedTemp);
  display.print(" C and ");
  display.print(roundedHumidity);
  display.println(" %rH.");
  display.println("---------------------");
  display.println("Gate 2. System ready.");
  display.println("---------------------");
  display.println("Waiting for skier.");
  display.display(); // Update the display
  delay(2000);

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data)
  // including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or
  // RMC+GGA since the parser doesn't care about other sentences at this time
  // Set the update rate (uncomment the one you want.)
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_200_MILLIHERTZ); // 5 second update
  // time
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ); // 10 second update time
  // For the parsing code to work nicely and have time to sort thru the data,
  // and print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  // LoRa initialization
  
  // General delay to display Welcome Messages
  delay(5000);
}

bool initLoRa() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    return false;  // Return false if initialization fails
  }
  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    return false;  // Return false if setting frequency fails
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(23, false);
  
  return true;  // Return true if everything is successful
}

void updateDisplayWithVoltage(MeteoData data) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  
  display.println("Welcome to the track!");
  if (data.isValid) {
    display.print("T [C]: ");
    display.println(data.temperature, 1);
    display.print("rH [%]: ");
    display.println(data.humidity, 1);
  } else {
    display.println("Meteo-Data: Updating...");
  }
  display.println("---------------------");
  display.println("Gate 2. Ready.");
  display.println("---------------------");
  
  int sensorValue = analogRead(VOLTAGE_PIN);
  float voltage = sensorValue * (3.0 / 1023.0);
  display.print("Voltage: ");
  display.println(voltage, 2);

  display.display();
}

// Function to initialize the sensor and get meteorological data
MeteoData getMeteoData() {
  sensors_event_t humidityEvent, tempEvent;
  MeteoData data = {0.0, 0.0, false}; // Initialize with default values

  if (!aht.begin()) {
    Serial.println("Failed to find AHT sensor!");
    // Instead of halting the program, return invalid data
    // while (1) delay(10);
    return data; // Return invalid data if sensor is not found
  }

  aht.getEvent(&humidityEvent, &tempEvent); // Get the data from the sensor

  // Update the structure with the read values
  data.temperature = tempEvent.temperature;
  data.humidity = humidityEvent.relative_humidity;
  data.isValid = true; // Data is valid

  return data; // Return the populated data structure
}

// GATE 2: Variables to hold time offset
long timeOffset = 0;
bool isSynced = false;


// GATE 2: Adjust local timestamp using the calculated time offset
unsigned long getAdjustedTime() {
  if (isSynced) {
    return millis() + timeOffset;
  } else {
    return millis(); // Return local time if not synced
  }
}


bool Gate1Passed = false; // Indicates whether Gate 1 has messaged about a skier's passing



void loop() {
  // Current time in milliseconds
  unsigned long currentMillis = millis();

  
  // Check for incoming messages from Gate 1
  if (rf95.available()) {

    
  
    // Buffer to hold the incoming message
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      // Message received, convert to string for easier handling
      String message = String((char*)buf);
      Serial.print("Received: "); Serial.println(message);

      // Check if the message is a LoRa test message from GATE 1
      if (message == "LoRa Ping") {
        Serial.println("LoRa Test message received from GATE 1.");

        // Send acknowledgment back to GATE 1
        String ackMessage = "Ack from GATE 2";
        rf95.send((uint8_t*)ackMessage.c_str(), ackMessage.length());
        rf95.waitPacketSent();
        Serial.println("Acknowledgment sent to GATE 1.");

        // Optionally, update the display to show that a test message was received and acknowledged
        display.clearDisplay();
        display.setCursor(0, 0);
        display.setTextSize(1);
        display.println("LoRa Test Received");
        display.println("Ack Sent to GATE 1");
        display.display();
      }
      else if (message.startsWith("SYNC:")) {
        unsigned long gate1Time = message.substring(5).toInt();
        unsigned long gate2Time = millis();
        
        // Calculate and set the time offset
        timeOffset = gate1Time - gate2Time;
        isSynced = true;
        
        Serial.print("Synced with GATE 1. Time offset: ");
        Serial.println(timeOffset);
      }
      else {
      // Message received, convert to string for easier handling
      String message = String((char*)buf);
      Serial.print("Received from Gate 1: "); Serial.println(message);
      Gate1Passed = true;
      }  
    } 
  }

  // Continuous voltage check for skier detection at Gate 2
  int sensorValue = analogRead(VOLTAGE_PIN);
  float voltage = sensorValue * (3.0 / 1023.0);

  // Display voltage on OLED and Serial Monitor
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Raw: "); display.print(sensorValue);
  display.print(", V: "); display.println(voltage);
  Serial.print("Gate 2 Raw: "); Serial.println(voltage);

  // Detect athlete at Gate 2 based on voltage threshold
  if (Gate1Passed && voltage < VOLTAGE_THRESHOLD) {
    // Process skier detection, e.g., log data, send acknowledgment
    Serial.println("Skier detected at Gate 2 after Gate 1 confirmation.");

    objectDetectedTime2 = millis();

    // Example: When detecting an event, use getAdjustedTime() instead of millis()
    unsigned long eventTime = getAdjustedTime();
    Serial.print("Event detected at adjusted time: ");
    Serial.println(eventTime);

    Serial.println("Athlete passed Gate 2 at: ");
    Serial.println("V = " + String(voltage));
    Serial.println("Time: " + String(objectDetectedTime2));
    Serial.println("Adjusted Time: " + String(eventTime));

    // Skier detected, send acknowledgment back with detection time
    //String ackMessage = "Skier passed Gate 2, detection time: " + String(objectDetectedTime2);
    String ackMessage = "Skier passed Gate 2, detection time: " + String(eventTime);

    // Calculate run duration
    unsigned long runDuration = objectDetectedTime2 - objectDetectedTime1; // Assuming objectDetectedTime1 is received from Gate 1

    // Display run duration
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.println("Run Completed!");
    display.print("Duration: ");
    display.print(runDuration / 1000.0, 2); // Convert milliseconds to seconds and display
    display.println(" seconds");
    display.display();




    rf95.send((uint8_t*)ackMessage.c_str(), ackMessage.length());
    rf95.waitPacketSent();
    Serial.println("Acknowledgment sent back to Gate 1 with time");

    // Optionally, update display to show detection status
    // Update the display
      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(1);
      display.println("GATE 2:");
      display.println("Athlete Detected");
      display.println("----------------");
      display.println("Time: " + String(objectDetectedTime2));
      display.println("----------------");
      display.println("Acknowledgment sent back to GATE 1 with time.");
      display.display();
      delay(5000);

      // Check if it's time to update MeteoData
      if (currentMillis - lastMeteoDataUpdateTime >= 30000) { // 30 seconds have passed
      currentMeteoData = getMeteoData(); // Update the meteo data
      lastMeteoDataUpdateTime = currentMillis; // Reset the timer
      }

  // Update the display with the latest MeteoData and voltage
  updateDisplayWithVoltage(currentMeteoData);

  } else {
    display.println("No object detected.");
  }
  
  // After handling skier detection or on every loop iteration,
 
  delay(5); // Adjust the delay as needed for your application


}
