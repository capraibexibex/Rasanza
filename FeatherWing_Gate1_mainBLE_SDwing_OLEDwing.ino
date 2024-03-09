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
const int VOLTAGE_PIN = A3;
const float VOLTAGE_THRESHOLD = 1.4;
const float VOLTAGE_UPTHRESHOLD = 2.5;

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
const int SD_CARD_CS_PIN = 13;

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

void setup() {
  delay(1000);

  Serial.begin(115200);
  Serial.println("GATE 1 Setup");

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
  digitalWrite(VOLTAGE_PIN, LOW);

  // Display System Ready Message
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("GATE 1 System Ready");
  display.display();
  delay(1000);

// Initialize the button pin as input
  pinMode(buttonAPin, INPUT_PULLUP); // Use INPUT if using an external pull-up resistor
  pinMode(buttonCPin, INPUT_PULLUP);

  // Custom splash screen
  display.setTextSize(2); // Increase text size for larger font
  display.setTextColor(SH110X_WHITE);
  display.setCursor(10, 28); // Center the text (adjust as needed)
  display.print("GATE 1");
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
  displayWelcomeMessage(roundedTemp, roundedHumidity);
  delay(2000);

  // Initialize GPS
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
  delay(1000); // Wait a bit for the GPS to get a fix

  // Ask for firmware version
  GPSSerial.println(PMTK_Q_RELEASE);

  // SD initialization
  if (!initSDCard()) {
    Serial.println("SD Card or Logfile initialization failed!");
    while (1);
  }
  
  // Display the welcome message along with the GPS location
  displayWelcomeMessageWithGPS(roundedTemp, roundedHumidity);


  // General delay to display Welcome Messages
  delay(5000);
}


void displayWelcomeMessageWithGPS(float roundedTemp, float roundedHumidity) {
    int attempts = 0;
    bool gpsFixed = false;
    String gpsLocation = "No GPS fix";

    while (attempts < 2 && !gpsFixed) {
        char c = GPS.read(); // Read GPS data
        if (GPSECHO && c) {
            Serial.print(c);
        }
        if (GPS.newNMEAreceived()) {
            if (GPS.parse(GPS.lastNMEA())) { // Successfully parsed a new NMEA sentence
                if (GPS.fix) {
                    gpsFixed = true;
                    // Update gpsLocation with latitude, longitude, and altitude
                    gpsLocation = "Lat: " + String(GPS.latitude, 4) + GPS.lat + ", ";
                    gpsLocation += "Lon: " + String(GPS.longitude, 4) + GPS.lon + ", ";
                    gpsLocation += "Alt: " + String(GPS.altitude) + "m";
                }
            }
        }
        attempts++;
        delay(100); // Wait a bit before the next read attempt
    }

    // Now update the OLED display with all the information
    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextSize(1); // Set text size to normal for better readability

    // Print the welcome message and meteorological data
    display.println("Welcome to the track!");
    display.print("Meteo-Data: ");
    display.print(roundedTemp);
    display.print(" C and ");
    display.print(roundedHumidity);
    display.println(" %rH.");
    display.println("---------------------");
    display.println("Gate 1. System Ready.");
    display.println("---------------------");

    // Print the GPS location or no-fix message on the last line
    display.setCursor(0, display.height() - 8); // Adjust cursor to the last line
    display.println(gpsLocation);
    display.display(); // Update the display with new information

    if (!gpsFixed) {
        Serial.println("Failed to obtain GPS Fix.");
    }
}


void displayWelcomeMessage(float roundedTemp, float roundedHumidity) {
  display.clearDisplay(); // Clear the display before showing new information
  display.setCursor(0, 0); // Set the cursor at the top left corner
  display.setTextSize(1); // Set text size to normal for better readability

  // Print the welcome message and meteorological data
  display.println("Welcome to the track!");
  display.print("Meteo-Data: ");
  display.print(roundedTemp);
  display.print(" C and ");
  display.print(roundedHumidity);
  display.println(" %rH.");
  
  // Print the status of Gate 1
  display.println("---------------------");
  display.println("Gate 1. System Ready.");
  display.println("---------------------");
  display.display(); // Make sure to update the display with new information
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
  display.println("Gate 1. Ready.");
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

bool initSDCard() {
  if (!SD.begin(SD_CARD_CS_PIN)) {
    return false;
  }
  logfile = SD.open("data.txt", FILE_WRITE);
  if (!logfile) {
    return false;
  }
  logfile.println("Time1,Time2,Velocity,Temperature,Humidity");
  Serial.println("Logfile created.");
  delay(500);
  return true;
}


bool updateDisplay = false;
bool athletePassedGate1 = false;  // Global flag
bool gate2DataReceived = false;

void detectAthletePassing() {
  Serial.println("Detection of Athlete:");
  Serial.println(athletePassedGate1);
  
  int sensorValue = analogRead(VOLTAGE_PIN);
  float voltage = sensorValue * (3.0 / 1023.0);
  Serial.println("V = " + String(voltage));


  if (!athletePassedGate1) {
    
    if (voltage < VOLTAGE_THRESHOLD) {
      objectDetectedTime1 = millis();
      Serial.println("Athlete passed Gate 1 at: ");
      Serial.println("V = " + String(voltage));
      Serial.println(objectDetectedTime1);

      // Update the display
      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(1);
      display.println("GATE 1:");
      display.println("Athlete Detected");
      display.println("----------------");
      String message = "Gate 1: t = " + String(objectDetectedTime1);
      display.println(message);
      display.println("----------------");
      display.println("Waiting for GATE 2.");
      display.display();

      // Send a message via LoRa
      const char *message2gate2 = "Object Detected at Gate 1";
      rf95.send((uint8_t *)message2gate2, strlen(message2gate2));
      rf95.waitPacketSent();

      Serial.println("Sent a message via LoRa");
      } else {
      display.println("No object detected.");
      }

      athletePassedGate1 = true;  // Set the flag
      Serial.println("Athlete flag Gate 1 set to true");
      delay(5000);
    }
}


void checkLoRaConnectivity() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.println("Checking LoRa...");
  display.display();

  // Ensure the message is exactly as intended
  const char* testMessage = "LoRa Ping";
  // Debugging: print the message being sent to the Serial Monitor
  Serial.print("Sending test message: ");
  Serial.println(testMessage);

  // Send test message
  //rf95.send((uint8_t*)testMessage.c_str(), testMessage.length());
  rf95.send((uint8_t*)testMessage, strlen(testMessage));
  rf95.waitPacketSent();
  Serial.println("Test message sent, waiting for ack...");

  // Wait for a response
  unsigned long startWait = millis();
  bool ackReceived = false;
  
  while (millis() - startWait < 5000) { // Wait for up to 5 seconds for a response
    if (rf95.available()) {
      // Buffer to hold the incoming message
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      
      if (rf95.recv(buf, &len)) {
        // Null-terminate the received data to create a proper string
        buf[len] = '\0';
        String message = String((char*)buf);
        Serial.println("Received message: " + message); // Debugging purpose

        // Check for acknowledgment
        if (message == "Ack from GATE 2") {
          ackReceived = true;
          break; // Ack received, exit the wait loop
        }
      }
    }

  // Update display based on the result
  display.clearDisplay();
  display.setCursor(0, 0);
  if (ackReceived) {
    display.println("LoRa Connectivity: OK");
    Serial.println("Ack received from GATE 2.");
  } else {
    display.println("LoRa Connectivity: FAIL");
    Serial.println("No ack received. Check GATE 2 or LoRa setup.");
  }
  display.display();
}


  // Update display based on the result
  display.clearDisplay();
  display.setCursor(0, 0);
  if (ackReceived) {
    display.println("LoRa Check: OK");
    Serial.println("Ack received");
  } else {
    display.println("LoRa Check: Fail");
    Serial.println("No ack received");
  }
  display.display();
}


void sendSyncBeacon() {
  unsigned long currentTime = millis();
  String syncMessage = "SYNC:" + String(currentTime);
  rf95.send((uint8_t *)syncMessage.c_str(), syncMessage.length());
  rf95.waitPacketSent();
  Serial.println("Sync beacon sent with time: " + String(currentTime));
}


// Global variable to track the last update time
unsigned long lastMeteoDataUpdateTime = 0;
MeteoData currentMeteoData;
static unsigned long lastSyncTime = 0;
// Global variable to track testSequences
int testSequenceCounter = 0; // Counter for ski test sequences
int timoutRestriction = 50000; // ms until GATE1 goes to reset
const long countdownDuration = 50000; // 10 seconds countdown

unsigned long countdownStartTime = 0; // When the countdown starts
bool countdownActive = false; // Is countdown active?

float velocity = 0;

void logDataToSDCard(unsigned long time1, unsigned long time2, float velocity, float temperature, float humidity) {
  if (!logfile) {
    Serial.println("SD Card logging failed: logfile not open");
    return;
  }

  // Format the log entry with temperature and humidity
  logfile.print("Time1: ");
  logfile.print(time1);
  logfile.print(", Time2: ");
  logfile.print(time2);
  logfile.print(", Velocity: ");
  logfile.print(velocity);
  logfile.print(", Temperature: ");
  logfile.print(temperature, 1); // One decimal place for temperature
  logfile.print(" C, Humidity: ");
  logfile.print(humidity, 1); // One decimal place for humidity
  logfile.println(" %");
  logfile.flush(); // Ensure the data is written to the SD card

  Serial.println("Logged detection times to SD card in one line.");
  // Print the same information to the Serial monitor
  Serial.print("Time1: ");
  Serial.print(time1);
  Serial.print(", Time2: ");
  Serial.print(time2);
  Serial.print(", Velocity: ");
  Serial.print(velocity);
  Serial.print(", Temperature: ");
  Serial.print(temperature, 1); // One decimal place for temperature
  Serial.print(" C, Humidity: ");
  Serial.print(humidity, 1); // One decimal place for humidity
  Serial.println(" %");

  display.println("----------------");
  String logmessage = "Logged to file!";
  display.println(logmessage);
  display.println("----------------");
  display.println("Going to Reset.");
  display.display();

}

void loop() {
  static unsigned long detectionTimestamp = 0; // Timestamp of the detection by Gate 1
  static unsigned long gate2DetectionTimestamp = 0; // Timestamp received from Gate 2
  bool receivedGate2Ack = false;

  bool newButtonCState = digitalRead(buttonCPin); // Read the current state of button C
  if (lastButtonCState == HIGH && newButtonCState == LOW) {
    delay(50); // Simple debounce delay

    // Increment the test sequence counter
    testSequenceCounter++;

    // Log a unique sequence to the SD card to distinguish between ski tests
    if (logfile) {
      String separator = "---------- Test Sequence ";
      separator += String(testSequenceCounter);
      separator += " ----------\n";
      logfile.print(separator);
      logfile.flush(); // Ensure data is written to the file
      Serial.print("Logged separator for Test Sequence ");
      Serial.println(testSequenceCounter);

      // Display the same separator on the OLED
      display.clearDisplay(); // Clear previous content
      display.setTextSize(1); // Set text size
      display.setTextColor(SH110X_WHITE); // Set text color
      display.setCursor(0, 0); // Set start position to top-left corner
    
      // Print the sequence info on the OLED
      display.print("Test Sequence: ");
      display.println(testSequenceCounter);
      display.println("---------------");
    
      // Add any additional info you want to display here
      display.display(); // Actually display all the printed text
      delay(1000);

    }
  }

if (countdownActive) {
    // Serial.println("Countdown loop");
    unsigned long currentTime = millis();
    long timeElapsed = currentTime - countdownStartTime;
    long timeLeft = countdownDuration - timeElapsed;

    if (timeLeft <= 0) {
        // Countdown has finished
        countdownActive = false; // Stop the countdown
        // Optional: Handle timeout, such as displaying a message or taking other actions
    } else {
        // Update the display every second with the remaining time
        if (timeElapsed % 1000 < 50) { // Update roughly every second
            display.clearDisplay();
            display.setCursor(0, 0);
            display.setTextSize(1);
            display.println("GATE 1:");
            display.println("Athlete Detected");
            display.println("----------------");
            display.println("Gate 1: t = " + String(detectionTimestamp));
            display.println("----------------");
            display.println("Waiting for GATE 2.");
            // Calculate remaining seconds and display
            int secondsLeft = timeLeft / 1000;
            display.print("Timeout in: ");
            display.print(secondsLeft);
            display.println(" sec");
            display.display();
        }
    }
  }


  // Current time in milliseconds
  unsigned long currentMillis = millis();

  
  if (millis() - lastSyncTime > 10000) { // Send sync beacon every 10 seconds
    sendSyncBeacon();
    lastSyncTime = millis();
  }

  int sensorValue = analogRead(VOLTAGE_PIN);
  float voltage = sensorValue * (3.0 / 1023.0);
  
  // Display voltage on OLED and Serial Monitor
  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("Raw: "); display.print(sensorValue);
  display.print(", V: "); display.println(voltage);
  Serial.print("Gate 1 Raw: "); Serial.println(voltage);


  if (!athletePassedGate1) {
    

    if (voltage < VOLTAGE_THRESHOLD ) {
      detectionTimestamp = millis(); // Record the detection time
      Serial.println("Athlete passed Gate 1 at: ");
      Serial.print("V = ");
      Serial.println(voltage);
      Serial.println(detectionTimestamp);

      // Update the display with the latest MeteoData and voltage
      updateDisplayWithVoltage(currentMeteoData);

      // Update the display
      display.clearDisplay();
      display.setCursor(0, 0);
      display.setTextSize(1);
      display.println("GATE 1:");
      display.println("Athlete Detected");
      display.println("----------------");
      String message = "Gate 1: t = " + String(detectionTimestamp);
      display.println(message);
      display.println("----------------");
      display.println("Waiting for GATE 2.");
      display.display();

      // Send a message via LoRa
      const char* message2gate2 = "Object Detected at Gate 1";
      rf95.send((uint8_t*)message2gate2, strlen(message2gate2));
      rf95.waitPacketSent();
      Serial.println("Sent a message via LoRa");

      athletePassedGate1 = true;  // Set the flag
      Serial.println("Athlete flag Gate 1 set to true");
      
      // Right after displaying "Waiting for GATE 2." on the OLED
      countdownStartTime = millis(); // Capture the start time of the countdown
      countdownActive = true; // Activate the countdown
      Serial.println("Countdowflag set to True!");

    }
    } else {
    // Check for incoming messages from Gate 2
    if (rf95.available()) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len)) {
        String message = String((char*)buf);
        Serial.print("Received from Gate 2: "); Serial.println(message);
        // Assuming the message format is "Skier passed Gate 2, detection time: TIMESTAMP"
        int index = message.indexOf("detection time: ");
        if (index > 0) {
          String timeStr = message.substring(index + strlen("detection time: "));
          gate2DetectionTimestamp = timeStr.toInt();
          Serial.println(gate2DetectionTimestamp);
          int tdiff = gate2DetectionTimestamp - detectionTimestamp;
          Serial.println(tdiff);

         receivedGate2Ack = true;
        }
      }
    }

    // If acknowledgment received or timeout reached
    if (receivedGate2Ack || millis() - detectionTimestamp > timoutRestriction) {
      if (receivedGate2Ack) {
        // Calculate time difference
        unsigned long timeDifference = gate2DetectionTimestamp - detectionTimestamp;

        // Print out to OLED
        display.clearDisplay(); // Clear previous messages
        display.setCursor(0, 0); // Reset cursor to top of display
        display.setTextSize(1); // Set text size
        display.println("GATE 2 passed.");
    
        // Display the time difference
        display.print("Time diff: ");
        display.print(timeDifference);
        display.println(" ms"); // Assuming milliseconds for time unit
        display.display(); // Make sure to update the display with new information
    

        // Assuming this part is where you handle the logging after GATE 2 detection
        MeteoData meteoData = getMeteoData(); // Get the latest meteorological data

        // Assuming you have time1, time2, and calculated velocity available
        logDataToSDCard(detectionTimestamp, gate2DetectionTimestamp, velocity, meteoData.temperature, meteoData.humidity);
        delay(10000);
        

    //MeteoData meteoData = getMeteoData(); // Get meteorological data

      if (meteoData.isValid){
        updateDisplayWithVoltage(meteoData);
        delay(250);
        } else {
        // Handle timeout condition
        display.clearDisplay();
        display.setCursor(0, 0);
        display.println("Reset. Waiting for Skier.");
        display.display();
        }
      } else {
      // Handle timeout condition
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Timeout waiting for");
      display.println("GATE 2. System Reset.");
      display.println("Ready.");
      display.display();
      //delay(5000);
      } 

      // Reset states after handling
      athletePassedGate1 = false;
      detectionTimestamp = 0;
      gate2DetectionTimestamp = 0;
      receivedGate2Ack = false;
      countdownActive = false; // Ensure the countdown is deactivated
      }
    }

  // Improved Button A Press Detection
  bool newButtonState = digitalRead(buttonAPin); // Read the current state of button A
  // Assuming this is part of the loop() function or another function
  if (lastButtonState == HIGH && newButtonState == LOW) {
    delay(50); // Simple debounce delay
    checkLoRaConnectivity(); // Perform LoRa connectivity check
    }
  lastButtonState = newButtonState; // Update lastButtonState for the next loop iteration

  // Your loop logic continues here...
  delay(1); // Refresh rate for the loop

}