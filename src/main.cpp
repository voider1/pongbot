#include <DRV8825.h>
#include <BasicStepperDriver.h>
#include <MultiDriver.h>
#include <SyncDriver.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoOTA.h>

const unsigned int SERIAL_BAUDRATE = 115200;
const char* HOSTNAME = "PongBot";
const String GET_COMMAND_URL = "";
const char* SSID = "";
const char* PASSWORD = "";

const uint8_t SOLENOID_PIN = D7;
const uint8_t HOMING_PIN   = D8;

/// Constants for Directions
enum class Direction {
    UP,
    DOWN,
    LEFT,
    RIGHT,
    SHOOT,
    UNDEFINED,
};

/// Possible axis orientations
enum Orientation {
    VERTICAL = 0,
    HORIZONTAL = 1,
};

/// This is a physical property of the stepper motors used.
/// It specifies how many steps have to be executed for a full rotation.
const unsigned int STEPS_PER_ROT = 200;

/// Struct with data containing settings for one stepper motor.
struct motor {
    short dir, step, microsteps, rpm, accel;
};

const motor motor1 { D5, D4, 1, 60, 1000 };
const motor motor2 { D2, D3, 1, 30, 500 };

/// Stepper motor 1 (horizontal)
DRV8825 stepper1(STEPS_PER_ROT, motor1.dir, motor1.step);
/// Stepper motor 2 (vertical)
DRV8825 stepper2(STEPS_PER_ROT, motor2.dir, motor2.step);

/// Sync driver is used to synchronize motion across the two axes
SyncDriver controller(stepper1, stepper2);

/// Function that checks a string argument and returns an Enum variant for it in the place
Direction convertDirection(const String &dir) {
    if (dir == "up") {
        return Direction::UP;
    } else if (dir == "down") {
        return Direction::DOWN;
    } else if (dir == "left") {
        return Direction::LEFT;
    } else if (dir == "right") {
        return Direction::RIGHT;
    } else if (dir == "shoot") {
        return Direction::SHOOT;
    } else {
        return Direction::UNDEFINED;
    }
}

/// Tiny calculation to correct for the pulley ratio
inline float calculateAngle(int degree) {
    return degree * 3.75f;
}

/// Function that extracts the Orientation and amount of degrees
/// from the string that the web server returned
bool processCommand(const String &payload, Direction *dir, int *deg) {
    int commaIndex = payload.indexOf(',');
    int endString = payload.length();
    String dir_string = payload.substring(0, commaIndex);
    String deg_string = payload.substring(commaIndex + 1, endString);
    *dir = convertDirection(dir_string);
    *deg = static_cast<int>(deg_string.toInt());

    Serial.printf("New command: %d deg in dir %s\n", *deg, dir);
}

/// Connects to the WiFi network to be able to access the internet
void wifiSetup() {
    // Set WIFI module to STA mode
    WiFi.mode(WIFI_STA);

    // Connect
    Serial.printf("[WIFI] Connecting to %s ", SSID);
    WiFi.begin(SSID, PASSWORD);

    // Wait
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(100);
    }
    Serial.println();

    // Connected!
    byte mac[6];
    WiFi.macAddress(mac);
    Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s, MAC adress: %02hX:%02hX:%02hX:%02hX:%02hX:%02hX\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str(), mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
}

/**
 * @brief This function sets up OTA firmware update.
 *
 * It includes a few options for hostname etc that are hardcoded inside the function
 */
void OTASetup() {
    // Hostname defaults to esp8266-[ChipID]
    ArduinoOTA.setHostname(HOSTNAME);

    // No authentication by default
    ArduinoOTA.setPassword("P4wnB0t");

    ArduinoOTA.onStart([]() {
        // TODO: disable motors;
        Serial.println("[ArduinoOTA] Start updating ");
    });

    ArduinoOTA.onEnd([]() { Serial.println("\nEnd"); });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("[ArduinoOTA] Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");

        ESP.restart();
    });

    Serial.printf("[ArduinoOTA] Starting OTA server with name %s\n", HOSTNAME);
    /* setup the OTA server */
    ArduinoOTA.begin();
    Serial.println("[ArduinoOTA] Server started");
}

/// This function activates the motors of pongbot to move a relative amount over a specified axis
bool moveHead(int rotation, float angle) {
    // If angle is 0, no movement is needed
    if (abs(angle) >= 0.01) {
        if (rotation == Orientation::VERTICAL) {
            Serial.printf("Rotating vertically for %f degrees\n", angle);
            controller.rotate(0.0f, angle);
        } else if (rotation == Orientation::HORIZONTAL) {
            Serial.printf("Rotating horizontally for %f degrees\n", angle);
            controller.rotate(angle, 0.0f);
        }
        return true;
    }
    return false;
}

/// interrupts that fires when the motors have to be stopped
void interruptHandler() {
    // Stop vertical stepper immediately to prevent damage
    stepper2.stop();
    Serial.println("Interrupt fired, vertical stepper motor stopped");
}

/// @brief Sets up hardware interrupts that fire when the homing switches are pressed.
void setupHomingInterrupt() {
    pinMode(HOMING_PIN, INPUT_PULLUP);
    Serial.println("[Interrupt]");
    attachInterrupt(digitalPinToInterrupt(HOMING_PIN), interruptHandler, FALLING);
}

/// Hardware setup like usual
void setup() {
    Serial.begin(SERIAL_BAUDRATE);

    stepper1.begin(motor1.rpm, motor1.microsteps);
    stepper2.begin(motor2.rpm, motor2.microsteps);
    // Set speed profiles to enable acceleration and deceleration
    stepper1.setSpeedProfile(stepper1.LINEAR_SPEED, motor1.accel, motor1.accel);
    stepper2.setSpeedProfile(stepper2.LINEAR_SPEED, motor2.accel, motor2.accel);

    // Setup WiFi and Over The Air firmware update
    wifiSetup();
    OTASetup();

    pinMode(SOLENOID_PIN, OUTPUT);
    digitalWrite(SOLENOID_PIN, LOW);
}

/// Main program loop
void loop() {
    if (WiFi.status() == WL_CONNECTED) {
        HTTPClient http;

        http.begin(GET_COMMAND_URL);
        int httpCode = http.GET();

        // If the page load is successful, parse and execute data
        if (httpCode == 200) {
            /// The data that is returned by the request to the server
            String payload = http.getString();
            /// Holds the direction specified by the request to the server
            Direction dir;
            /// Holds the amount of degrees to move, if appropriate
            int deg;

            // Serial.printf("New command: %d deg in dir %d\n", degs, dir);

            processCommand(payload, &dir, &deg);

            if (dir == Direction::SHOOT) {
                digitalWrite(SOLENOID_PIN, HIGH);
                delay(500);
                digitalWrite(SOLENOID_PIN, LOW);
                return;
            }

            int rotation = -1;
            float calculatedAngle = calculateAngle(deg);

            switch (dir) {
                case Direction::UP:
                    rotation = Orientation::VERTICAL;
                    break;
                case Direction::DOWN:
                    rotation = Orientation::VERTICAL;
                    calculatedAngle = -calculatedAngle;
                    break;
                case Direction::LEFT:
                    rotation = Orientation::HORIZONTAL;
                    calculatedAngle = -calculatedAngle;
                    break;
                case Direction::RIGHT:
                    rotation = Orientation::HORIZONTAL;
                    break;
                default:
                    calculatedAngle = 0;
                    break;
            }

            Serial.printf("Calculated angle is %f deg\n", calculatedAngle);

            moveHead(rotation, calculatedAngle);
        }

        http.end();
    }

    delay(1000);
}
