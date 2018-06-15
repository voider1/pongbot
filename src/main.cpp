#include <DRV8825.h>
#include <BasicStepperDriver.h>
#include <MultiDriver.h>
#include <SyncDriver.h>
#include <ArduinoJson.h>

#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ArduinoOTA.h>
#define WIFI_HTM_PROGMEM
#include <PersWiFiManager.h>
#include <FS.h>

const unsigned int SERIAL_BAUDRATE = 115200;
const char HOSTNAME[] = "PongBot";
const char AP_NAME[] = "PongBot WiFi Setup";

const String GET_COMMAND_URL = "";

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
    /// The buffer stores the parsed JSON object (not the string itself).
    // The JSON_OBJECT_SIZE calculates the size needed to store the amount of items of its argument
    StaticJsonBuffer<JSON_OBJECT_SIZE(4)> jsonBuffer;
    /// Reference to a location inside the buffer where the object is located with the extracted properties.
    JsonObject& root = jsonBuffer.parseObject(payload);

    if (root.success()) {
        *dir = convertDirection(root["dir"]);
        *deg = root["deg"]; // The library handles conversion automagically
    } else { return false; }

    Serial.printf("New command: %d deg in dir %d (%s)\n", *deg, *dir, root["dir"].as<char *>());
    return true;
}

ESP8266WebServer server(80);
DNSServer dnsServer;
PersWiFiManager WiFiManager(server, dnsServer);
// This shouldn't be necessary with #define WIFI_HTM_PROGMEM, but that does not seem to be working.
const char wifi_htm[] PROGMEM = R"=====(<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1, user-scalable=no"/><title>ESP WiFi</title><script>function g(i){return document.getElementById(i);};function p(t,l){if(confirm(t)) window.location=l;};function E(s){return document.createElement(s)};var S="setAttribute",A="appendChild",H="innerHTML",X,wl;function scan(){if(X) return;X=new XMLHttpRequest(),wl=document.getElementById('wl');wl[H]="Scanning...";X.onreadystatechange=function(){if (this.readyState==4&&this.status==200){X=0;wl[H]="";this.responseText.split("\n").forEach(function (e){let t=e.split(","), s=t.slice(2).join(',');var d=E('div'),i=E('a'),c=E('a');i[S]('class','s'); c[S]('class','q');i.onclick=function(){g('s').value=s;g('p').focus();};i[A](document.createTextNode(s));c[H]=t[0]+"%"+(parseInt(t[1])?"\uD83D\uDD12":"\u26A0");wl[A](i); wl[A](c);wl[A](document.createElement('br'));});}};X.open("GET","wifi/list",true);X.send();};</script><style>input{padding:5px;font-size:1em;width:95%;}body{text-align:center;font-family:verdana;background-color:black;color:white;}a{color:#1fa3ec;}button{border:0;border-radius:0.3em;background-color:#1fa3ec;color:#fff;line-height:2.4em;font-size:1.2em;width:100%;display:block;}.q{float:right;}.s{display:inline-block;width:14em;overflow:hidden;text-overflow:ellipsis;white-space:nowrap;}#wl{line-height:1.5em;}</style></head><body><div style='text-align:left;display:inline-block;width:320px;padding:5px'><button onclick="scan()">&#x21bb; Scan</button><p id='wl'></p><form method='post' action='/wifi/connect'><input id='s' name='n' length=32 placeholder='SSID'><br><input id='p' name='p' length=64 type='password' placeholder='password'><br><br><button type='submit'>Connect</button></form><br><br><button onclick="p('Start WPS?','/wifi/wps')">WPS Setup</button><br><button onclick="p('Reboot device?','/wifi/rst')">Reboot</button><br><a href="javascript:history.back()">Back</a> |<a href="/">Home</a></div></body></html>)=====";

/// Connects to the WiFi network to be able to access the internet
void wifiSetup() {

    Serial.println("[WIFI] Starting WiFiManager");

    SPIFFS.begin();

    WiFiManager.onAp([](){
        Serial.print("[WIFI] AP mode with SSID: ");
        Serial.println(WiFiManager.getApSsid());
    });
    WiFiManager.setApCredentials(AP_NAME);

    // Run the WiFi manager and check if it could connect
    // if that is not the case, it will setup an AP to allow selection of another network
    if (!WiFiManager.begin()) {
        Serial.println("[WIFI] Could not connect to previous network, setting up AP");

        // Again, this should be redundant, but alas.
        server.on("/wifi.htm", [&]() {
            server.send(200, "text/html", wifi_htm);
        });
        server.begin();
        uint32_t looper = millis();
        for (uint8_t i = 0; i < 120; ++i) {
            while (millis() - looper < 1000) {
                dnsServer.processNextRequest();
                server.handleClient();
                delay(1);
            } looper = millis();
            Serial.write('.');
        }
        Serial.println("[WIFI] Timeout for setting up WiFi, resetting");
        ESP.restart();
    }
    Serial.println();

    // Set WIFI module to STA mode
    WiFi.mode(WIFI_STA);

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
    ArduinoOTA.setPort(8266);

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
    Serial.println();

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

    Serial.println("Setup complete");
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
    ArduinoOTA.handle();
}
