#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
//#include <LSIR_Camera.h>
#include <JPEGCamera.h>
#include <ESP32_Servo.h>
#include <Preferences.h>
#include "private.h"


#define ENABLE_SERIAL 1
#define DEBUG 1
#ifndef ENABLE_SECURITY
#define ENABLE_SECURITY 0
#endif

//WIFI Settings
//const char* SSID = "ssid";
//const char* PASSWORD = "password";
//const char* SERVER = "ipaddress";


#if ENABLE_SECURITY
WiFiClientSecure client;
#else
WiFiClient client;
#endif








bool led_value;
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details.

PubSubClient mqtt(client);

#define PRE "esp32/"

const char * ESP32_STATUS = PRE "status";
const char * ESP32_LED = PRE "led";
const char * ESP32_CAPTURE_REQUEST = PRE "camera/request";
const char * ESP32_CAPTURE_SNAP = PRE "camera/snap";
const char * ESP32_CAMERA_SEND_DATA = PRE "camera/send";
const char * ESP32_IMAGE_SIZE = PRE "camera/size";
const char * ESP32_IMAGE_DATA = PRE "camera/data";

const char * ESP32_POWER_MANUAL = PRE "power/manual";
const char * ESP32_POWER_STATE = PRE "power/state";
const char * ESP32_POWER_ENABLE_SENSORS = PRE "power/enable_sensors";
const char * ESP32_BATTERY_VOLTAGE = PRE "battery/voltage";

const char * ESP32_SERVO_MANUAL = PRE "servo/manual";
const char * ESP32_SERVO_STATE = PRE "servo/state";
const char * ESP32_SERVO_ROLL = PRE "servo/roll";
const char * ESP32_SERVO_ROLL_POS = PRE "servo/roll_pos";

const char * ESP32_SERVO_PITCH = PRE "servo/pitch";
const char * ESP32_SERVO_PITCH_POS = PRE "servo/pitch_pos";

const char * ESP32_LIGHT_TOP_LEFT = PRE "light/top_left";
const char * ESP32_LIGHT_TOP_RIGHT = PRE "light/top_right";
const char * ESP32_LIGHT_BOTTOM_LEFT = PRE "light/bottom_left";
const char * ESP32_LIGHT_BOTTOM_RIGHT = PRE "light/bottom_right";

const char * ESP32_LIGHT = PRE "light";

const char * ESP32_SOLAR_TRACKED = PRE "solar/tracked";


const uint16_t CAPTURE_PORT = 2048;


//LinkSprite Camera
#define CAM_CHUNK_SIZE 64
HardwareSerial CameraSerial(2);
JPEGCamera IRCamera(CameraSerial);
byte cam_data[CAM_CHUNK_SIZE];
int cam_file_size = 0;


//Power Interface
#define BOOST_ENABLE 15
#define ROLL_SERVO 27
#define PITCH_SERVO 12
#define LDR_TR A2
#define LDR_BR A3
#define LDR_TL A7
#define LDR_BL A4

#define BAT_VOLTAGE A13


int solar_tracked = 0;
bool servo_manual_enable;
bool enable_sensor_output;

int roll_servo_pos = 90;
int pitch_servo_pos = 90;

Servo RollServo;
Servo PitchServo;

const int THRESHOLD = 120;

const int ROLL_MIN = 800;
const int ROLL_MAX = 2400;

const int PITCH_MIN = 1000;
const int PITCH_MAX = 2400;

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to minutes */


//Deep Sleep Timer
#if DEBUG

#define TIME_TO_SLEEP  30                               /* Time ESP32 will go to sleep (in seconds) */
#define TIME_AWAKE 3 * 60

#else

#define TIME_AWAKE 3 * 60
#define TIME_TO_SLEEP  (15 * 60) - (TIME_AWAKE)         /* Time ESP32 will go to sleep (in seconds) */

#endif

#define BUFFER_SIZE 100
char main_buffer[BUFFER_SIZE];


//Awake Timer
RTC_DATA_ATTR int boot_count = 0;
hw_timer_t * timer = NULL;
volatile uint32_t wake_count = 0;
volatile uint32_t time_awake = 0;
volatile boolean time_to_sleep = false;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

Preferences preferences;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  time_awake = millis() / 1000;
  time_to_sleep = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {

  /***************************************************
     Setup Board Level Boilerplate stuff
   ***************************************************/
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(BOOST_ENABLE, OUTPUT);
  digitalWrite(BOOST_ENABLE, LOW);
  RollServo.attach(ROLL_SERVO, ROLL_MIN, ROLL_MAX);
  PitchServo.attach(PITCH_SERVO, PITCH_MIN, PITCH_MAX);

  servo_manual_enable = false;
  enable_sensor_output = false;
  solar_tracked = 0;
  time_to_sleep = false;


  /***************************************************
     Get persistant data
   ***************************************************/
  preferences.begin("overlook", false);
  wake_count = preferences.getUInt("wake_count", 0);
  roll_servo_pos = preferences.getUInt("roll_pos", 90);
  pitch_servo_pos = preferences.getUInt("pitch_pos", 90);
  wake_count++;

  preferences.putUInt("wake_count", wake_count);
  // Close the Preferences
  preferences.end();




  /***************************************************
     Awake Timer
   ***************************************************/
  timer = timerBegin(0, 80, true);
  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIME_AWAKE * uS_TO_S_FACTOR, true);
  // Start an alarm
  timerAlarmEnable(timer);

  /***************************************************
     Setup Host Serial Port
   ***************************************************/
#if ENABLE_SERIAL
  Serial.begin(115200);
  Serial.println("Booting");
#endif

  /***************************************************
     Setup WIFI
   ***************************************************/

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASSWORD);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
#if ENABLE_SERIAL
    Serial.println("Connection Failed! Rebooting...");
#endif
    delay(5000);
    ESP.restart();
  }
  digitalWrite(LED_BUILTIN, HIGH);

#ifdef ENABLE_SERIAL
    Serial.print("Connected to: ");
    Serial.print(SSID);
#endif

  /***************************************************
     Setup OTA updates
   ***************************************************/
  // Port defaults to 3232
  ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("overlook-esp32");

  // No authentication by default
  //ArduinoOTA.setPassword("admin");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH){
      mqtt.publish(ESP32_STATUS, "Start Sketch Update");
    }
    else {// U_SPIFFS
      mqtt.publish(ESP32_STATUS, "Start Filesystem Update");
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
  });
  ArduinoOTA.onEnd([]() {
    mqtt.publish(ESP32_STATUS, "OTA Update Finished");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    snprintf(main_buffer, BUFFER_SIZE, "Progress: %u%%\0", (progress / total / 100));
    mqtt.publish(ESP32_STATUS, main_buffer);
  });
  ArduinoOTA.onError([](ota_error_t error) {
    if (error == OTA_AUTH_ERROR) mqtt.publish(ESP32_STATUS, "OTA Error: Auth Failed");
    else if (error == OTA_BEGIN_ERROR) mqtt.publish(ESP32_STATUS, "OTA Error: Begin Failed");
    else if (error == OTA_CONNECT_ERROR) mqtt.publish(ESP32_STATUS, "OTA Error: Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) mqtt.publish(ESP32_STATUS, "OTA Error: Receive Failed");
    else if (error == OTA_END_ERROR) mqtt.publish(ESP32_STATUS, "OTA Error: End Failed");
  });
  ArduinoOTA.begin();
#if ENABLE_SERIAL
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  Serial.print("Attempt to connect to: ");
  Serial.print(SERVER);
  Serial.print(":");
  Serial.println(MQTT_PORT);

#endif

#if ENABLE_SECURITY
  Serial.println("Configuring Secure Client for connection...");
  //client.setTrustAnchros(&ca_cert_X509);
  client.setCACert(ca_cert);
  //client.allowSelfSignedCerts();
  //client.setFingerprint(clientCertFingerprint);
#endif


  /***************************************************
     Setup MQTT
   ***************************************************/
  //client.setServer(SERVER, MQTT_PORT);
  while (!client.connect(SERVER, MQTT_PORT)){
  //while(mqtt.setServer(SERVER, MQTT_PORT));
      String line = client.readStringUntil('\n');
      if (line == "\r") {
        Serial.println("headers received");
        break;
      }
    Serial.print("Error: ");
    //Serial.println(buf);
    Serial.println("Connection Failed! Wait 2 Seconds...");
    delay(2000);
  }
  mqtt.setServer(SERVER, MQTT_PORT);
  Serial.println("Connection Successful!");
  mqtt.setCallback(callback);

  /***************************************************
     Camera Serial Port
   ***************************************************/
  CameraSerial.begin(38400);
  IRCamera.powerSaving(true);

  /***************************************************
     Give the system Power to do all the WIFI Tracking
   ***************************************************/
  enable_boost(1);

}

void loop() {
  //Handle MQTT
  if (!mqtt.connected()) {
    reconnect();
    print_wakeup_reason();
    mqtt.publish(ESP32_SERVO_STATE, "0");
    mqtt.publish(ESP32_POWER_STATE, "0");
    mqtt.publish(ESP32_CAPTURE_REQUEST, "1");
  }
  mqtt.loop();
  ArduinoOTA.handle();
  app_loop();

}

void print_wakeup_reason() {
  esp_deep_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_deep_sleep_get_wakeup_cause();
  snprintf(main_buffer, BUFFER_SIZE, "Awake: (Boot Count: %d) Caused by: %s\0", boot_count, "Wakeup caused by external signal using RTC_IO");

  switch (wakeup_reason)
  {
    case 1  : snprintf(main_buffer, BUFFER_SIZE, "Awake: (Boot Count: %d) Caused by: %s\0", boot_count, "external signal using RTC_IO"); break;
    case 2  : snprintf(main_buffer, BUFFER_SIZE, "Awake: (Boot Count: %d) Caused by: %s\0", boot_count, "external signal using RTC_CNTL"); break;
    case 3  : snprintf(main_buffer, BUFFER_SIZE, "Awake: (Boot Count: %d) Caused by: %s\0", boot_count, "timer"); break;
    case 4  : snprintf(main_buffer, BUFFER_SIZE, "Awake: (Boot Count: %d) Caused by: %s\0", boot_count, "touchpad"); break;
    case 5  : snprintf(main_buffer, BUFFER_SIZE, "Awake: (Boot Count: %d) Caused by: %s\0", boot_count, "ULP program"); break;
    default : snprintf(main_buffer, BUFFER_SIZE, "Awake: (Boot Count: %d) %s\0", boot_count, "Wakeup was not caused by deep sleep"); break;
  }
  mqtt.publish(ESP32_STATUS, main_buffer);
  boot_count++;
}

/***************************************************
   MQTT Functions
 ***************************************************/
//MQTT Connect
void reconnect() {
  while (!mqtt.connected()) {
#if ENABLE_SERIAL
    Serial.print("Attempting MQTT Connection...");
#endif
    //Attempt to connect
    if (mqtt.connect("ESP8266 Client")) {
#if ENABLE_SERIAL
      Serial.println("Connected");
#endif
      mqtt.subscribe(ESP32_LED);
      mqtt.subscribe(ESP32_CAPTURE_SNAP);
      mqtt.subscribe(ESP32_CAMERA_SEND_DATA);
      mqtt.subscribe(ESP32_POWER_MANUAL);
      mqtt.subscribe(ESP32_SERVO_MANUAL);
      mqtt.subscribe(ESP32_SERVO_ROLL);
      mqtt.subscribe(ESP32_SERVO_PITCH);
      mqtt.subscribe(ESP32_POWER_ENABLE_SENSORS);
    }
    else {
#if ENABLE_SERIAL
      Serial.print("Failed, RetVal: ");
      Serial.print(mqtt.state());
      Serial.println(" Try again in 5 seconds");
#endif
      delay(5000);
    }
  }
}


//MQTT Subscription Callback
void callback(char * topic, byte* payload, unsigned int length) {
  int value = 0;
  if (strcmp(topic, ESP32_LED) == 0) {
//    Serial.println("LED Control");
    led_value = atoi((char *) payload);
  }
  if (strcmp(topic, ESP32_CAPTURE_SNAP) == 0) {
//    Serial.println("Capture Image");
    capture_image();
  }
  if (strcmp(topic, ESP32_CAMERA_SEND_DATA) == 0) {
//    Serial.println("Send Data");
    send_image_data();
  }
  if (strcmp(topic, ESP32_POWER_MANUAL) == 0) {
//    Serial.println("Manual Power Control");
    enable_boost(atoi((char *) payload));
  }
  if (strcmp(topic, ESP32_SERVO_MANUAL) == 0) {
//    Serial.println("Manual Power Control");
    //enable_boost(atoi((char *) payload));
    if (atoi((char *) payload) > 0) {
//      Serial.println("Servo Manual Control Enabled");
      mqtt.publish(ESP32_SERVO_STATE, "1");
      servo_manual_enable = true;
    }
    else {
//      Serial.println("Servo Manual Control Disabled");
      mqtt.publish(ESP32_SERVO_STATE, "0");
      servo_manual_enable = false;
    }
  }
  if (strcmp(topic, ESP32_SERVO_ROLL) == 0) {
    if (servo_manual_enable) {
//      Serial.println("Servo Roll Control");
      set_roll(atoi((char *) payload));
    }
    else {
//      Serial.println("Error: Attempting to move the servo when not enabled");
    }
  }
  if (strcmp(topic, ESP32_SERVO_PITCH) == 0) {
    if (servo_manual_enable) {
//      Serial.println("Servo Pitch Control");
      set_pitch(atoi((char *) payload));
    }
    else {
//      Serial.println("Error: Attempting to move the servo when not enabled");
    }
  }

  if (strcmp(topic, ESP32_POWER_ENABLE_SENSORS) == 0) {
    value = atoi((char* ) payload);
//    Serial.print ("Value: ");
//    Serial.println(value);
    if (value > 0) {
//      Serial.println("Start Sensor Dump");
      enable_sensor_output = true;
    }
    else {
//      Serial.println("Stop Sensors Dump");
      enable_sensor_output = false;
    }
  }
}


void app_loop() {
  char buf[10];
  int retval = 0;
  digitalWrite(LED_BUILTIN, led_value);   // turn the LED on (HIGH is the voltage level)
#if ENABLE_SERIAL
  Serial.print("TR: ");
  Serial.print(analogRead(LDR_TR));
  Serial.print(" ");
  Serial.print("BR: ");
  Serial.print(analogRead(LDR_BR));
  Serial.print(" ");
  Serial.print("TL: ");
  Serial.print(analogRead(LDR_TL));
  Serial.print(" ");
  Serial.print("BL: ");
  Serial.print(analogRead(LDR_BL));
  Serial.print(" ");
  Serial.print("BAT: ");
  Serial.println(analogRead(BAT_VOLTAGE) * 2);
#endif
  if (enable_sensor_output) {
    snprintf(buf, 10, "%d\0", analogRead(LDR_TL));
    delay(100);
    mqtt.publish(ESP32_LIGHT_TOP_LEFT, buf);
    snprintf(buf, 10, "%d\0", analogRead(LDR_TR));
    mqtt.publish(ESP32_LIGHT_TOP_RIGHT, buf);
    delay(100);
    snprintf(buf, 10, "%d\0", analogRead(LDR_BL));
    mqtt.publish(ESP32_LIGHT_BOTTOM_LEFT, buf);
    delay(100);
    snprintf(buf, 10, "%d\0", analogRead(LDR_BR));
    mqtt.publish(ESP32_LIGHT_BOTTOM_RIGHT, buf);
    delay(100);
    snprintf(buf, 10, "%d\0", analogRead(BAT_VOLTAGE));
    mqtt.publish(ESP32_BATTERY_VOLTAGE, buf);
    delay(900);
  }

  if (is_boost_enabled()) {
    retval = track_sun();

    if ((solar_tracked == 0) && (retval == 0)) {
      mqtt.publish(ESP32_SOLAR_TRACKED, "1");
      solar_tracked = 1;
      enable_boost(false);
    }
    else if ((solar_tracked == 1) && (retval != 0)) {
      mqtt.publish(ESP32_SOLAR_TRACKED, "0");
      solar_tracked = 0;
    }
  }



  delay(100);
  if (time_to_sleep) {
    //Create a string to tell the mqtt we are going to sleep
    send_sensor_data();
    snprintf(main_buffer, BUFFER_SIZE, "Time Awake (seconds): %d, going to sleep\0", time_awake);
    mqtt.publish(ESP32_STATUS, main_buffer);
    go_to_sleep();
  }
}

void capture_image() {
  IRCamera.powerSaving(false);
  delay(100);
  char size_buffer[10];
  IRCamera.setImageSize(IRCamera.ImageSize320x280);
  //IRCamera.setImageSize(IRCamera.ImageSize160x120);
  //Serial.println("Set Image Size");
  IRCamera.reset();
  IRCamera.takePicture();

  IRCamera.readJpegFileSize(&cam_file_size);
  //Serial.print("JPEG File Size: ");
  //Serial.println(cam_file_size);

  snprintf(&size_buffer[0], 10, "%d", cam_file_size);
  mqtt.publish(ESP32_IMAGE_SIZE, size_buffer, false);
}

void send_image_data() {
  boolean is_end = false;
  int address = 0x00;
  int count = CAM_CHUNK_SIZE;

  while (is_end == false) {
    if ((cam_file_size - address) > CAM_CHUNK_SIZE) {
      count = CAM_CHUNK_SIZE;
    }
    else {
      count = cam_file_size - address;
    }
    IRCamera.readJpegFileContent(address, CAM_CHUNK_SIZE, cam_data, &is_end);
    address += count;
    mqtt.publish((const char *) ESP32_IMAGE_DATA, (const uint8_t *) cam_data, (unsigned int) count, false);
    delay(100);
  }
  //Serial.print("Data Count: ");
  //Serial.println(address);
  IRCamera.stopTakingPictures();
  IRCamera.powerSaving(true);
}

void enable_boost(int enable) {
  if (enable) {
    //Serial.println("Enable Power");
    digitalWrite(BOOST_ENABLE, HIGH);
    mqtt.publish(ESP32_POWER_STATE, "1");
  }
  else {
    //Serial.println("Disable Power");
    digitalWrite(BOOST_ENABLE, LOW);
    mqtt.publish(ESP32_POWER_STATE, "0");
  }
}

boolean is_boost_enabled() {
  return digitalRead(BOOST_ENABLE);
}

void set_roll(int pos) {
  char buf[10];
  snprintf(buf, 10, "%d\0", pos);
  mqtt.publish(ESP32_SERVO_ROLL_POS, buf);
  RollServo.write(180 - pos);
}

void set_pitch(int pos) {
  char buf[10];
  snprintf(buf, 10, "%d\0", pos);
  mqtt.publish(ESP32_SERVO_PITCH_POS, buf);
  PitchServo.write(180 - pos);
}

void send_sensor_data(){
    uint32_t data = 0;
    float bat_voltage;
    data += analogRead(LDR_TL);
    data += analogRead(LDR_TR);
    data += analogRead(LDR_BL);
    data += analogRead(LDR_BR);
    data /= 4;
    snprintf(main_buffer, BUFFER_SIZE, "%d\0", data);
    mqtt.publish(ESP32_LIGHT, main_buffer);


    bat_voltage = analogRead(BAT_VOLTAGE) * 2;
    bat_voltage /= 1000;

    snprintf(main_buffer, BUFFER_SIZE, "%f\0", bat_voltage);
    mqtt.publish(ESP32_BATTERY_VOLTAGE, main_buffer);
    delay(900);
}

int track_sun() {
  int busy = 0;
  //returns 0 when sun has been tracked, the user should keep calling this function over and over until the sun has been tracked
  int ph_left = 0;
  int ph_right = 0;
  int diffx = 0;

  int ph_top = 0;
  int ph_bot = 0;
  int diffy = 0;

  ph_left = (analogRead(LDR_TL) + analogRead(LDR_BL)) / 2;
  ph_right = (analogRead(LDR_TR) + analogRead(LDR_BR)) / 2;
  ph_top = (analogRead(LDR_TR) + analogRead(LDR_TL)) / 2;
  ph_bot = (analogRead(LDR_BL) + analogRead(LDR_BR)) / 2;

  diffx = ph_left - ph_right;
  diffy = ph_top - ph_bot;

  if ((diffx > THRESHOLD) && (roll_servo_pos < 180)) {
    //Serial.println("Roll Left");
    roll_servo_pos += 1;
    busy += 1;
  }
  else if ((diffx < -THRESHOLD) && (roll_servo_pos > 0)) {
    //Serial.println("Roll Right");
    roll_servo_pos -= 1;
    busy += 1;
  }

  if ((diffy > THRESHOLD) && (pitch_servo_pos < 180)) {
    //Serial.println("Pitch Up");
    pitch_servo_pos += 1;
    busy += 1;
  }
  else if ((diffy < -THRESHOLD) && (pitch_servo_pos > 0)) {
    //Serial.println("Pitch Down");
    pitch_servo_pos -= 1;
    busy += 1;
  }
  if (!servo_manual_enable) {
    set_roll(roll_servo_pos);
    set_pitch(pitch_servo_pos);
  }

  return busy;
}

void go_to_sleep() {
  enable_boost(false);

  preferences.begin("overlook", false);
  roll_servo_pos = preferences.getUInt("roll_pos", 90);
  pitch_servo_pos = preferences.getUInt("pitch_pos", 90);

  preferences.putUInt("roll_pos", roll_servo_pos);
  preferences.putUInt("pitch_pos", pitch_servo_pos);
  // Close the Preferences
  preferences.end();


  //Serial.println("Going to sleep now");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);

  /***************************************************
     Setup Deep Sleep
   ***************************************************/
  esp_deep_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  //Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Minutes");
  esp_deep_sleep_start();
  delay(1000);
}



