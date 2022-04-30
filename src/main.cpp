#include <Arduino.h>
#include <Wire.h>
#include "SPI.h"
#include <Adafruit_BMP280.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "RH_RF95.h"
#include "RHHardwareSPI1.h"
#include <TinyGPS++.h>
//#include "SdFat.h"
#include <SD.h>

// ======================================================== Defines ========================================================
// Status Indicator Pins
#define STATUS_LED_PIN        (6)
#define TE_RX_LED_PIN         (32)
#define TE_TX_LED_PIN         (31)
#define GPS_RX_LED_PIN        (36)
#define GPS_TX_LED_PIN        (37)
#define BUZZER_PIN            (2)

// Interface Pins
#define BMP_CS                (10)
#define TE_CS_PIN             (0)
#define TE_INT_PIN            (28)

// Ematch Pins
#define EMATCH_ARM_1          (4)
#define EMATCH_ARM_2          (5)
#define EMATCH_ARM_3          (41)
#define EMATCH_ARM_4          (40)
#define EMATCH_TEST_1         (17)
#define EMATCH_TEST_2         (16)
#define EMATCH_TEST_3         (39)
#define EMATCH_TEST_4         (38)
#define EMATCH_1              (23)
#define EMATCH_2              (3)
#define EMATCH_3              (30)
#define EMATCH_4              (33)

// V_Batt Pins
#define VBATT1                (21)
#define VBATT2                (22)
#define VBATT_12V             (29)

// LoRa Defines
#define LORA_PACKET_SIZE      (84)        // bytes
#define LORA_FREQ             (868.0)     // MHz
#define LORA_SF               (9)

#define LOCAL_PRESSURE        (1013.25)   // hPa

// Flight Parameters
#define EVENT_1_TIMER         (10000)     // seconds
#define EVENT_2_TIMER         (20000)     // seconds
#define EVENT_2_ALT           (0)         // meters
#define MOTOR_BURNOUT         (5000)      // milliseconds
#define MACH_DELAY            (5000)      // milliseconds

// Thresholds and Delay Defines
#define LIFTOFF_THRESHOLD     (0)         // m/s2
#define LIFTOFF_DELAY         (100)       // milliseconds
#define GROUND_ALT_THRESHOLD  (20)        // meters
#define TOUCHDOWN_DELAY       (10000)     // milliseconds

// ======================================================== Hardware Interfaces ========================================================
// LoRa: SPI1
// GPS: UART8
// FRAM: I2C2
// Baro: SPI0
// IMU: I2C0

// ======================================================== State Machine ========================================================
#define IDLE                  (1)
#define ARMED                 (2)
#define P_ASCENT              (3)
#define B_ASCENT              (4)
#define EVENT_1               (5)
#define EVENT_2               (6)
#define TOUCHDOWN             (7)

// ======================================================== LoRa Commands ========================================================
#define LORA_ARM              (0xA0)
#define LORA_RESET            (0xFF)

// ======================================================== Structures ========================================================
struct ACCEL
{
  float x;
  float y;
  float z;
};
struct GYRO
{
  float x;
  float y;
  float z;
};
struct TEMP
{
  float mpu;
  float bmp;
};
struct GPS
{
  float sats;
  float hdop;
  float longitude;
  float latitude;
  float altitude;
  float speed;
};

// ======================================================== Global Variables ========================================================
// SDFat Variables
File myFile;

// LoRa Variables
RH_RF95 rf95(TE_CS_PIN, TE_INT_PIN, hardware_spi1);
uint8_t lora_packet[RH_RF95_MAX_MESSAGE_LEN];

// MPU Variables
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// BMP Variables
Adafruit_BMP280 bmp(BMP_CS);

// GPS Variables
TinyGPSPlus gps;

//State Machine Variables
uint8_t current_state = IDLE;

//Battery Levels
float v_batt1;
float v_batt2;
float v_batt_12V;

// Ematch Variables
uint8_t ematch_1_status;
uint8_t ematch_2_status;
uint8_t ematch_3_status;
uint8_t ematch_4_status;

// Timers
uint32_t timer_burnout_init = 0;
uint32_t timer_event_1_init = 0;
uint32_t timer_event_2_init = 0;

// Misc. Variables
uint32_t liftoff_delay = false;
uint32_t liftoff_timer = 0;
uint8_t liftoff_status = 0;
uint8_t apogee_status = 0;
uint32_t touchdown_delay = 0;
uint32_t touchdown_timer = 0;
uint8_t touchdown_status = 0;
uint8_t first_execute = true;
uint8_t logging_enable = false;
uint16_t ground_altitude = 0;

// Sensor Variables
ACCEL accel_data;
GYRO gyro_data;
TEMP temp_data;
GPS gps_data;
float pressure;
float est_altitude;

// ======================================================== Function Prototypes ========================================================
void heartbeat(void);
void SD_init(void);
void MPU_init(void);
void MPU_read(void);
void BMP_init(void);
void BMP_read(void);
void lora_packet_build(void);
void lora_tx_handle(void);
void lora_rx_handle(void);
void lora_parse(uint8_t *buffer);
void GPS_read(void);
void state_update(void);
void safety_timer_event_1_handle(void);
void safety_timer_event_2_handle(void);
uint8_t liftoff_detect(void);
uint8_t apogee_detect(void);
uint8_t touchdown_detect(void);
void ematch_test(void);
void ematch_arm(void);
void ematch_disarm(void);
void battery_level_read(void);
void ematch_trigger(uint8_t ematch);
static void printStr(const char *str, int len);
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
static void printInt(unsigned long val, bool valid, int len);
static void printFloat(float val, bool valid, int len, int prec);
static void smartDelay(unsigned long ms);
void logData(void);

// ======================================================== Setup ========================================================
void setup() {

  // Pin Init
  pinMode(STATUS_LED_PIN,OUTPUT);
  pinMode(TE_RX_LED_PIN,OUTPUT);
  pinMode(TE_TX_LED_PIN,OUTPUT);
  pinMode(GPS_RX_LED_PIN,OUTPUT);
  pinMode(GPS_TX_LED_PIN,OUTPUT);
  pinMode(TE_CS_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(VBATT1, INPUT);
  pinMode(VBATT2, INPUT);
  pinMode(VBATT_12V, INPUT);
  pinMode(EMATCH_TEST_1, INPUT);
  pinMode(EMATCH_TEST_2, INPUT);
  pinMode(EMATCH_TEST_3, INPUT);
  pinMode(EMATCH_TEST_4, INPUT);
  pinMode(EMATCH_ARM_1, OUTPUT);
  pinMode(EMATCH_ARM_2, OUTPUT);
  pinMode(EMATCH_ARM_3, OUTPUT);
  pinMode(EMATCH_ARM_4, OUTPUT);
  pinMode(EMATCH_1, OUTPUT);
  pinMode(EMATCH_2, OUTPUT);
  pinMode(EMATCH_3, OUTPUT);
  pinMode(EMATCH_4, OUTPUT);

  Serial.begin(115200);
  while (!Serial);
  Serial8.begin(9600);
  while (!Serial8);
  
  // LoRa Init
  if (!rf95.init())
    Serial.println("init failed");
  rf95.setTxPower(12, false);
  rf95.setFrequency(LORA_FREQ);
  rf95.setSpreadingFactor(LORA_SF);

  // GPS Init
  Serial.println(F("Testing TinyGPSPlus library"));
  Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to Cernier  ----  RX    RX        Fail"));
  Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));

  // Sensor init
  MPU_init();
  BMP_init();

  // Safety Ematch Disarm
  ematch_disarm();

  // SD Init
  Serial.print("Initializing SD card...");
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("initialization failed!");
    return;
  }

  Serial.println("Initialization Complete.");
  Serial.println("SMORRT Ready for flight.");
}

// ======================================================== Loop ========================================================
void loop() {

  // Data Acquisition Handle
  MPU_read();
  BMP_read();
  GPS_read();
  battery_level_read();
  //TODO Moyenne mobile
  //TODO Recovery event triggering/arming/testing
  ematch_test();

  // State Machine Handle
  state_update();

  // LoRa Handle
  lora_packet_build();
  lora_tx_handle();
  lora_rx_handle();

  // SD Log
  if(logging_enable)
   logData();

  heartbeat();
  delay(1000);  
}

// ======================================================== Functions ========================================================
void heartbeat(void) {
  for (int i = 0; i < 2; i++) {
    digitalWrite(STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(STATUS_LED_PIN, LOW);
    delay(100);
  }
}

void MPU_init(void) {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  Serial.println("Accelerometer set to 16G!");
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.println("Gyro range set to 500deg!");
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("Filter bandwidth set to 21Hz!");
}

void MPU_read(void) {
  mpu.getEvent(&a, &g, &temp);

  accel_data.x = a.acceleration.x;
  accel_data.y = a.acceleration.y;
  accel_data.z = a.acceleration.z;
  gyro_data.x = g.gyro.x;
  gyro_data.y = g.gyro.y;
  gyro_data.z = g.gyro.z;
  temp_data.mpu = temp.temperature;

  Serial.print("Acceleration X: ");
  Serial.print(accel_data.x);
  Serial.print(", Y: ");
  Serial.print(accel_data.y);
  Serial.print(", Z: ");
  Serial.print(accel_data.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(gyro_data.x);
  Serial.print(", Y: ");
  Serial.print(gyro_data.y);
  Serial.print(", Z: ");
  Serial.print(gyro_data.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp_data.mpu);
  Serial.println(" degC");

  Serial.println("");
}

void BMP_init(void) {
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
    Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
    Serial.print("        ID of 0x60 represents a BME 280.\n");
    Serial.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */
}

void BMP_read(void) {

  temp_data.bmp = bmp.readTemperature();
  pressure = bmp.readPressure();
  est_altitude = bmp.readAltitude(LOCAL_PRESSURE);

  Serial.print(F("Temperature = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" °C");

  Serial.print(F("Pressure = "));
  Serial.print(bmp.readPressure());
  Serial.println(" Pa");

  Serial.print(F("Approx altitude = "));
  Serial.print(bmp.readAltitude(LOCAL_PRESSURE)); /* Adjusted to local forecast! */
  Serial.println(" m");

  Serial.println("");
  delay(100);
  }

void lora_packet_build(void) {

  uint8_t temp_buffer[sizeof(float)];

  // Timestamp
  memcpy(temp_buffer, &(temp.timestamp), sizeof(float));
  lora_packet[0] = temp_buffer[0];
  lora_packet[1] = temp_buffer[1];
  lora_packet[2] = temp_buffer[2];
  lora_packet[3] = temp_buffer[3];

  // IMU Data
  memcpy(temp_buffer, &(accel_data.x), sizeof(float));
  lora_packet[4] = temp_buffer[0];
  lora_packet[5] = temp_buffer[1];
  lora_packet[6] = temp_buffer[2];
  lora_packet[7] = temp_buffer[3];
  memcpy(temp_buffer, &(accel_data.y), sizeof(float));
  lora_packet[8] = temp_buffer[0];
  lora_packet[9] = temp_buffer[1];
  lora_packet[10] = temp_buffer[2];
  lora_packet[11] = temp_buffer[3];
  memcpy(temp_buffer, &(accel_data.z), sizeof(float));
  lora_packet[12] = temp_buffer[0];
  lora_packet[13] = temp_buffer[1];
  lora_packet[14] = temp_buffer[2];
  lora_packet[15] = temp_buffer[3];
  memcpy(temp_buffer, &(gyro_data.x), sizeof(float));
  lora_packet[16] = temp_buffer[0];
  lora_packet[17] = temp_buffer[1];
  lora_packet[18] = temp_buffer[2];
  lora_packet[19] = temp_buffer[3];
  memcpy(temp_buffer, &(gyro_data.y), sizeof(float));
  lora_packet[20] = temp_buffer[0];
  lora_packet[21] = temp_buffer[1];
  lora_packet[22] = temp_buffer[2];
  lora_packet[23] = temp_buffer[3];
  memcpy(temp_buffer, &(gyro_data.z), sizeof(float));
  lora_packet[24] = temp_buffer[0];
  lora_packet[25] = temp_buffer[1];
  lora_packet[26] = temp_buffer[2];
  lora_packet[27] = temp_buffer[3];

  // Baro Data
  memcpy(temp_buffer, &(temp_data.mpu), sizeof(float));
  lora_packet[28] = temp_buffer[0];
  lora_packet[29] = temp_buffer[1];
  lora_packet[30] = temp_buffer[2];
  lora_packet[31] = temp_buffer[3];
  memcpy(temp_buffer, &(temp_data.bmp), sizeof(float));
  lora_packet[32] = temp_buffer[0];
  lora_packet[33] = temp_buffer[1];
  lora_packet[34] = temp_buffer[2];
  lora_packet[35] = temp_buffer[3];
  memcpy(temp_buffer, &pressure, sizeof(float));
  lora_packet[36] = temp_buffer[0];
  lora_packet[37] = temp_buffer[1];
  lora_packet[38] = temp_buffer[2];
  lora_packet[39] = temp_buffer[3];
  memcpy(temp_buffer, &est_altitude, sizeof(float));
  lora_packet[40] = temp_buffer[0];
  lora_packet[41] = temp_buffer[1];
  lora_packet[42] = temp_buffer[2];
  lora_packet[43] = temp_buffer[3];

  // Battery Levels
  memcpy(temp_buffer, &v_batt1, sizeof(float));
  lora_packet[44] = temp_buffer[0];
  lora_packet[45] = temp_buffer[1];
  lora_packet[46] = temp_buffer[2];
  lora_packet[47] = temp_buffer[3];
  memcpy(temp_buffer, &v_batt2, sizeof(float));
  lora_packet[48] = temp_buffer[0];
  lora_packet[49] = temp_buffer[1];
  lora_packet[50] = temp_buffer[2];
  lora_packet[51] = temp_buffer[3];
  memcpy(temp_buffer, &v_batt_12V, sizeof(float));
  lora_packet[52] = temp_buffer[0];
  lora_packet[53] = temp_buffer[1];
  lora_packet[54] = temp_buffer[2];
  lora_packet[55] = temp_buffer[3];

  // Status
  lora_packet[56] = current_state;
  lora_packet[57] = ematch_1_status;
  lora_packet[58] = ematch_2_status;
  lora_packet[59] = ematch_3_status;
  lora_packet[60] = ematch_4_status;

  // GPS Data
  memcpy(temp_buffer, &(gps_data.sats), sizeof(float));
  lora_packet[61] = temp_buffer[0];
  lora_packet[62] = temp_buffer[1];
  lora_packet[63] = temp_buffer[2];
  lora_packet[64] = temp_buffer[3];
  memcpy(temp_buffer, &(gps_data.hdop), sizeof(float));
  lora_packet[65] = temp_buffer[0];
  lora_packet[66] = temp_buffer[1];
  lora_packet[67] = temp_buffer[2];
  lora_packet[68] = temp_buffer[3];
  memcpy(temp_buffer, &(gps_data.longitude), sizeof(float));
  lora_packet[69] = temp_buffer[0];
  lora_packet[70] = temp_buffer[1];
  lora_packet[71] = temp_buffer[2];
  lora_packet[72] = temp_buffer[3];
  memcpy(temp_buffer, &(gps_data.latitude), sizeof(float));
  lora_packet[73] = temp_buffer[0];
  lora_packet[74] = temp_buffer[1];
  lora_packet[75] = temp_buffer[2];
  lora_packet[76] = temp_buffer[3];
  memcpy(temp_buffer, &(gps_data.altitude), sizeof(float));
  lora_packet[77] = temp_buffer[0];
  lora_packet[78] = temp_buffer[1];
  lora_packet[79] = temp_buffer[2];
  lora_packet[80] = temp_buffer[3];
  memcpy(temp_buffer, &(gps_data.speed), sizeof(float));
  lora_packet[81] = temp_buffer[0];
  lora_packet[82] = temp_buffer[1];
  lora_packet[83] = temp_buffer[2];
  lora_packet[84] = temp_buffer[3];
 
}

void lora_tx_handle(){
  Serial.println("Sending to Ground Station");
  // Send a message to rf95_server
  digitalWrite(TE_TX_LED_PIN, HIGH);
  rf95.send(lora_packet, sizeof(lora_packet));

  rf95.waitPacketSent();
  digitalWrite(TE_TX_LED_PIN, LOW);
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(3000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      digitalWrite(TE_RX_LED_PIN, HIGH);
      Serial.print("got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      digitalWrite(TE_RX_LED_PIN, LOW);
    }
    else
    {
      Serial.println("recv failed");
    }
  }
  else
  {
    Serial.println("No reply, is GRUND running?");
  }
  delay(100);
}

void lora_rx_handle(){

  if (rf95.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len))
    {
      digitalWrite(TE_RX_LED_PIN, HIGH);
//      RH_RF95::printBuffer("request: ", buf, len);
      Serial.print("got request: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      
      lora_parse(buf);

      // Send a reply
      uint8_t data[] = "ACK";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent ACK");
      digitalWrite(TE_RX_LED_PIN, LOW);
    }
    else
    {
      Serial.println("recv failed");
    }
  }
}

void lora_parse(uint8_t *buffer){
  switch(buffer[0]){
    case LORA_ARM:
      current_state = ARMED;
      break;
    
    case LORA_RESET:
      SCB_AIRCR = 0x05FA0004;
      asm volatile ("dsb");
      break;

    default:
      break;
  }
}

void state_update(void){

  Serial.print("Current State: ");
  Serial.println(current_state);
  switch(current_state){
    case IDLE:
      if(first_execute){
        ematch_disarm();
        ground_altitude = bmp.readAltitude(LOCAL_PRESSURE);
        logging_enable = false;
        first_execute = false;
      }
      // if Telemetry command or vertical for a little while -> current_state = ARMED;
      // Liftoff Detection
      if(liftoff_detect()){
        current_state = P_ASCENT;
        first_execute = true;
        timer_event_1_init = millis();
        timer_event_2_init = millis();
        ematch_arm();
        logging_enable = true;
      }
      
      break;
    case ARMED:
      if(first_execute){
        ematch_arm();
        logging_enable = true;
        first_execute = false;
      }

      // Liftoff Detection
      if(liftoff_detect()){
        current_state = P_ASCENT;
        first_execute = true;
        timer_event_1_init = millis();
        timer_event_2_init = millis();
      }
      break;
    case P_ASCENT:
      if(first_execute){
          timer_burnout_init = millis();
          first_execute = false;
        }
      if((millis() - timer_burnout_init) > MOTOR_BURNOUT){
        current_state = B_ASCENT;
        first_execute = true;
      }
      //if apogee detection or burn timer -> current_state = B_ASCENT
      break;
    case B_ASCENT:
      //apogee detection or safety timer -> current_state = EVENT_1
      if(apogee_detect()){
        current_state = EVENT_1;
        first_execute = true;
      }
      else if((millis() - timer_event_1_init) > EVENT_1_TIMER){
        current_state = EVENT_1;
        first_execute = true;
      }
      break;
    case EVENT_1:
      if(first_execute){
        ematch_trigger(EMATCH_1);
        ematch_trigger(EMATCH_2);
        first_execute = false;
      }
      //altitude detection + safety timer -> current_state = EVENT_2
      if(est_altitude <= EVENT_2_ALT){
        current_state = EVENT_2;
        first_execute = true;
      }
      else if((millis() - timer_event_2_init) > EVENT_2_TIMER){
        current_state = EVENT_2;
        first_execute = true;
      }
      break;
    case EVENT_2:
      if(first_execute){
        ematch_trigger(EMATCH_3);
        ematch_trigger(EMATCH_4);
        first_execute = false;
      }
      if(touchdown_detect()){
        current_state = TOUCHDOWN;
        first_execute = true;
      }
      //no movement for a bit -> current_state = TOUCHDOWN
      break;
    case TOUCHDOWN:
      ematch_disarm();
      logging_enable = false;
      //log on SD then back to idle when done
      break;
    default: // ERROR
      //Error handling
      first_execute = true;
      SCB_AIRCR = 0x05FA0004;
      asm volatile ("dsb");
      break;
  }
}

uint8_t liftoff_detect(void){

  if((accel_data.y < LIFTOFF_THRESHOLD) && liftoff_delay == false){
  liftoff_timer = millis();
  liftoff_delay = true;
  }
  if((millis() - liftoff_timer > LIFTOFF_DELAY) && (accel_data.y < LIFTOFF_THRESHOLD) && (liftoff_delay = true)){
    liftoff_status = true;
  }
  else{
    liftoff_status = false;
  }
  return liftoff_status;
}

uint8_t apogee_detect(void){

  return apogee_status;
}

uint8_t touchdown_detect(void){
  if((abs(est_altitude - ground_altitude) < GROUND_ALT_THRESHOLD) && (touchdown_delay == false)){
  touchdown_timer = millis();
  touchdown_delay = true;
  }
  if((millis() - touchdown_timer > TOUCHDOWN_DELAY) && (abs(est_altitude - ground_altitude) < GROUND_ALT_THRESHOLD) && (touchdown_delay = true)){
    touchdown_status = true;
  }
  else{
    touchdown_status = false;
  }
  return touchdown_status;
}

void ematch_test(void){
  ematch_1_status = digitalRead(EMATCH_TEST_1);
  ematch_2_status = digitalRead(EMATCH_TEST_2);
  ematch_3_status = digitalRead(EMATCH_TEST_3);
  ematch_4_status = digitalRead(EMATCH_TEST_4);
}

void ematch_arm(void){
  digitalWrite(EMATCH_ARM_1, HIGH);
  digitalWrite(EMATCH_ARM_2, HIGH);
  digitalWrite(EMATCH_ARM_3, HIGH);
  digitalWrite(EMATCH_ARM_4, HIGH);
}

void ematch_disarm(void){
  digitalWrite(EMATCH_ARM_1, LOW);
  digitalWrite(EMATCH_ARM_2, LOW);
  digitalWrite(EMATCH_ARM_3, LOW);
  digitalWrite(EMATCH_ARM_4, LOW);
}

void ematch_trigger(uint8_t ematch){
  digitalWrite(ematch, HIGH);
}

void battery_level_read(void){
  v_batt1 = analogRead(VBATT2)*32.0/20.0*3.3/1024;
  v_batt2 = analogRead(VBATT1)*32.0/20.0*3.3/1024;
  v_batt_12V = analogRead(VBATT_12V)*88.0/20.0*3.3/1024;
}

void GPS_read(void){
  static const double CERNIER_LAT = 47.051906, CERNIER_LON = 6.916414;
  digitalWrite(GPS_RX_LED_PIN, HIGH);

  gps_data.sats = gps.satellites.value();
  gps_data.hdop = gps.hdop.value();
  gps_data.longitude = gps.location.lng();
  gps_data.latitude = gps.location.lat();
  gps_data.altitude = gps.altitude.meters();
  gps_data.speed = gps.speed.mps();

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  unsigned long distanceKmToCernier =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      CERNIER_LAT, 
      CERNIER_LON) / 1000;
  printInt(distanceKmToCernier, gps.location.isValid(), 9);

  double courseToCernier =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      CERNIER_LAT, 
      CERNIER_LON);

  printFloat(courseToCernier, gps.location.isValid(), 7, 2);

  const char *cardinalToCernier = TinyGPSPlus::cardinal(courseToCernier);

  printStr(gps.location.isValid() ? cardinalToCernier : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  digitalWrite(GPS_RX_LED_PIN, LOW);

  smartDelay(100);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial8.available())
      gps.encode(Serial8.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

void logData(){
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  myFile = SD.open("log_Cernier.txt", FILE_WRITE);

  // if the file opened okay, write to it:
  if (myFile) {
    myFile.print(temp.timestamp);
    myFile.print(" ; ");
    myFile.print(a.acceleration.x);
    myFile.print(" ; ");
    myFile.print(a.acceleration.y);
    myFile.print(" ; ");
    myFile.print(a.acceleration.z);
    myFile.print(" ; ");
    myFile.print(a.gyro.x);
    myFile.print(" ; ");
    myFile.print(a.gyro.y);
    myFile.print(" ; ");
    myFile.print(a.gyro.z);
    myFile.print(" ; ");
    myFile.print(temp.temperature);
    myFile.print(" ; ");
    myFile.print(bmp.readTemperature());
    myFile.print(" ; ");
    myFile.print(bmp.readPressure());
    myFile.print(" ; ");
    myFile.print(bmp.readAltitude(LOCAL_PRESSURE));
    myFile.print(" ; ");
    myFile.print(v_batt1);
    myFile.print(" ; ");
    myFile.print(v_batt2);
    myFile.print(" ; ");
    myFile.print(v_batt_12V);
    myFile.print(" ; ");
    myFile.print(current_state);
    myFile.print(" ; ");
    myFile.print(ematch_1_status);
    myFile.print(" ; ");
    myFile.print(ematch_2_status);
    myFile.print(" ; ");
    myFile.print(ematch_3_status);
    myFile.print(" ; ");
    myFile.print(ematch_4_status);
    myFile.print(" ; ");
    myFile.print(gps.satellites.value());
    myFile.print(" ; ");
    myFile.print(gps.hdop.value());
    myFile.print(" ; ");
    myFile.print(gps.location.lng());
    myFile.print(" ; ");
    myFile.print(gps.location.lat());
    myFile.print(" ; ");
    myFile.print(gps.altitude.meters());
    myFile.print(" ; ");
    myFile.print(gps.speed.mps());
    myFile.println("");
    // close the file:
    myFile.close();
    Serial.println("done.");
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening file");
  }

}
// ======================================================== OLD CODE, might be useful ========================================================
/* IN LOOP
uint8_t data[] = "NA"; //Default ping
TxHandle(data);
RxHandle();  


void RxHandle(void) {
  if (rf95.available())
  {
    // Should be a message for us now
    uint8_t buf_Rx[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len_Rx = sizeof(buf_Rx);
    if (rf95.recv(buf_Rx, &len_Rx))
    {
      for (int i = 0; i < 2; i++) {
    heartbeat();
  }
      digitalWrite(LED_HEARTBEAT, HIGH);
      #ifdef DEBUG
      Serial.print("got request of length: ");
      Serial.print(len_Rx);
      Serial.print(", and payload: ");
      Serial.println((char*)buf_Rx);
      #endif
      //      Serial.print("RSSI: ");
      //      Serial.println(rf95.lastRssi(), DEC);
      if((buf_Rx[0] == 'R') && (buf_Rx[1] == 'G')){
        //RING
        digitalWrite(LED_HEARTBEAT, HIGH);
        digitalWrite(1, HIGH);
      }
      else if((buf_Rx[0] == 'S') && (buf_Rx[1] == 'T')){
        // Stop ringing
        digitalWrite(LED_HEARTBEAT, HIGH);
        digitalWrite(1, LOW);
      }
      rf95.send(buf_Rx, len_Rx);
      rf95.waitPacketSent();
      #ifdef DEBUG
      Serial.println("Sent Ack");
      #endif
      digitalWrite(LED_HEARTBEAT, LOW);
      heartbeat();
    }
    else
    {
      #ifdef DEBUG
      Serial.println("recv failed");
      #endif
    }
  }
  heartbeat();
  delay(100);
}

void TxHandle(uint8_t data[]) {
  rf95.send(data, sizeof(data));

  rf95.waitPacketSent();
  // Now wait for a reply
  uint8_t buf_Tx[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len_Tx = sizeof(buf_Tx);
  if (rf95.waitAvailableTimeout(2000))
  {
    // Should be a reply message for us now
    if (rf95.recv(buf_Tx, &len_Tx))
    {
      digitalWrite(LED_HEARTBEAT, HIGH);
      #ifdef DEBUG
      Serial.print("got reply of length: ");
      Serial.print(len_Tx);
      Serial.print(", and payload: ");
      Serial.println((char*)buf_Tx);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      #endif
      if((buf_Tx[0] == 'R') && (buf_Tx[1] == 'G')){
        //RING
        digitalWrite(LED_HEARTBEAT, HIGH);
        digitalWrite(1, HIGH);
      }
      else if((buf_Tx[0] == 'S') && (buf_Tx[1] == 'T')){
        // Stop ringing
        digitalWrite(LED_HEARTBEAT, HIGH);
        digitalWrite(1, LOW);
      }
      digitalWrite(LED_HEARTBEAT, LOW);
      heartbeat();
    }
    else
    {
      #ifdef DEBUG
      Serial.println("recv failed");
      #endif
    }
  }
  else
  {
    #ifdef DEBUG
    Serial.println("No reply, is Dispenser running?");
    #endif
  }
}*/