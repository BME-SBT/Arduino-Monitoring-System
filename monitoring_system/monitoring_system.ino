#include "I2Cdev.h"
#include "ArduinoJson.h"
#include "SoftwareSerial.h"
#include "string.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

SoftwareSerial mySerial(5, 6);

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Calculate the size of our Json object
// Help: https://arduinojson.org/v6/assistant/
const int jsonSize = JSON_ARRAY_SIZE(3) + 2 * JSON_OBJECT_SIZE(2) + 2 * JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(6);

// Allocate the JSON document
StaticJsonDocument<jsonSize> jsonDoc;

char jsonData[jsonSize];


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float AccX, AccY, AccZ;


//interrupt detection routine
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void setup() {
  // join I2C bus
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  // initialize serial communication
  Serial.begin(115200);
  mySerial.begin(115200);  //megoldottam hogy a nálam lévő modul 115200-on menjen ezért átírtam

  while (!Serial); // wait for enumeration, others continue immediately
  // initialize device
  mpu.initialize();

  devStatus = mpu.dmpInitialize();


  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
  }

}


void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {

  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;



    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    Wire.beginTransmission(0x68);
    Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true); // Read 6 registers total, each axis value is stored in 2 registers


    //For a range of +-2g, we need to divide the raw values by 1638.4, according to the datasheet
    AccX = (Wire.read() << 8 | Wire.read()) / 1638.40; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 1638.40; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 1638.40; // Z-axis value


    // Add values in the Json document
    jsonData["tilt"]["x"] = ypr[1] * 180 / M_PI;
    jsonData["tilt"]["y"] = ypr[2] * 180 / M_PI;
    jsonData["tilt"]["z"] = ypr[0] * 180 / M_PI;

    jsonData["acceleration"]["x"] = AccX;
    jsonData["acceleration"]["y"] = AccY;
    jsonData["acceleration"]["z"] = AccZ;

    jsonData["compass"]["x"] = 123;
    jsonData["compass"]["y"] = 456;
    jsonData["compass"]["z"] = 789;

    jsonData["motor"]["RpM"] = 123;
    jsonData["motor"]["temp"] = 123;

    jsonData["battery"]["in"] = 12;
    jsonData["battery"]["out"] = 34;
    jsonData["battery"]["SoC"] = 99;
    jsonData["battery"]["temp"] = 40;

    jsonData["error"]["source"] = "BMS/controller/etc";
    jsonData["error"]["message"] = "Something went wrong";

    // Add an array.
    JsonArray data = jsonData.createNestedArray("extra temps");
    data.add(12);
    data.add(34);
    data.add(56);

    //Serialize Json to the serial port
    serializeJson(jsonData, jsonData);

    Serial.write(jsonData);

    mySerial.write(data);

    delay(1000);
  }
}
