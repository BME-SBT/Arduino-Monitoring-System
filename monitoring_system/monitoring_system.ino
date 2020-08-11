#include "I2Cdev.h"

#include "ArduinoJson.h"

#include "SoftwareSerial.h"

#include "string.h"

#include "MPU6050_6Axis_MotionApps20.h"

#include "Wire.h"

#include "string.h"

#include "mcp_can.h"

SoftwareSerial mySerial(5, 6);



MPU6050 mpu;



// MPU control/status vars

bool dmpReady = false;  // set true if DMP init was successful

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU

uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)

uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)

uint16_t fifoCount;     // count of all bytes currently in FIFO

uint8_t fifoBuffer[64]; // FIFO storage buffer



volatile int sensorInterrupt =false;

volatile int CANInterrupt = false;

int CAN_Init =0;

// Calculate the size of our Json object

// Help: https://arduinojson.org/v6/assistant/

const int jsonSize = JSON_ARRAY_SIZE(3) + 2 * JSON_OBJECT_SIZE(2) + 2 * JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(6);



// Allocate the JSON document

StaticJsonDocument<jsonSize> jsonDoc;



char jsonData[jsonSize];

String CAN_status;

String Sensor_status;

// orientation/motion vars

Quaternion q;           // [w, x, y, z]         quaternion container

VectorInt16 aa;         // [x, y, z]            accel sensor measurements

VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements

VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements

VectorFloat gravity;    // [x, y, z]            gravity vector

float euler[3];         // [psi, theta, phi]    Euler angle container

float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector





  unsigned const char CAN_len = 0;

  unsigned char CAN_buf[8];



    

const int spiCSPin = 53;





MCP_CAN CAN(spiCSPin);





float AccX, AccY, AccZ;



unsigned long time;

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



  //**********SENSOR INIT*******************

  Serial.begin(115200);



  mySerial.begin(115200);  //megoldottam hogy a nálam lévő modul 115200-on menjen ezért átírtam



  while (!Serial); // wait for enumeration, others continue immediately

//   initialize device



//Test if snesor is OK



time = millis();

while((mpu.testConnection()!=true)&& (sensorInterrupt==false))

{

  if(time > 5000)

  {

    sensorInterrupt = Serial.read();

  }

 Serial.println("Sensor init failed");

}





if(sensorInterrupt ==false)

{

  Sensor_status =String("Sensor init OK");

}

if(sensorInterrupt ==true)

{

  Sensor_status =String("Sensor init failed");

  }

  mpu.initialize();



 devStatus = mpu.dmpInitialize();



  if (devStatus == 0) {

    // turn on the DMP, now that it's ready

    mpu.setDMPEnabled(true);



    attachInterrupt(0, dmpDataReady, RISING);

    mpuIntStatus = mpu.getIntStatus();



//     set our DMP Ready flag so the main loop() function knows it's okay to use it

    dmpReady = true;



//     get expected DMP packet size for later comparison

    packetSize = mpu.dmpGetFIFOPacketSize();

  } else {

    // ERROR!

  }



  //**********SENSOR INIT*******************



    //**********CAN INIT*******************

    while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz))

   {

        //TODO error handling

        delay(100);

    }

  

 //**********CAN INIT*******************

}





void loop() {



  //********************SENSOR READ******************************

  

  // if programming failed, don't try to do anything

  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available

  //while (!mpuInterrupt && fifoCount < packetSize) {



 // }



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



//***************CAN READ***************************



 if(CAN_MSGAVAIL == CAN.checkReceive())

    {

        CAN.readMsgBuf(&CAN_len, CAN_buf);

        unsigned long canId = CAN.getCanId();  

    }



//***************CREATE JSON************************

    // Add values in the Json document

 jsonDoc["tilt"]["x"] = ypr[1] * 180 / M_PI;

    jsonDoc["tilt"]["y"] = ypr[2] * 180 / M_PI;

    jsonDoc["tilt"]["z"] = ypr[0] * 180 / M_PI;



    jsonDoc["acceleration"]["x"] = AccX;

    jsonDoc["acceleration"]["y"] = AccY;

    jsonDoc["acceleration"]["z"] = AccZ;



    jsonDoc["compass"]["x"] = 123;

    jsonDoc["compass"]["y"] = 456;

    jsonDoc["compass"]["z"] = 789;



   // jsonDoc["battery"]["BMS"] = CAN_buf[0];

   // jsonDoc["battery"]["BMS2"] = CAN_buf[2];

     int SoC  = CAN_buf[0] >>1;
     
     unsigned int PackVolt = 0;
     PackVolt |= CAN_buf[1]<<8;
     PackVolt |= CAN_buf[2];
     float floatPackVoltage = PackVolt/1000;
     
     int temperature = CAN_buf[4];
     
     unsigned int PackCurr = 0;
     PackCurr |= CAN_buf[5]<<8;
     PackCurr |= CAN_buf[6];
     float floatPackCurrent = PackCurr/1000;

    jsonDoc["battery"]["Package current"] = floatPackCurrent; //TODO

    jsonDoc["battery"]["Package voltage"] = floatPackVoltage;

    jsonDoc["battery"]["Package SoC"] = SoC;

    jsonDoc["battery"]["Temperature"] = temperature;



    jsonDoc["error"]["source"] = "";

    jsonDoc["error"]["message"] = "";

    // Add an array.

    JsonArray data = jsonDoc.createNestedArray("extra temps");

    data.add(12);

    data.add(34);

    data.add(56);



    //Serialize Json to the serial port


    serializeJson(jsonDoc, Serial); //or mySerial


    delay(1000);

  }

}
