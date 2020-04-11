
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
StaticJsonDocument<size> jsonDoc;

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
  mySerial.begin(9600);
  
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

//float->string  converter function doesn't placed in the string.h library in arduino
void ftoa(float f, char *str, uint8_t precision) {
  uint8_t i, j, divisor = 1;
  int8_t log_f;
  int32_t int_digits = (int)f;             //store the integer digits
  float decimals;
  char s1[10];

  memset(str, 0, sizeof(s1));  
  memset(s1, 0, 10);

  if (f < 0) {                             //if a negative number 
    //str[0] = '-';                          //start the char array with '-', <- it doesn't need, it makes double -
    f = abs(f);                            //store its positive absolute value
  }
  log_f = ceil(log10(f));                  //get number of digits before the decimal
  if (log_f > 0) {                         //log value > 0 indicates a number > 1
    if (log_f == precision) {              //if number of digits = significant figures
      f += 0.5;                            //add 0.5 to round up decimals >= 0.5
      itoa(f, s1, 10);                     //itoa converts the number to a char array
      strcat(str, s1);                     //add to the number string
    }
    else if ((log_f - precision) > 0) {    //if more integer digits than significant digits
      i = log_f - precision;               //count digits to discard
      divisor = 10;
      for (j = 0; j < i; j++) divisor *= 10;    //divisor isolates our desired integer digits 
      f /= divisor;                             //divide
      f += 0.5;                            //round when converting to int
      int_digits = (int)f;
      int_digits *= divisor;               //and multiply back to the adjusted value
      itoa(int_digits, s1, 10);
      strcat(str, s1);
    }
    else {                                 //if more precision specified than integer digits,
      itoa(int_digits, s1, 10);            //convert
      strcat(str, s1);                     //and append
    }
  }

  else {                                   //decimal fractions between 0 and 1: leading 0
    s1[0] = '0';
    strcat(str, s1);
  }

  if (log_f < precision) {                 //if precision exceeds number of integer digits,
    decimals = f - (int)f;                 //get decimal value as float
    strcat(str, ".");                      //append decimal point to char array

    i = precision - log_f;                 //number of decimals to read
    for (j = 0; j < i; j++) {              //for each,
      decimals *= 10;                      //multiply decimals by 10
      if (j == (i-1)) decimals += 0.5;     //and if it's the last, add 0.5 to round it
      itoa((int)decimals, s1, 10);         //convert as integer to character array
      strcat(str, s1);                     //append to string
      decimals -= (int)decimals;           //and remove, moving to the next
    }
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
 

   ftoa(AccX, charAccX, 5);
   ftoa(AccY, charAccY, 5);
   ftoa(AccZ, charAccZ, 5);


   ypr[0]=0; 
   ypr[1] = ypr[1] * 180 / M_PI;
   ypr[2] = ypr[2] * 180 / M_PI;


   ftoa(ypr[0], charTiltZ, 5);
   ftoa(ypr[1], charTiltX, 5);
   ftoa(ypr[2], charTiltY, 5);
   
// Add values in the Json document
	doc["tilt"]["x"] = ypr[1] * 180 / M_PI;
	doc["tilt"]["y"] = ypr[2] * 180 / M_PI;
	doc["tilt"]["z"] = ypr[0] * 180 / M_PI;
	
	doc["acceleration"]["x"] = AccX;
	doc["acceleration"]["y"] = AccY;
	doc["acceleration"]["z"] = AccZ;

	doc["compass"]["x"] = 123;
	doc["compass"]["y"] = 456;
	doc["compass"]["z"] = 789;

	doc["motor"]["RpM"] = 123;
	doc["motor"]["temp"] = 123;

	doc["battery"]["in"] = 12;
	doc["battery"]["out"] = 34;
	doc["battery"]["SoC"] = 99;
	doc["battery"]["temp"] = 40;

	doc["error"]["source"] = "BMS/controller/etc";
	doc["error"]["message"] = "Something went wrong";

	// Add an array.
	//
	JsonArray data = doc.createNestedArray("extra temps");
	data.add(12);
	data.add(34);
	data.add(56);
         
    //Serialize Json to the serial port
    serializeJson(doc, jsonData);

    Serial.write(jsonData);
    mySerial.write(jsonData);
   
//--------------------------
   //
    
    //StaticJsonBuffer<200> jBuffer;
    //JsonArray& datas = jBuffer.createArray();

    //JsonObject& tilt = jBuffer.createObject();
    //tilt["tilt_x:"] = ypr[1] * 180 / M_PI;
    //tilt["tilt_y:"] = ypr[2] * 180 / M_PI;
    //tilt["z:"] =ypr[0]* 180/M_PI;
    //datas.add(tilt);

    //JsonObject& acceleration = jBuffer.createObject();

   // acceleration["acc_x:"] = AccX;
    //acceleration["acc_y:"] = AccY;
    //acceleration["acc_z:"] = AccZ;

    //datas.add(acceleration);

    //JsonObject& compass = jBuffer.createObject();

   // compass["compass"] = 0;
   // datas.add(compass);

// datas.prettyPrintTo(Serial);
//--------------------

    delay(1000);

    mySerial.write(data);
    
   
 }
}
