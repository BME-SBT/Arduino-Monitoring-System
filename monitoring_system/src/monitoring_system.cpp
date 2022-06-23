#include <Arduino.h>

#include "I2Cdev.h"
#include "ArduinoJson.h"
#include "SoftwareSerial.h"
#include "string.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "string.h"
#include "mcp_can.h"

#define SPI_CS_PIN (53)
#define I2C_SPEED (24); // 400kHz I2C clock (200kHz if CPU is 8MHz)
#define SERIAL_BAUD_RATE (115200)
#define SW_SERIAL_RX (19)
#define SW_SERIAL_TX (18)

SoftwareSerial BLE_Serial(SW_SERIAL_RX, SW_SERIAL_TX); // for BLE modul

//********static variables******************

// MPU control/status vars
MPU6050 mpu;
static bool dmpReady = false;  // set true if DMP init was successful
static uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
static uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error
static uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
static uint16_t fifoCount;     // count of all bytes currently in FIFO
static uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
static Quaternion q;        // [w, x, y, z]         quaternion container
static VectorInt16 aa;      // [x, y, z]            accel sensor measurements
static VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
static VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
static VectorFloat gravity; // [x, y, z]            gravity vector
static float euler[3];      // [psi, theta, phi]    Euler angle container
static float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

static float floAccX;
static float floAccY;
static float floAccZ;

// CAN datas
static byte len = 0;

// BMS values
// static uint8_t u8SoC  = 0;
// static uint16_t u16PackVolt = 0;
// static float floPackVoltage = 0;
// static uint8_t u8Temperature = 0;
// static int16_t  s16PackCurr = 0;
// static float floPackCurrent = 0;

static MCP_CAN CAN(SPI_CS_PIN);

typedef union
{
    struct
    {
        uint16_t u16LowCellVoltage;  // 6-7.frame
        uint16_t u16HighCellVoltage; // 4-5.frame
        int16_t u16PackCurrent;      // 2-3.frame
        uint8_t u8HighTemp;          // 1.frame
        uint8_t u8SoC;               // 0.frame

    } cFRAMES;

    uint64_t u64Message; // 0.-7.frame

} uMESSAGE;

static uint8_t motor_data_raw[45];

static uMESSAGE uCAN_Msg;
static uint8_t bufin[8];
// JSON vars
//  Calculate the size of our Json object
//  Help: https://arduinojson.org/v6/assistant/
//  Hozzáadtam a low cell voltage-et, basztam kiszámolni az új méretet, úgyhogy csak beszoroztam az egészet kettővel. -Z
// Legyen az 3 - @Barrow099
static const int jsonSize = 3 * (JSON_ARRAY_SIZE(3) + 2 * JSON_OBJECT_SIZE(2) + 2 * JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(6));
static char jsonData[jsonSize];
// Allocate the JSON document
StaticJsonDocument<jsonSize> jsonDoc;

//********static function definitions*****************
static void vSensorInit(void);
static void vCANInit(void);
static bool boSensorRead(void);
static void vCANRead(void);
static void vSendDatasBLE(void);
static void readMotorData();

// interrupt detection routine
volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
bool motorReadSuccess = false;

void dmpDataReady()
{
    mpuInterrupt = true;
}
static void vSensorInit(void)
{
    // join I2C bus
    Wire.begin();

    TWBR = I2C_SPEED; // 400kHz I2C clock (200kHz if CPU is 8MHz)

    // Test if snesor is OK
    while ((mpu.testConnection() != true))
    {
        Serial.println("Sensor init failed");
    }
    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
    else
    {

        // ERROR!
    }
}

static void vCANInit(void)

{
    while (CAN_OK != CAN.begin(CAN_500KBPS, MCP_8MHz))
    {
    }
}

void setup()
{
    Serial.begin(115200);
    // Serial.begin(SERIAL_BAUD_RATE);
    BLE_Serial.begin(SERIAL_BAUD_RATE); // megoldottam hogy a nálam lévő modul 115200-on menjen ezért átírtam

    // while (!Serial)
    //{
    // wait for enumeration, others continue immediately
    //  }

    vSensorInit();
    vCANInit();

    // Initialize motor telemetry
    // HoTT bullshit uses 19200 baud UART
    Serial3.begin(19200);
}

/**
 * @brief Reads the motor telemetry from the motor controller
 * @author Barrow099
 */
static void readMotorData()
{
    motorReadSuccess = false;
    Serial3.write(0x80);
    Serial3.write(0x8d);
    delay(5);

    if (Serial3.readBytes(motor_data_raw, 2) == 2)
    {
        if (motor_data_raw[0] == 0x7c && motor_data_raw[1] == 0x8d)
        {
            Serial3.readBytes(motor_data_raw + 2, 43);
            if (motor_data_raw[43] == 0x7d)
                motorReadSuccess = true;
        }
    }
}

static bool boSensorRead(void)
{
    bool boSensReadAcc = false;

    // if programming failed, don't try to do anything
    if (!dmpReady)
    {
        return;
    }

    // wait for MPU interrupt or extra packet(s) available
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize)
        {
            fifoCount = mpu.getFIFOCount();
        }

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

        // For a range of +-2g, we need to divide the raw values by 1638.4, according to the datasheet
        floAccX = (Wire.read() << 8 | Wire.read()) / 1638.40; // X-axis value
        floAccY = (Wire.read() << 8 | Wire.read()) / 1638.40; // Y-axis value
        floAccZ = (Wire.read() << 8 | Wire.read()) / 1638.40; // Z-axis value

        boSensReadAcc = true;
    }
    return boSensReadAcc;
}

static void vCANRead(void)
{

    if (CAN_MSGAVAIL == CAN.checkReceive())
    {
        CAN.readMsgBuf(&len, bufin);
        unsigned long canId = CAN.getCanId();
    }
    // Filling the union with the given message
    uCAN_Msg.u64Message = 0U;
    uCAN_Msg.u64Message |= (uint64_t)bufin[0] << 56U;
    uCAN_Msg.u64Message |= (uint64_t)bufin[1] << 48U;
    uCAN_Msg.u64Message |= (uint64_t)bufin[2] << 40U;
    uCAN_Msg.u64Message |= (uint64_t)bufin[3] << 32U;
    uCAN_Msg.u64Message |= (uint64_t)bufin[4] << 24U;
    uCAN_Msg.u64Message |= (uint64_t)bufin[5] << 16U;
    uCAN_Msg.u64Message |= (uint64_t)bufin[6] << 8U;
    uCAN_Msg.u64Message |= (uint64_t)bufin[7];
}

static void vSendDatasBLE(void)
{
    // Add values in the Json document
    jsonDoc["tilt"]["x"] = ypr[1] * 180 / M_PI;
    jsonDoc["tilt"]["y"] = ypr[2] * 180 / M_PI;
    jsonDoc["tilt"]["z"] = ypr[0] * 180 / M_PI;

    jsonDoc["acceleration"]["x"] = floAccX / 9.81;
    jsonDoc["acceleration"]["y"] = floAccY / 9.81;
    ;
    jsonDoc["acceleration"]["z"] = floAccZ / 9.81;
    ;

    // jsonDoc["compass"]["x"] = 123;//unused
    // jsonDoc["compass"]["y"] = 456;//unused
    // jsonDoc["compass"]["z"] = 789;//unused

    // jsonDoc["motor"]["RpM"] = 123;//unused
    // jsonDoc["motor"]["temp"] = 123;//unused

    jsonDoc["battery"]["Package current"] = uCAN_Msg.cFRAMES.u16PackCurrent / 10.0;
    jsonDoc["battery"]["Package voltage low"] = uCAN_Msg.cFRAMES.u16LowCellVoltage / 10000.0;
    jsonDoc["battery"]["Package voltage"] = uCAN_Msg.cFRAMES.u16HighCellVoltage / 10000.0;
    jsonDoc["battery"]["Package SoC"] = uCAN_Msg.cFRAMES.u8SoC / 2.0;
    jsonDoc["battery"]["Temperature"] = uCAN_Msg.cFRAMES.u8HighTemp;

    // Serialize motor info
    if (motorReadSuccess)
    {
        jsonDoc["motor"]["temp1"] = motor_data_raw[16];
        jsonDoc["motor"]["temp2"] = motor_data_raw[17];
        jsonDoc["motor"]["rpm"] = (motor_data_raw[22] << 8) & motor_data_raw[21];
        jsonDoc["motor"]["current"] = (motor_data_raw[29] << 8) & motor_data_raw[28];
        jsonDoc["motor"]["success"] = true;
    }else {
        jsonDoc["motor"]["success"] = false;
    }
    // jsonDoc["error"]["source"] = "";//unused
    // jsonDoc["error"]["message"] = "";//unused

    Serial.print("I = ");
    Serial.print(uCAN_Msg.cFRAMES.u16PackCurrent / 10.0);
    Serial.print("; U = ");
    Serial.println(uCAN_Msg.cFRAMES.u16HighCellVoltage / 10000.0);

    // Serial.println(uCAN_Msg.cFRAMES.u16LowCellVoltage/10000.0);

    // Add an array.
    // JsonArray data = jsonDoc.createNestedArray("extra temps");//unused
    // data.add(12);//unused
    // data.add(34);//unused
    // data.add(56);//unused

    // Serialize Json to the serial port
    serializeJson(jsonDoc, BLE_Serial);
}

void loop()
{

    if (boSensorRead() == true)
    {

        vCANRead();
        readMotorData();
        vSendDatasBLE();

        delay(100);
    }
}
