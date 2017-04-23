
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_)
#include <SoftwareSerial.h>
#endif

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <utility/quaternion.h>
#include "Button.h"

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "HIDKeyConfig.h"
#include "BluefruitConfig.h"


/*=========================================================================
    APPLICATION SETTINGS

  â€‚ â€‚ FACTORYRESET_ENABLEâ€‚   â€‚  Perform a factory reset when running this sketch
  â€‚ â€‚
  â€‚ â€‚                           Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
  â€‚ â€‚                           running this at least once is a good idea.
  â€‚ â€‚
  â€‚ â€‚                           When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.â€‚ If you are making changes to your
  â€‚ â€‚                           Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.â€‚ Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
  â€‚ â€‚ â€‚ â€‚
  â€‚ â€‚                           Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    -----------------------------------------------------------------------*/
#define FACTORYRESET_ENABLE         1
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"
/*=========================================================================*/

Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (10)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

#define TRIGGER_PIN 12
#define RELOAD_PIN 11

#define TRIGGER_KEY HID_KEY_KEYPAD_0
#define RELOAD_KEY HID_KEY_KEYPAD_1
#define RECENTER_KEY HID_KEY_KEYPAD_2

Button triggerButton = Button(TRIGGER_PIN, HIGH);
Button reloadButton = Button(RELOAD_PIN, HIGH);

unsigned int newDebounceDelay = 30;

// hold trigger for 3 seconds to reset gun
unsigned long recenterTimeDelay = 4000;
unsigned long recenterTime = 0;

unsigned long BNO_LASTSAMPLE = 0;

imu::Quaternion InitQuaternion;
imu::Vector<3> IntVector ;

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}


void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}


/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);
  delay(500);


  triggerButton.setDebounceDelay(newDebounceDelay);
  //triggerButton.clearDebounceDelay();

  reloadButton.setDebounceDelay(newDebounceDelay);
  //reloadButton.clearDebounceDelay();


  Serial.begin(115200);

  Serial.println(F("Adafruit Bluefruit HID Keyboard Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'WildWestVR': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=WildWestVR" )) ) {
    error(F("Could not set device name?"));
  }

  /* Enable HID Service */
  Serial.println(F("Enable HID Service (including Keyboard): "));
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    if ( !ble.sendCommandCheckOK(F( "AT+BLEHIDEN=1" ))) {
      error(F("Could not enable Keyboard"));
    }
  } else
  {
    if (! ble.sendCommandCheckOK(F( "AT+BLEKEYBOARDEN=1"  ))) {
      error(F("Could not enable Keyboard"));
    }
  }

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }

  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }


  pinMode(LED_BUILTIN, OUTPUT);


  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);

  CalibrateGun();

  Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");


  displaySensorDetails();

  delay( 1000 );
  CalibrateGun();

}

String hex_to_str(uint8_t hex) {
  String str = String(hex, HEX);
  if (hex < 16) {
    str = "0" + str;
  }
  return str;
}

//calibration
void CalibrateGun()
{

  InitQuaternion = bno.getQuat().scale(1);
  IntVector = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{

  triggerButton.listen();
  reloadButton.listen();

  if (triggerButton.onPress())
  {
    ble.println("AT+BLEKEYBOARDCODE=00-00-" + hex_to_str( TRIGGER_KEY ));
    Serial.println("trigger");
    delay( 10 );
    //  release key
    ble.println("AT+BLEKEYBOARDCODE=00-00");

    recenterTime = millis() + recenterTimeDelay;
    digitalWrite(LED_BUILTIN, HIGH);
  }

  if (reloadButton.onPress())
  {
    ble.println("AT+BLEKEYBOARDCODE=00-00-" + hex_to_str( RELOAD_KEY ));
    Serial.println("reload");
    delay( 10 );
    //  release key
    ble.println("AT+BLEKEYBOARDCODE=00-00");
    digitalWrite(LED_BUILTIN, HIGH);
  }

  if ( triggerButton.onRelease( true ) || reloadButton.onRelease( true ) )
  {
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (triggerButton.isPressed())
  {
    if (  millis() > recenterTime )
    {

      CalibrateGun();

      ble.println("AT+BLEKEYBOARDCODE=00-00-" + hex_to_str( RECENTER_KEY ));
      Serial.println("recenter");
      delay( 10 );
      //  release key
      ble.println("AT+BLEKEYBOARDCODE=00-00");
      recenterTime = millis() + 10000;
    }
  }

  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2


  if (  BNO_LASTSAMPLE < millis() )
  {

    //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

    /*
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER) - IntVector;


      Serial.print(euler.x());
      Serial.print(",");

      Serial.print(euler.y());
      Serial.print(",");

      Serial.println(euler.z());



      ble.print("AT+BLEKEYBOARD=");
      ble.print(euler.x());
      ble.print(",");

      ble.print(euler.y());
      ble.print(",");

      ble.println(euler.z());

      .

    */
    /* Display the floating point data */

    /*Serial.print("X: ");
      Serial.print(euler.x());
      Serial.print(" Y: ");
      Serial.print(euler.y());
      Serial.print(" Z: ");
      Serial.print(euler.z());
      Serial.print("\t\t");
    */

    // Quaternion data
    /*imu::Quaternion quat = bno.getQuat();
      Serial.print("qW: ");
      Serial.print(quat.w(), 4);
      Serial.print(" qX: ");
      Serial.print(quat.y(), 4);
      Serial.print(" qY: ");
      Serial.print(quat.x(), 4);
      Serial.print(" qZ: ");
      Serial.print(quat.z(), 4);
      Serial.print("\t\t");
    */
    /* Display calibration status for each sensor. */
    /* uint8_t system, gyro, accel, mag = 0;
       bno.getCalibration(&system, &gyro, &accel, &mag);
       Serial.print("CALIBRATION: Sys=");
       Serial.print(system, DEC);
       Serial.print(" Gyro=");
       Serial.print(gyro, DEC);
       Serial.print(" Accel=");
       Serial.print(accel, DEC);
       Serial.print(" Mag=");
       Serial.println(mag, DEC);*/

    imu::Quaternion quat = bno.getQuat();

    imu::Quaternion diffQuat = quat * InitQuaternion.inverse();

    Serial.print(diffQuat.x(), 4);
    Serial.print(",");

    Serial.print(diffQuat.y(), 4);
    Serial.print(",");

    Serial.print(diffQuat.z(), 4);
    Serial.print(",");

    Serial.println(diffQuat.w(), 4);


    ble.print("AT+BLEKEYBOARD=");
    ble.print(diffQuat.x(), 4);
    ble.print(",");

    ble.print(diffQuat.y(), 4);
    ble.print(",");

    ble.print(diffQuat.z(), 4);
    ble.print(",");

    ble.println(diffQuat.w(), 4);


    BNO_LASTSAMPLE = millis() + BNO055_SAMPLERATE_DELAY_MS;
  }

  //delay(BNO055_SAMPLERATE_DELAY_MS);
}

