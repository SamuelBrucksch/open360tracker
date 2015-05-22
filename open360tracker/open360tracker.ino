/* Open Source 360° continuous rotation antenna tracker software
 * created by Samuel Brucksch
 *
 * Digital Smooth method from Arduino playground: http://playground.arduino.cc/Main/DigitalSmooth
 * Local GPS uses parser from TinyGPS: http://arduiniana.org/libraries/tinygps/
 * FPV Community thread: http://fpv-community.de/showthread.php?46439-360%B0-Community-Antennentracker-mit-Schleifring-Wer-macht-mit
 */
#include <EEPROM.h>
#include "defines.h"
#include "config.h"
#include <Wire.h>
#include "compass.h"
#include "servos.h"
#include "inttypes.h"
#include "telemetry.h"
#include "math.h"
#include <TinyGPS.h>
#ifdef Mavlink
#include <Mavlink.h>
#endif

void calcTilt();
void getError();
void calculatePID();
void getBatterieVoltage();

unsigned long time;
unsigned long calib_timer;

//PID stuff
long Error[11];
long Accumulator;
long PID;
uint8_t Divider = 15;
int PWMOutput;
long Dk;

// The target position (lat/lon)
geoCoordinate_t targetPosition;
// The tracker position (lat/lon)
geoCoordinate_t trackerPosition;

//only use tinygps when local gps is used
#ifdef LOCAL_GPS
uint8_t localSats;
#include <SoftwareSerial.h>
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPS gps;
void initGps();
#define START_TRACKING_DISTANCE 0
#endif

#ifdef LCD_DISPLAY
//download from https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
#if (LCD_DISPLAY == I2C)
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7);
#elif (LCD_DISPLAY == SPI)
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 13, 4, 3, 2);
#endif

char lcd_str[24];
long lcd_time;
#endif

#ifdef BATTERYMONITORING
  #ifndef BATTERYMONITORING_AVERAGE
    #define BATTERYMONITORING_AVERAGE 2
  #endif
  #ifndef BATTERYMONITORING_VREF
    #define BATTERYMONITORING_VREF 1.1
  #endif
  #ifndef BATTERYMONITORING_VREF_SOURCE
    #define BATTERYMONITORING_VREF_SOURCE INTERNAL
  #endif
  uint16_t Bat_ADC_Last[BATTERYMONITORING_AVERAGE];
  float Bat_Voltage;
  float Bat_denominator = (float)BATTERYMONITORING_RESISTOR_2 / ((float)BATTERYMONITORING_RESISTOR_1 + (float)BATTERYMONITORING_RESISTOR_2);
#endif

#ifdef MFD
uint16_t distance;
#endif

#ifdef SERVOTEST
int p = P;
int i = I;
int d = D;
int tilt = 0;
#endif

void setup()
{
  #ifdef BATTERYMONITORING
    pinMode(VOLTAGEDIVIDER, INPUT);
    analogReference(BATTERYMONITORING_VREF_SOURCE);
    int n = 0;
    for (n = 0; n < BATTERYMONITORING_AVERAGE; n++) {
      getBatterieVoltage();
    }
  #endif
#ifdef LCD_DISPLAY
  lcd.begin(16, LCD_SIZE_ROW);
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home();
  #if LCD_SIZE_ROW == 4
      lcd.setCursor ( 0, 1 );
  #endif
  lcd.print(" open360tracker ");
  #if LCD_SIZE_ROW == 4
    lcd.setCursor ( 0, 2 );
  #elif LCD_SIZE_ROW == 2
    lcd.setCursor ( 0, 1 );
  #endif
  lcd.print(" open360tracker ");
  lcd.setCursor ( 0, 1 );
  lcd.print("   version ");
  lcd.print(FMW_VERSION);
  lcd.print("  ");
#endif

  HAS_ALT = false;
  HAS_FIX = false;
#ifdef SERVOTEST
  HOME_SET = true;
  TRACKING_STARTED = true;
#else
  HOME_SET = false;
  TRACKING_STARTED = false;
#endif
  SETTING_HOME = false;
  PREVIOUS_STATE = true;

  CURRENT_STATE = true;
  NEW_HEADING = false;
#ifdef MFD
  TEST_MODE = false;
#endif

  Serial.begin(BAUD);

#ifdef DEBUG
  Serial.println("Setup start");
#endif

  pinMode(LED_PIN, OUTPUT);
  pinMode(HOME_BUTTON, INPUT);
  pinMode(CALIB_BUTTON, INPUT);

  //LED off
  digitalWrite(LED_PIN, LOW);

  //enable internal pullups
  digitalWrite(HOME_BUTTON, HIGH);
  digitalWrite(CALIB_BUTTON, HIGH);

  Wire.setClock(uint32_t(400000L));
  Wire.begin();

  // init pan/tilt servos controlled via hardware pwm
#ifdef DEBUG
  Serial.println("Init Servos");
#endif
  initServos();

#ifdef DEBUG
  Serial.println("Init Compass");
#endif
  initCompass();

#ifdef LOCAL_GPS
#ifdef DEBUG
  Serial.println("Init local GPS");
#endif
  gpsSerial.begin(GPS_BAUDRATE);
  //let gps serial initialize before we configure it
  delay(20);
  initGps();
#endif

#ifdef LCD_DISPLAY
  lcd.clear();
  lcd.setCursor(0, 0);
  sprintf(lcd_str, "HDG:%03u AZI:%03u", 0, 0);
  lcd.print(lcd_str);
  lcd.setCursor(0, 1);
  sprintf(lcd_str, "A:%05d D:%05u", 0, 0);
  lcd.print(lcd_str);
  lcd_time = millis();
#endif

  time = millis();

#ifdef DEBUG
  Serial.println("Setup finished");
#endif
}

#ifdef SERVOTEST
long servoTimer = 0;
#endif

void loop()
{
#ifdef SERVOTEST
  if (millis() - servoTimer > 200) {
    Serial.print("Heading: "); Serial.print(trackerPosition.heading / 10);
    Serial.print(" Target Heading: "); Serial.print(targetPosition.heading / 10);
    Serial.print(" PAN: "); Serial.print(PWMOutput);
    Serial.print(" TILT: "); Serial.print(tilt);
    Serial.print(" P: "); Serial.print(p);
    Serial.print(" I: "); Serial.print(i);
    Serial.print(" D: "); Serial.println(d);
    servoTimer = millis();
  }
#endif

  //TODO change to telemetry serial port
  if (Serial.available() > 1)
  {
    uint8_t c = Serial.read();
#ifdef SERVOTEST
    if (c == 'H' || c == 'h') {
      //target heading in degree
      targetPosition.heading = Serial.parseInt();
    } else if (c == 'T' || c == 't') {
      //tilt angle in degree
      int value = Serial.parseInt();
      if (value > 90)
        value = 90;
      else if (value < 0)
        value = 0;
      tilt = map(value, 0, 90, TILT_0, TILT_90);
      SET_TILT_SERVO_SPEED(tilt);
    } else if (c == 'M' || c == 'm') {
      //tilt angle in ms
      tilt = Serial.parseInt();
      SET_TILT_SERVO_SPEED(tilt);
    } else if (c == 'P' || c == 'p') {
      p = Serial.parseInt();
    } else if (c == 'I' || c == 'i') {
      i = Serial.parseInt();
    } else if (c == 'D' || c == 'd') {
      d = Serial.parseInt();
    } else if (c == 'C' || c == 'c') {
      calibrate_compass();
    }
#else
    encodeTargetData(c);
#endif
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
#ifndef SERVOTEST
#ifdef LCD_DISPLAY
  if (millis() > lcd_time) {
    int lcd_nr;
    //switch screen every X seconds
    #if LCD_SIZE_ROW == 2
      if ((millis() % 10000 < 7000)) {
        lcd_nr = 1;
      }
      else {
        lcd_nr = 2;
      }
    #elif LCD_SIZE_ROW == 4
      lcd_nr = 0;
    #endif
    if (lcd_nr == 0 || lcd_nr == 1) {
      //headings, alt, distance, sats
      lcd.setCursor(0, 0);
#ifdef MFD
      sprintf(lcd_str, "H:%03u A:%03u", trackerPosition.heading / 10, targetPosition.heading / 10);
      lcd.print(lcd_str);
#else
      #ifdef BATTERYMONITORING
        sprintf(lcd_str, "H:%03u V%02u.%01u ", trackerPosition.heading / 10, (uint16_t)Bat_Voltage,(uint16_t)(Bat_Voltage*10)%10);
      #else
        sprintf(lcd_str, "H:%03u A:%03u ", trackerPosition.heading / 10, targetPosition.heading / 10);
      #endif
      
      lcd.print(lcd_str);
      
      #ifdef LOCAL_GPS
        sprintf(lcd_str, "S:%02d", localSats);
      #else
        sprintf(lcd_str, "S:%02d", getSats());
      #endif
        lcd.print(lcd_str);
#endif
      lcd.setCursor(0, 1);
      sprintf(lcd_str, "A:%05d  D:%05u", targetPosition.alt/100, targetPosition.distance);
      lcd.print(lcd_str);
    }
    if (lcd_nr == 0 || lcd_nr == 2) {
      //lat, lon
      #if LCD_SIZE_ROW == 4
        lcd.setCursor ( 0, 2 );
      #else
        lcd.setCursor(0, 0);
      #endif
      lcd.print("T LAT:");
      dtostrf(targetPosition.lat / 100000.0f, 10, 5, lcd_str);
      lcd.print(lcd_str);
      #if LCD_SIZE_ROW == 4
        lcd.setCursor ( 0, 3 );
      #else
        lcd.setCursor(0, 1);
      #endif
      lcd.print("T LON:");
      dtostrf(targetPosition.lon / 100000.0f, 10, 5, lcd_str);
      lcd.print(lcd_str);
    }
    #ifdef BATTERYMONITORING
      getBatterieVoltage();
    #endif
    lcd_time = millis() + 200;
  }
#endif

  if (HAS_ALT) {
    targetPosition.alt = getTargetAlt();

    // mfd has all the data at once, so we do not have to wait for valid lat/lon
#ifdef MFD
    distance = getDistance();
    targetPosition.heading = getAzimuth() * 10;
#ifdef DEBUG
    Serial.print("Target alt: "); Serial.print(targetPosition.alt/100);
    Serial.print(" Target distance: "); Serial.print(targetPosition.distance);
    Serial.print(" Target heading: "); Serial.print(targetPosition.heading / 10);
    Serial.print(" Tracker heading: "); Serial.print(trackerPosition.heading / 10);
    Serial.print(" Target Sats: "); Serial.println(getSats());
#endif
#else
#ifdef DEBUG
    Serial.print("Target alt: "); Serial.println(targetPosition.alt/100);
#endif
#endif

    HAS_ALT = false;
  }

#ifndef MFD
  //only calculate distance and heading when we have valid telemetry data
  if (HAS_FIX) {
    targetPosition.lat = getTargetLat();
    targetPosition.lon = getTargetLon();

    // calculate distance without haversine. We need this for the slope triangle to get the correct pan value
    //distance = sqrt(sq(trackerPosition.lat - targetPosition.lat) + sq(trackerPosition.lon - targetPosition.lon));
    if (HOME_SET) {
      targetPosition.distance = TinyGPS::distance_between(trackerPosition.lat / 100000.0f, trackerPosition.lon / 100000.0f, targetPosition.lat / 100000.0f, targetPosition.lon / 100000.0f);
    }
    targetPosition.heading = TinyGPS::course_to(trackerPosition.lat / 100000.0f, trackerPosition.lon / 100000.0f, targetPosition.lat / 100000.0f, targetPosition.lon / 100000.0f) * 10.0f;

    //calcTargetDistanceAndHeading(&trackerPosition, &targetPosition);

#ifdef DEBUG
    Serial.print("Lat: "); Serial.print(targetPosition.lat);
    Serial.print(" Lon: "); Serial.print(targetPosition.lon);
    Serial.print(" Distance: "); Serial.print(targetPosition.distance);
    Serial.print(" Heading: "); Serial.print(trackerPosition.heading / 10);
    Serial.print(" Target Heading: "); Serial.print(targetPosition.heading / 10);
#ifdef MAVLINK
    Serial.print(" Target Sats: "); Serial.print(getSats());
#else
    Serial.println();
#endif
#endif
    HAS_FIX = false;
  }
#endif



#endif
  // refresh rate of compass is 75Hz -> 13.333ms to refresh the data
  // we update the heading every 14ms to get as many samples into the smooth array as possible
  if (millis() > time) {
    time = millis() + 14;
    trackerPosition.heading = getHeading();
    NEW_HEADING = true;
  }

  CURRENT_STATE = digitalRead(CALIB_BUTTON);
  if (CURRENT_STATE != PREVIOUS_STATE) {
    digitalWrite(LED_PIN, !CURRENT_STATE);
    //pin changed
    if (!CURRENT_STATE && calib_timer == 0) {
      calib_timer = millis();
    } else if (CURRENT_STATE && millis() - calib_timer < 4000) {
      //button not pressed long enough
      calib_timer = 0;
    } else if (CURRENT_STATE && millis() - calib_timer > 4000) {
      //start calibration routine if button pressed > 4s and released
      //cli();
#ifdef LCD_DISPLAY
      lcd.clear();
      #if LCD_SIZE_ROW == 4
        lcd.setCursor(0, 1);
      #else
        lcd.setCursor(0, 0);
      #endif
      //sprintf(lcd_str, "HDG:%03u AZI:%03u", 0, 0);
      lcd.print(" Calibration in ");
      #if LCD_SIZE_ROW == 4
        lcd.setCursor(0, 2);
      #else
        lcd.setCursor(0, 1);
      #endif
      lcd.print("   progress...  ");
#endif
      calibrate_compass();
      calib_timer = 0;
      //sei();
    }
    PREVIOUS_STATE = CURRENT_STATE;
  }
#ifndef SERVOTEST
#ifndef LOCAL_GPS
  //only needed if no local gps
  if (!digitalRead(HOME_BUTTON)) {
    //set home
#ifdef MFD
    //MFD protocol: set home must be pressed on driver!
#else
  setHome(&trackerPosition, &targetPosition);
#endif

  }
#else
  if (gpsSerial.available()) {
    uint8_t c = gpsSerial.read();
#ifdef DEBUG
    Serial.write(c);
#endif
    if (gps.encode(c)) {
      unsigned long fix_age;
      gps.get_position(&trackerPosition.lat, &trackerPosition.lon, &fix_age);
      if (fix_age == TinyGPS::GPS_INVALID_AGE){
        //TODO no fix
        if (gps.satellites() != TinyGPS::GPS_INVALID_SATELLITES){
          localSats = (uint8_t)gps.satellites();
        }
        HOME_SET = false;
      }else if (fix_age >5000){
        //TODO fix too old, maybe tracker lost fix
        if (gps.satellites() != TinyGPS::GPS_INVALID_SATELLITES){
          localSats = (uint8_t)gps.satellites();
        }
        
        //attention, if gps is lost, last position will remain. So if gps somehow loses signal and the pos is not nulled in these 5s then it will continue tracking with last pos.
      }else{
        //TODO valid data
        trackerPosition.lat = trackerPosition.lat / 10;
        trackerPosition.lon = trackerPosition.lon / 10;

        if (gps.altitude() != TinyGPS::GPS_INVALID_ALTITUDE) {
          trackerPosition.alt = int32_t(gps.altitude());
        }

        if (gps.satellites() != TinyGPS::GPS_INVALID_SATELLITES){
          localSats = (uint8_t)gps.satellites();
        }
        
        HOME_SET = true;
        //Serial.print("Lat: ");Serial.print(trackerPosition.lat);Serial.print(" Lon: ");Serial.print(trackerPosition.lon);Serial.print(" Alt: ");Serial.println(trackerPosition.alt);
      }
    }
  }
#endif

#ifdef MFD
  if (SETTING_HOME) {
    HOME_SET = true;
    SETTING_HOME = 0;
  }

  if (!HOME_SET && TEST_MODE && NEW_HEADING) {
    distance = getDistance();
    targetPosition.alt = getTargetAlt();
    targetPosition.heading = getAzimuth() * 10;

    getError();
    calculatePID();
    SET_PAN_SERVO_SPEED(PWMOutput);
    calcTilt();
    NEW_HEADING = false;
  }
  if (HOME_SET && HAS_FIX && NEW_HEADING && !TEST_MODE) {
    targetPosition.alt = getTargetAlt();
    targetPosition.heading = getAzimuth() * 10;
    distance = getDistance();

    getError();
    calculatePID();
    SET_PAN_SERVO_SPEED(PWMOutput);
    calcTilt();
    NEW_HEADING = false;
  }
#endif
#endif
#ifndef MFD
  //Only track if home ist set.
  if (HOME_SET) {
#if (START_TRACKING_DISTANCE > 0)
    //Only track if tracking process started.
    if (!TRACKING_STARTED) {
      //if plane is START_TRACKING_DISTANCE meter away from tracker start tracking process.
      if (targetPosition.distance >= uint16_t(START_TRACKING_DISTANCE)) {
        TRACKING_STARTED = true;
      }
    } else {
#endif
      // only update pan value if there is new data
      if ( NEW_HEADING ) {
        getError();       // Get position error
        calculatePID();   // Calculate the PID output from the error
        SET_PAN_SERVO_SPEED(PWMOutput);
        NEW_HEADING = false;
#ifndef SERVOTEST
        calcTilt();
#endif
#if (START_TRACKING_DISTANCE > 0)
      }
#endif
    }
  }
#endif
}

//Tilt angle alpha = atan(alt/dist)
void calcTilt() {
  uint16_t alpha = 0;
  //prevent division by 0
  if (targetPosition.distance == 0) {
    alpha = 90;
  }
  else {
    alpha = toDeg(atan(float(targetPosition.alt - trackerPosition.alt) / targetPosition.distance));
  }
  //just for current tests, later we will have negative tilt as well
  if (alpha < 0)
    alpha = 0;
  else if (alpha > 90)
    alpha = 90;
  SET_TILT_SERVO_SPEED(map(alpha, 0, 90, TILT_0, TILT_90));
}

void getError(void)
{
  // shift error values
  for (byte i = 0; i < 10; i++) {
    Error[i + 1] = Error[i];
  }

  int16_t delta = targetPosition.heading - trackerPosition.heading;
  if (delta > 1800) {
    delta -= 3600;
  }
  else if (delta < -1800) {
    delta += 3600;
  }
  // load new error into top array spot
  Error[0] = delta;
}

void calculatePID(void)
{
  // Calculate the PID
#ifdef SERVOTEST
  PID = Error[0] * p;     // start with proportional gain
  Accumulator += Error[0];  // accumulator is sum of errors
  if (Accumulator > 5000)
    Accumulator = 5000;
  if (Accumulator < -5000)
    Accumulator = -5000;
  PID += i * Accumulator; // add integral gain and error accumulation
  Dk = d * (Error[0] - Error[10]);
  PID += Dk; // differential gain comes next
  PID = PID >> Divider; // scale PID down with divider
  // limit the PID to the resolution we have for the PWM variable
  if (PID >= 500)
    PID = 500;
  if (PID <= -500)
    PID = -500;
  if (Error[0] > 10) {
    PWMOutput = PAN_0 + PID + MIN_PAN_SPEED;
  } else if (Error[0] < -10) {
    PWMOutput = PAN_0 + PID - MIN_PAN_SPEED;
  } else {
    PWMOutput = PAN_0;
  }
#else
  PID = Error[0] * P;     // start with proportional gain
  Accumulator += Error[0];  // accumulator is sum of errors
  if (Accumulator > 5000)
    Accumulator = 5000;
  if (Accumulator < -5000)
    Accumulator = -5000;
  PID += I * Accumulator; // add integral gain and error accumulation
  Dk = D * (Error[0] - Error[10]);
  PID += Dk; // differential gain comes next
  PID = PID >> Divider; // scale PID down with divider
  // limit the PID to the resolution we have for the PWM variable
  if (PID >= 500)
    PID = 500;
  if (PID <= -500)
    PID = -500;
  if (Error[0] > 10) {
    PWMOutput = PAN_0 + PID + MIN_PAN_SPEED;
  } else if (Error[0] < -10) {
    PWMOutput = PAN_0 + PID - MIN_PAN_SPEED;
  } else {
    PWMOutput = PAN_0;
  }
#endif
}

#ifdef LOCAL_GPS
void initGps() {
#ifdef MTK
  //set to 1Hz
  gpsSerial.println("$PMTK220,1000*1F");
  //delay for ack/nack as softserial cannot read and write at the same time
  delay(20);

  //only enable GGA sentences
  gpsSerial.print("$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
  //delay for ack/nack as softserial cannot read and write at the same time
  delay(20);
#endif
}
#endif

#ifdef BATTERYMONITORING
  void getBatterieVoltage() {
    int n = 0;
    uint16_t Bat_ADC = (uint16_t)analogRead(VOLTAGEDIVIDER); //Hole Wert
    uint32_t Bat_AVG = (uint32_t)Bat_ADC;
    for (n = 0; n < BATTERYMONITORING_AVERAGE; n++) {
      Bat_AVG += (uint32_t)Bat_ADC_Last[n];
    }
    Bat_AVG /= BATTERYMONITORING_AVERAGE + 1;
    for (n = 0; n < BATTERYMONITORING_AVERAGE - 1; n++) {
      Bat_ADC_Last[n] = Bat_ADC_Last[n + 1];
    }
    Bat_ADC_Last[BATTERYMONITORING_AVERAGE - 1] = Bat_ADC;
    Bat_Voltage = ((float)Bat_AVG / 1024.0) * BATTERYMONITORING_VREF / Bat_denominator * BATTERYMONITORING_CORRECTION;
    Serial.print("V: "); Serial.println(Bat_Voltage);
  }
#endif
