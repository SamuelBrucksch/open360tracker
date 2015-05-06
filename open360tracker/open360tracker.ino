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
void getBatterie();
void checkAlarms();
void beep();

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

#include <SoftwareSerial.h>
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
TinyGPS gps;
void initGps();
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
      getBatterie();
    }
  #endif
  #ifdef BUZZER
    pinMode(BUZZER_PIN, OUTPUT);
  #endif
#ifdef LCD_DISPLAY
  lcd.begin(16, 2);
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.home();
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
    //switch screen every X seconds
    if (millis() % 10000 < 7000) {
      //headings, alt, distance, sats
      lcd.setCursor(0, 0);
#ifdef MFD
      sprintf(lcd_str, "H:%03u A:%03u", trackerPosition.heading / 10, targetPosition.heading / 10);

#else
      #ifdef BATTERYMONITORING
        sprintf(lcd_str, "H:%03u V%02u.%01u S:%02d", trackerPosition.heading / 10, (uint16_t)Bat_Voltage,(uint16_t)(Bat_Voltage*10)%10, getSats());
      #else
        sprintf(lcd_str, "H:%03u A:%03u S:%02d", trackerPosition.heading / 10, targetPosition.heading / 10, getSats());
      #endif
#endif
      lcd.print(lcd_str);
      lcd.setCursor(0, 1);
      sprintf(lcd_str, "A:%05d D:%05u ", targetPosition.alt, targetPosition.distance);
      lcd.print(lcd_str);
    } else {
      //lat, lon
      lcd.setCursor(0, 0);
      lcd.print("T LAT:");
      dtostrf(targetPosition.lat / 100000.0f, 10, 5, lcd_str);
      lcd.print(lcd_str);
      lcd.setCursor(0, 1);
      lcd.print("T LON:");
      dtostrf(targetPosition.lon / 100000.0f, 10, 5, lcd_str);
      lcd.print(lcd_str);
    }
    #ifdef BATTERYMONITORING
      getBatterie();
    #endif
    #ifdef BUZZER
      checkAlarms();
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
    Serial.print("Target alt: "); Serial.print(targetPosition.alt);
    Serial.print(" Target distance: "); Serial.print(targetPosition.distance);
    Serial.print(" Target heading: "); Serial.print(targetPosition.heading / 10);
    Serial.print(" Tracker heading: "); Serial.print(trackerPosition.heading / 10);
    Serial.print(" Target Sats: "); Serial.println(getSats());
#endif
#else
#ifdef DEBUG
    Serial.print("Target alt: "); Serial.println(targetPosition.alt);
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
    Serial.print(" Target Fix Type: "); Serial.println(getTargetFixType());
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
      lcd.setCursor(0, 0);
      //sprintf(lcd_str, "HDG:%03u AZI:%03u", 0, 0);
      lcd.print(" Calibration in ");
      lcd.setCursor(0, 1);
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
      gps.get_position(&trackerPosition.lat, &trackerPosition.lon);
      digitalWrite(LED_PIN, HIGH);
      if (gps.altitude() != TinyGPS::GPS_INVALID_ALTITUDE) {
        trackerPosition.alt = (int16_t)gps.altitude();
      }
    }
    else {
      digitalWrite(LED_PIN, LOW);
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
  void getBatterie() {
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

#ifdef BUZZER
  void checkAlarms() {
    /*byte names[] = {'c', 'd', 'e', 'f', 'g', 'a', 'b', 'C'};  
    int tones[] = {1915, 1700, 1519, 1432, 1275, 1136, 1014, 956};
    byte melody[] = "2d2a1f2c2d2a2d2c2f2d2a2c2d2a1f2c2d2a2a2g2p8p8p8p";
    int count = 0;
    int count2 = 0;
    int count3 = 0;
    int MAX_COUNT = 24;
    int statePin = LOW;
    analogWrite(BUZZER_PIN, 0);     
    for (count = 0; count < MAX_COUNT; count++) {
      statePin = !statePin;
      for (count3 = 0; count3 <= (melody[count*2] - 48) * 30; count3++) {
        for (count2=0;count2<8;count2++) {
          if (names[count2] == melody[count*2 + 1]) {       
            analogWrite(BUZZER_PIN,500);
            delayMicroseconds(tones[count2]);
            analogWrite(BUZZER_PIN, 0);
            delayMicroseconds(tones[count2]);
          } 
          if (melody[count*2 + 1] == 'p') {
            // make a pause of a certain size
            analogWrite(BUZZER_PIN, 0);
            delayMicroseconds(500);
          }
        }
      }
    }*/
    //sing(1);
    sing(1);
    //sing(2);
  }
  int setTone(int Tone_Value) {
    if (BUZZER_CLOCK) {
      analogWrite(BUZZER_PIN, Tone_Value);
    }
    else {
      digitalWrite(BUZZER_PIN, Tone_Value);
    }
  }
#endif

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
 
//Mario main theme melody
int melody[] = {
  NOTE_E7, NOTE_E7, 0, NOTE_E7,
  0, NOTE_C7, NOTE_E7, 0,
  NOTE_G7, 0, 0,  0,
  NOTE_G6, 0, 0, 0,
 
  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,
 
  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0,
 
  NOTE_C7, 0, 0, NOTE_G6,
  0, 0, NOTE_E6, 0,
  0, NOTE_A6, 0, NOTE_B6,
  0, NOTE_AS6, NOTE_A6, 0,
 
  NOTE_G6, NOTE_E7, NOTE_G7,
  NOTE_A7, 0, NOTE_F7, NOTE_G7,
  0, NOTE_E7, 0, NOTE_C7,
  NOTE_D7, NOTE_B6, 0, 0
};
//Mario main them tempo
int tempo[] = {
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
 
  9, 9, 9,
  12, 12, 12, 12,
  12, 12, 12, 12,
  12, 12, 12, 12,
};
//Underworld melody
int underworld_melody[] = {
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0, NOTE_DS4, NOTE_CS4, NOTE_D4,
  NOTE_CS4, NOTE_DS4,
  NOTE_DS4, NOTE_GS3,
  NOTE_G3, NOTE_CS4,
  NOTE_C4, NOTE_FS4, NOTE_F4, NOTE_E3, NOTE_AS4, NOTE_A4,
  NOTE_GS4, NOTE_DS4, NOTE_B3,
  NOTE_AS3, NOTE_A3, NOTE_GS3,
  0, 0, 0
};
//Underwolrd tempo
int underworld_tempo[] = {
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  3,
  12, 12, 12, 12,
  12, 12, 6,
  6, 18, 18, 18,
  6, 6,
  6, 6,
  6, 6,
  18, 18, 18, 18, 18, 18,
  10, 10, 10,
  10, 10, 10,
  3, 3, 3
};
int song = 0;
void sing(int s) {
  // iterate over the notes of the melody:
  song = s;
  if (song == 2) {
    int size = sizeof(underworld_melody) / sizeof(int);
    for (int thisNote = 0; thisNote < size; thisNote++) {
 
      // to calculate the note duration, take one second
      // divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000 / underworld_tempo[thisNote];
 
      buzz(BUZZER_PIN, underworld_melody[thisNote], noteDuration);
 
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
 
      // stop the tone playing:
      buzz(BUZZER_PIN, 0, noteDuration);
 
    }
 
  } else {
     int size = sizeof(melody) / sizeof(int);
    for (int thisNote = 0; thisNote < size; thisNote++) {
 
      // to calculate the note duration, take one second
      // divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000 / tempo[thisNote];
 
      buzz(BUZZER_PIN, melody[thisNote], noteDuration);
 
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
 
      // stop the tone playing:
      buzz(BUZZER_PIN, 0, noteDuration);
 
    }
  }
}

void buzz(int targetPin, long frequency, long length) {
  long delayValue = 1000000 / frequency / 2; // calculate the delay value between transitions
  //// 1 second's worth of microseconds, divided by the frequency, then split in half since
  //// there are two phases to each cycle
  long numCycles = frequency * length / 1000; // calculate the number of cycles for proper timing
  //// multiply frequency, which is really cycles per second, by the number of seconds to
  //// get the total number of cycles to produce
  for (long i = 0; i < numCycles; i++) { // for the calculated length of time...
    setTone(HIGH); // write the buzzer pin high to push out the diaphram
    delayMicroseconds(delayValue); // wait for the calculated delay value
    setTone(LOW); // write the buzzer pin low to pull back the diaphram
    delayMicroseconds(delayValue); // wait again or the calculated delay value
  }
}
