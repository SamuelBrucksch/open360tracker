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
#include <TinyGPS.h>
void calcTilt();
void getError();
void calculatePID();

unsigned long time;
unsigned long calib_timer;

//PID stuff
long Error[11];
long Accumulator;
long PID;
uint8_t Divider = 15;
int PWMOutput;
long Dk;

//distance in meter
uint16_t distance;
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

#ifndef MFD
  uint16_t getTargetHeading(geoCoordinate_t *a, geoCoordinate_t *b);
#endif

#ifdef LCD_DISPLAY
  //download from https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
  #include <LiquidCrystal_I2C.h>
  LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7);
  char lcd_str[24];
  long lcd_time;
#endif

void setup()
{
  #ifdef LCD_DISPLAY
    lcd.begin(16,2);
    lcd.setBacklightPin(3,POSITIVE);
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
  HOME_SET = false;
  SETTING_HOME = false;
  PREVIOUS_STATE = true;
  TRACKING_STARTED = false;
  CURRENT_STATE = true;
  NEW_HEADING = false;
  TEST_MODE = false;

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
    lcd.setCursor(0,0);
    sprintf(lcd_str, "HDG:%03u AZI:%03u", 0, 0);
    lcd.print(lcd_str);
    lcd.setCursor(0,1);
    sprintf(lcd_str, "A:%05d D:%05u", 0, 0);
    lcd.print(lcd_str);
    lcd_time = millis();
  #endif

  time = millis();
  
  #ifdef DEBUG
    Serial.println("Setup finished");
  #endif
}

void loop()
{
  //TODO change to telemetry serial port
  if (Serial.available() > 1)
  {
    uint8_t c = Serial.read();

    encodeTargetData(c);
    digitalWrite(LED_PIN, HIGH);
  }else{
    digitalWrite(LED_PIN, LOW);  
  }
  
  #ifdef LCD_DISPLAY
  if (millis() > lcd_time){
    //switch screen every X seconds
    if (millis() % 10000 < 7000){
      //headings, alt, distance, sats
      lcd.setCursor(0,0);
      #ifndef MFD
        sprintf(lcd_str, "H:%03u A:%03u S:%02d", trackerPosition.heading/10, targetPosition.heading/10, getSats());
      #else
        sprintf(lcd_str, "H:%03u A:%03u", trackerPosition.heading/10, targetPosition.heading/10);
      #endif
      lcd.print(lcd_str);
      lcd.setCursor(0,1);
      sprintf(lcd_str, "A:%05d D:%05u", targetPosition.alt, distance);
      lcd.print(lcd_str);
    }else{
      //lat, lon
      lcd.setCursor(0,0);
      lcd.print("T LAT:");
      dtostrf(targetPosition.lat/100000.0f, 10, 5, lcd_str);
      lcd.print(lcd_str);
      lcd.setCursor(0,1);
      lcd.print("T LON:");
      dtostrf(targetPosition.lon/100000.0f, 10, 5, lcd_str);
      lcd.print(lcd_str);  
    }
    lcd_time = millis() + 200;
  }
  #endif

  if (HAS_ALT){
    targetPosition.alt = getTargetAlt();
    
    // mfd has all the data at once, so we do not have to wait for valid lat/lon
    #ifdef MFD
      distance = getDistance();
      targetPosition.heading = getAzimuth() * 10;
      #ifdef DEBUG
        Serial.print("Target alt: ");Serial.print(targetPosition.alt);
        Serial.print(" Target distance: ");Serial.print(distance);
        Serial.print(" Target heading: ");Serial.print(targetPosition.heading/10);
        Serial.print(" Tracker heading: ");Serial.println(trackerPosition.heading/10);
      #endif
    #else
      #ifdef DEBUG
        Serial.print("Target alt: ");Serial.println(targetPosition.alt);
      #endif
    #endif
    
    HAS_ALT = false;
  }
  
  #ifndef MFD
    //only calculate distance and heading when we have valid telemetry data
    if (HAS_FIX){
      targetPosition.lat = getTargetLat();
      targetPosition.lon = getTargetLon();    
      
      // calculate distance without haversine. We need this for the slope triangle to get the correct pan value
      distance = sqrt(sq(trackerPosition.lat - targetPosition.lat) + sq(trackerPosition.lon - targetPosition.lon));  
      //targetPosition.heading = getTargetHeading(&trackerPosition, &targetPosition);
      targetPosition.heading = TinyGPS::course_to(trackerPosition.lat/100000.0f, trackerPosition.lon/100000.0f, targetPosition.lat/100000.0f, targetPosition.lon/100000.0f)*10.0f;

      #ifdef DEBUG
        Serial.print("Lat: "); Serial.print(targetPosition.lat); 
        Serial.print(" Lon: "); Serial.print(targetPosition.lon);
        Serial.print(" Distance: "); Serial.print(distance);
        Serial.print(" Heading: "); Serial.print(trackerPosition.heading/10);
        Serial.print(" Target Heading: "); Serial.println(targetPosition.heading/10);
      #endif
      HAS_FIX = false;
    }
  #endif
  
  // refresh rate of compass is 75Hz -> 13.333ms to refresh the data
  // we update the heading every 14ms to get as many samples into the smooth array as possible
  if (millis() > time){
    time = millis() + 14;
    trackerPosition.heading = getHeading();
    NEW_HEADING = true;
  }
  
  CURRENT_STATE = digitalRead(CALIB_BUTTON);
  if (CURRENT_STATE != PREVIOUS_STATE){
    digitalWrite(LED_PIN, !CURRENT_STATE);
     //pin changed
     if (!CURRENT_STATE && calib_timer == 0){
        calib_timer = millis();
     } else if (CURRENT_STATE && millis() - calib_timer < 4000){
       //button not pressed long enough
       calib_timer = 0; 
     } else if (CURRENT_STATE && millis() - calib_timer > 4000){
       //start calibration routine if button pressed > 4s and released
       //cli();
       lcd.clear();
       lcd.setCursor(0,0);
       //sprintf(lcd_str, "HDG:%03u AZI:%03u", 0, 0);
       lcd.print("Calibration in");
       lcd.setCursor(0,1);
       lcd.print("progress...");
       calibrate_compass();
       calib_timer = 0; 
       //sei();
     }
     PREVIOUS_STATE = CURRENT_STATE;
  }
  
  #ifndef LOCAL_GPS
    //only needed if no local gps
    if (!digitalRead(HOME_BUTTON)){
      //set home
      #ifndef MFD
        trackerPosition.alt = targetPosition.alt;
        trackerPosition.lat = targetPosition.lat;
        trackerPosition.lon = targetPosition.lon;
        HOME_SET = true;
      #else
      //MFD protocol: set home must be pressed on driver!
      #endif
      
    }
  #else
    if (gpsSerial.available()){
      uint8_t c = gpsSerial.read();
      #ifdef DEBUG
        Serial.write(c);
      #endif
      if (gps.encode(c)){
        gps.get_position(&trackerPosition.lat,&trackerPosition.lon);
        digitalWrite(LED_PIN, HIGH);
        if(gps.altitude() != TinyGPS::GPS_INVALID_ALTITUDE) {
          trackerPosition.alt = (int16_t)gps.altitude();
        }
      }
      else{
        digitalWrite(LED_PIN, LOW); 
      }
    }
  #endif
  
  #ifdef MFD
    if (SETTING_HOME){
       HOME_SET = true;
       SETTING_HOME = 0;
    }
    
    if (!HOME_SET && TEST_MODE && NEW_HEADING){
      distance = getDistance();
      targetPosition.alt = getTargetAlt();
      targetPosition.heading = getAzimuth()*10;
        
      getError();
      calculatePID();
      SET_PAN_SERVO_SPEED(PWMOutput);
      calcTilt(); 
      NEW_HEADING = false;
    } 
    if (HOME_SET && HAS_FIX && NEW_HEADING && !TEST_MODE){
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
  #ifndef MFD
    #if (START_TRACKING_DISTANCE > 0)
      //Only track if tracking process started.
      if(!TRACKING_STARTED) {
        //if plane is START_TRACKING_DISTANCE meter away from tracker start tracking process.
        if (START_TRACKING_DISTANCE <= distance && HOME_SET){
          TRACKING_STARTED = true;
        }
      } else {
    #else
      //Only track if home ist set.
      if (HOME_SET){
    #endif
        digitalWrite(LED_PIN, HIGH);
        // only update pan value if there is new data
        if( NEW_HEADING ) {
          getError();       // Get position error
          calculatePID();   // Calculate the PID output from the error
          SET_PAN_SERVO_SPEED(PWMOutput);
          NEW_HEADING = false;
          calcTilt(); 
        }
      } 
    #endif
}

//Tilt angle alpha = atan(alt/dist) 
void calcTilt(){
  uint16_t alpha = 0;
  //prevent division by 0
  if (distance == 0){
    alpha = 90;
  }
  else {
    alpha = toDeg(atan((targetPosition.alt - trackerPosition.alt)/distance));
  }
  //just for current tests, later we will have negative tilt as well
  if (alpha < 0)
    alpha = 0;
  
  SET_TILT_SERVO_SPEED(map(alpha, 0, 90, TILT_0, TILT_90));
}

void getError(void)
{
  // shift error values
  for(byte i = 0; i < 10; i++){
    Error[i+1] = Error[i];
  }

  int16_t delta = targetPosition.heading - trackerPosition.heading;
  if (delta > 1800){
    delta -= 3600; 
  } 
  else if (delta < -1800){
    delta += 3600;
  }  
  // load new error into top array spot  
  Error[0] = delta;
}

void calculatePID(void)
{
  // Calculate the PID  
  PID = Error[0] * P;     // start with proportional gain
  Accumulator += Error[0];  // accumulator is sum of errors
  if(Accumulator>5000)
    Accumulator=5000;
   if(Accumulator<-5000)
    Accumulator=-5000; 
  PID += I * Accumulator; // add integral gain and error accumulation
  Dk=D * (Error[0]-Error[10]);
  PID += Dk; // differential gain comes next
  PID = PID >> Divider; // scale PID down with divider
  // limit the PID to the resolution we have for the PWM variable
  if(PID >= 500)
    PID = 500;
  if(PID <= -500)
    PID = -500;
  if (Error[0] > 10){
    PWMOutput = PAN_0+PID;
  }else if(Error[0] < -10){
    PWMOutput = PAN_0+PID;
  } else {
    PWMOutput = PAN_0;
  }
}

#ifdef LOCAL_GPS
void initGps(){
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
