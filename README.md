Introduction
=====
This project is about an antenna tracker that is easy to build. It features a slipring that offers full 360° continuous rotation and support for lots of different telemetry protocols and flight controllers like FrSky, HoTT, Mavlink, MultiWii, Naza, ArduPilot, Arducopter.

More Information: http://fpv-community.de/showthread.php?49179-open360tracker-Der-Community-Antennentracker-Made-in-Germany

Details
=====
The slipring offers free 360° continuous rotation without the hassle of twisted cables. For this we use a robot servo that has no fixed endpoint. To measure the direction the antenna tracker points it antenna to we use a compass. This has many different advantages like no need for pointing the tracker to north or calibrate it somehow to the servo travel of the used servos.

The initial version will make use of the Frsky Protocol as there are so many different easy to build arduino projects that support all kinds of flight controls.

Heres a list of what might be supported at a later stage:

    Frsky
        normal telemetry data (Frsky GPS) 1Hz only
        DIY Arduino Frsky GPS (5Hz or more)
        MultiWii2Frsky
        APM2Frsky
        DIY Arduino Naza2Frsky

    HoTT
        normal telemetry data (HoTT GPS)
        DIY Arduino HoTT GPS
        DIY Arduino Naza2HoTT
        APM2HoTT
        MultiWii2HoTT (not sure here) 

    433/868/915Mhz 3DR radio
        Mavlink
        Multiwii
        direct GPS data (NMEA)
        any other protocol that is open 

Current state of the code
=====
FRSKY_X and FRSKY_D seem to work:
https://www.youtube.com/watch?v=F41oIQ15KQs
Its tested in the garden but not in flight yet.

Mavlink is implemented and tested at home, but not in flight yet.

Servos
=====
Pan servo needs to be connected to pin D9

Tilt Servo needs to be connected to pin D10 

Buttons
=====
There is a HOME button and a CALIBRATION button. The HOME button is to set the home position of the tracker when no local GPS is present. The CALIBRATION button is to calibrate the compass to your local magnetic field. Press CALIBRATION button for 4s and the tracker will start to rotate to read your magnetic field.

The HOME button needs to be connected to D5 and GND

The CALIBRATION button needs to be connected to D6 and GND 

Tracker GPS
=====
You can connect a GPS module to the tracker. It will set the current position of the tracker so no need to manually set HOME. With a GPS attached to the tracker you can even move the tracker around (for example on a boat or car) and it still points to the aircraft. You can use different GPS modules to work with the tracker. However it needs to be configured to a maximum (still needs to be investigated) of 38400baud with a refresh rate of 1Hz and only GGA sentences enabled. This is due to a restriction of the used library to attach another serial device.

For Mediatek modules these settings will be set automatically, so no need to configure anything here.

TX of GPS needs to be connected to D8

RX of GPS needs to be connected to D7 (important for auto configuration of GPS) 
