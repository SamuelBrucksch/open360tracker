Introduction
=====
This project is about an antenna tracker that is easy to build. It features a slipring that offers full 360° continuous rotation and support for lots of different telemetry protocols and flight controllers like FrSky, HoTT, Mavlink, MultiWii, Naza, ArduPilot, Arducopter.

More Information: http://fpv-community.de/showthread.php?49179-open360tracker-Der-Community-Antennentracker-Made-in-Germany

Details
=====
The slipring offers free 360° continuous rotation without the hassle of twisted cables. For this we use a robot servo that has no fixed endpoint. To measure the direction the antenna tracker points it antenna to we use a compass. This has many different advantages like no need for pointing the tracker to north or calibrate it somehow to the servo travel of the used servos.

The initial version will make use of the Frsky Protocol as there are so many different easy to build arduino projects that support all kinds of flight controls.

Heres you can find a list of what might be supported: [Protocols](https://github.com/SamuelBrucksch/open360tracker/wiki/Protocols)

Current state of the project
=====
FRSKY_X, FRSKY_D and MAVLINK seem to work:

[![ScreenShot](http://img.youtube.com/vi/F41oIQ15KQs/0.jpg)](https://www.youtube.com/watch?v=F41oIQ15KQs)

Click the pic to see the video. Its tested in the garden but not in flight yet.


For furter information, please have a look at our Wiki Howto's:

[Servos](https://github.com/SamuelBrucksch/open360tracker/wiki/Servos)

[Buttons](https://github.com/SamuelBrucksch/open360tracker/wiki/Buttons)

[Tracker GPS](https://github.com/SamuelBrucksch/open360tracker/wiki/Tracker-GPS)
