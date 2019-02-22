# AR2 Raspberry Pi Source Files
Contributed by Zach Allen

This code is modified from the software version `2.1.5` so you can drive the AR2 Robot with a Raspberry Pi.  You can use a keyboard, mouse, and monitor with the RPi, or configure it as an access point and connect to it remotely from a tablet or phone using VNC Viewer.

## Getting Started

Copy all the files onto your Raspberry Pi using the clone or download function in GitHub.

It is crucial that all of the files are located in  `/home/pi/Documents/AR2/`.

The paths in the source code are mapped to this directory.  If you would prefer the source files to be located in a different directory, feel free to update all of the paths in the `AR2.py` file.

### Prerequisites

Make sure you're using a complete version of Raspbian.  This has not been tested on other OS versions.  Find the latest Raspbian distribution at Raspberry Pi's [Download](https://www.raspberrypi.org/downloads/raspbian/) page.

### Installing

Once the files are on your Raspberry Pi, follow the links below to complete the set-up procedure.  All of these are optional, but the most-used functionality will be #3.

1. [Configure RPi](ref/CONFIGURE.md)
2. [Set Up Access Point](ref/AP-SETUP.md)
3. [Create Desktop Shortcut](ref/SHORTCUT.md)
4. [Set Program to Run on Start Up](ref/RUN-ON-BOOT.md)

```
Steps to be updated here
```

## Using the program

Plug the USB into the Raspberry Pi and the Arduino, give power to the Arduino and the AR2, and you will be all set to run the AR2 software.  Please see Chris's [documentation](https://www.anninrobotics.com/downloads) for using the software.

## Built With

* [Raspbian Stretch](https://www.raspberrypi.org/downloads/raspbian/) - The OS used

## Authors

* **Chris Annin** - *Initial work* - [Annin Robotics](https://www.anninrobotics.com/)
* **Zach Allen** - *Modifying software for RPi*

## Acknowledgments

* Huge kudos to Chris Annin for all the work he has done on the AR2, and for sharing it for free for others to learn and create!
