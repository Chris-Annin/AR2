# Set Up and Configure your Raspberry Pi
[Return to README.md](../README.md)

## Download Raspbian OS

If you are using a "fresh" Raspberry Pi, you'll need to get an OS on a Micro SD card to use the RPI.  Go to Raspberry Pi's [download](https://www.raspberrypi.org/downloads/raspbian/) page to get the latest Raspbian distribution for Raspberry Pi.  At the time of this writing, the OS is "Raspbian Stretch with desktop and recommended software".  This page also has instructions for writing the OS image to a Micro SD card as well.

## Boot up the Raspberry Pi

Once the OS image is copied onto the Micro SD card, insert the SD card into the RPi, connect a monitor, keyboard and mouse, and plug in 5V power with a micro USB.  The RPi will turn on automatically and begin installing the OS. Once booted, follow the on-screen set-up instructions.

### Configure Settings

Open the applications ("Start") menu and select `Preferences` and then `Raspberry Pi Configuration`.

On the `Interfaces` tab, make sure the following are marked as `Enabled`
* SSH
* VNC
* Serial Port
* Serial Console

If you will be using a tablet for remote control of the RPi, you can change the screen resolution accordingly on the `System` tab.  Once you change the resolution, you will be required to restart the RPi.

## Clone GitHub Repository

Open terminal and navigate to the `Documents` folder.
```
cd /home/pi/Documents/
```

Type the following command to clone the repository
```
git clone https://github.com/Chris-Annin/AR2.git
```

Once it has finished downloading, navigate down into the `RaspberryPi` folder
```
cd /AR2/RaspberryPi
```

The paths in the source code are mapped to this directory.  If you would prefer the source files to be located in a different directory, all of the paths in the `AR2.py` file must be updated. If anyone knows of a better way to use the relative path, please let me know and I'll update the files and this guide.

If you want to check that the program run, type the following command
```
python AR2.py
```

[Continue](./SHORTCUT.md) to create a desktop shortcut to run the program.

[Return to README.md](../README.md)
