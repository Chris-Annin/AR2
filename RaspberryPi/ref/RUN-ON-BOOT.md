# Set up Raspberry Pi to run AR2 Program on Start Up
[Return to README.md](../README.md)

This guide will show you how to configure the RPI to run the AR2 program automatically when the RPi is booted.

First, open terminal.  The working directory is not important at this time.

Edit `/etc/profile` and add a command to run the program.
```
sudo nano /etc/profile
```
Scroll all the way to the bottom and add the following
```
sudo python /home/pi/Documents/AR2/RaspberryPi/AR2.py &
```

It is **critical** you have the ampersand (&) at the end, otherwise your Raspberry Pi will get stuck in a loop when booting.

Once you have added the command, save and exit, and reboot.
```
Ctrl+x
y
Enter
```
```
sudo reboot
```

Once the RPi has rebooted, the AR2 software screen should automatically run. Unless you close the program and need to re-open it, you shouldn't need to use the desktop shortcut.  However it's there anyway!

[Continue](./AP-SETUP.md) to learn how to configure your RPi for remote connection via VNC Viewer.

[Return to README.md](../README.md)
