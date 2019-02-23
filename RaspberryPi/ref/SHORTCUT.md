# Create Desktop shortcut
[Return to README.md](../README.md)

This guide will show you how to create a Desktop shortcut from which you can run the program.

### Create a Desktop Shortcut

Open a terminal window and point the directory to the Desktop
```
cd /home/pi/Desktop/
```

Create a new text file on the Desktop.  Since it does not yet exist, a new file will be automatically created.
```
sudo nano AR2.desktop
```

Type or copy/paste the following into the text file.  This will configure the details necessary to make it a desktop shortcut with an icon.
```
[Desktop Entry]
Name=AR2
Comment=AR2 Robot Control Program
Icon=/home/pi/Documents/AR2/AR2.ico
Exec=python /home/pi/Documents/AR2/AR2.py
Type=Application
Terminal=false
Categories=None;
```
Then, save and exit
```
Ctrl+x
y
Enter
```

Reboot your RPi
```
sudo reboot
```

Now, the shortcut should appear on your desktop!

If you want the program to start up when you turn on the Raspberry Pi, check out the [run-on-boot documentation](./RUN-ON-BOOT.md)

[Return to README.md](../README.md)
