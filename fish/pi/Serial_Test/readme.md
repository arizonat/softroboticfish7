# Serial Logging Test (mbed -> pi)

To test serial connection between mbed and pi, do the following:

1. If on Ubuntu MATE 18.04 (Pi image), ensure that UART is enabled (either use sudo raspi-config -> Interfacing options or uncomment "enable_UART = 1" in /boot/config.txt)
2. If on Ubuntu MATE 18.04, disable serial console logging (sudo raspi-config -> Interfacing options) and DELETE any text within /boot/cmdline.txt that refers to said console and the serial port /dev/serial0 ("console=152000,/dev/serial0"). When you see this in cmdline.txt it means the Getty service is interrupting the serial port without your notice.
3. Ensure mbed and pi are properly connected (RX, TX, common GND). See schematic for more details.
4. Ensure mbed is flashed with code that writes to the serial port ( ex: serial->printf("Test\r\n"); )
5. Run log_serial_test.py, enter .txt file name when prompted (first create a new .txt file in logs/). If write statements on mbed are terminated with \r\n, they will be printed to the terminal as they are read. 
6. Once time has passed, Ctrl+C to kill program. Check logs/<filename.txt> for logged serial messages.

# Serial Control Test (pi -> mbed)


To test serial connection between pi and mbed, do the following:

1. If on Ubuntu MATE 18.04 (Pi image), ensure that UART is enabled (either use sudo raspi-config -> Interfacing options or uncomment "enable_UART = 1" in /boot/config.txt)
2. If on Ubuntu MATE 18.04, disable serial console logging (sudo raspi-config -> Interfacing options) and DELETE any text within /boot/cmdline.txt that refers to said console and the serial port /dev/serial0 ("console=152000,/dev/serial0"). When you see this in cmdline.txt it means the Getty service is interrupting the serial port without your notice.
3. Ensure mbed and pi are properly connected (RX, TX, common GND). See schematic for more details.
4. Ensure mbed is flashed with code that writes to the serial port ( ex: serial->printf("Test\r\n"); )
5. Open mbed serial terminal on MCUXpresso (or other) to listen to incoming messages from pi. Ensure mbed is plugged in via USB to machine using the serial terminal.
5. Run write_serial_test.py and observe the serial terminal for incoming messages.
