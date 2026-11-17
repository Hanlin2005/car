# Autonomous Car (Arduino Uno Q)

The microcontroller in `sketch/sketch.ino` drives the motors, reads the
encoders, the lidar, the VL53L1X, and the BH1750. The Linux side runs the
camera, the lane logic, and the page. Pin numbers live in `sketch/pins.h`.

Open this folder in Arduino App Lab.

## What plugs in where

Change a pin in `sketch/pins.h`. That file is the one the sketch compiles.
For now Motor1 is the right wheel and Motor2 is the left wheel. Swapping the
left and right values in `pins.h` swaps which physical wheel is left.

The Uno Q header is the classic Uno layout. D0 is the pin closest to the
USB-C connector on the digital row. The VL53L1X and the BH1750 share the
pins labeled SDA and SCL. A4 and A5 are a different I2C bus.

| PCB | Uno Q |
| --- | --- |
| Motor1 forward (DRV8833 BIN1) | D9 (right wheel) |
| Motor1 back (DRV8833 BIN2) | D10 |
| Motor1 encoder A | D4 |
| Motor1 encoder B | D3 |
| Motor2 forward (DRV8833 AIN1) | D6 (left wheel) |
| Motor2 back (DRV8833 AIN2) | D5 |
| Motor2 encoder A | D11 |
| Motor2 encoder B | D12 |
| Rear light 1 | D7 |
| Rear light 2 | D8 |
| Headlights (both) | D13 |
| Time-of-flight SDA, SCL | header SDA, SCL |
| Light sensor SDA, SCL | header SDA, SCL |
| Lidar TX | D0 (RX) |

Power the sensors from 3.3V and tie their ground to a Uno Q GND pin. A lamp
pin only switches a MOSFET or the lamp module's enable input. The lamp
current stays on the battery side, and the battery negative ties to Uno GND.

Power the Uno Q from its USB-C supply. The motor battery only goes to the
motor driver. Tie the battery negative to a Uno Q GND pin as well.

The motor driver is a DRV8833. Its VM input takes 2.7 to 10.8 V, so a pack
of 8 alkaline AA or AAA cells (about 12 V fresh) is too much; 6 cells or a
2S Li-ion pack fits. nSLEEP (SLP or EEP on some boards) must be high or
neither wheel turns. Tie it to 3.3V if the board doesn't pull it up. The
left motor goes on AOUT1/AOUT2 and the right motor on BOUT1/BOUT2.

The speed feed-forward in `sketch/sketch.ino` was measured on the old MDD3A
driver. Run the duty-cycle test again with the DRV8833 and the battery you
use, then update `KF_LEFT`, `KF_RIGHT`, and `WEB_MAX_SPEED`.

The camera plugs into a MIPI-CSI connector on the UNO Media Carrier with
the 22-pin ribbon cable. The code reads it through libcamera, so run
`cam -l` on the Uno Q first: if it lists no camera, the code won't see one
either. Set `CAMERA_FLIP` and `CAMERA_HFOV_DEG` in `python/config.py` for
how the camera is mounted and which module it is. To go back to a USB
webcam, set `CAMERA_SOURCE = "usb"`.

If a wheel's distance goes negative while driving forward, set
`INVERT_LEFT_ENCODER` or `INVERT_RIGHT_ENCODER` to `true` in
`sketch/pins.h`.

## Run

In Arduino App Lab, open this folder and press Run. On a computer on the
same network, open `http://10.194.91.71:8080` (`CAR_IP` in
`python/config.py`). Manual driving is selected
at startup and the wheels stay still until a direction is held.
