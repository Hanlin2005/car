# Autonomous Car (Arduino Uno Q)

Same car program as the Raspberry Pi copy, running on an Arduino Uno Q.
The microcontroller in `sketch/sketch.ino` drives the motors, reads the
encoders, the lidar, the VL53L0X, and the BH1750. The Linux side runs the
camera, the lane logic, and the page. Pin numbers live in `sketch/pins.h`.

The Pi version stays in the `autonomous-car` folder. Open **this** folder
in Arduino App Lab.

## What plugs in where

Change a pin in `sketch/pins.h`. That file is the one the sketch compiles.
For now Motor1 is the right wheel and Motor2 is the left wheel. Swapping the
left and right values in `pins.h` swaps which physical wheel is left.

The Uno Q header is the classic Uno layout. D0 is the pin closest to the
USB-C connector on the digital row. The VL53L0X and the BH1750 share the
pins labeled SDA and SCL. A4 and A5 are a different I2C bus.

| PCB | Uno Q |
| --- | --- |
| Motor1 forward | D9 (right wheel) |
| Motor1 back | D10 |
| Motor1 encoder A | D4 |
| Motor1 encoder B | D3 |
| Motor2 forward | D6 (left wheel) |
| Motor2 back | D5 |
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

A USB webcam goes in the Uno Q's USB port.

If a wheel's distance goes negative while driving forward, set
`INVERT_LEFT_ENCODER` or `INVERT_RIGHT_ENCODER` to `true` in
`sketch/pins.h`.

## Run

In Arduino App Lab, open this folder and press Run. On a computer on the
same network, open `http://<the-uno-q>:8080`. Manual driving is selected
at startup and the wheels stay still until a direction is held.
