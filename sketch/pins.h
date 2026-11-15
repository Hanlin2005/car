// The only place Uno Q pin numbers are set. sketch/sketch.ino includes this file.
// For now Motor1 is the right wheel and Motor2 is the left wheel.
// To swap which physical motor is left, exchange the left and right values.

// Motor2, left wheel. Forward is pin 6, back is pin 5.
const int LEFT_IN1 = 6;
const int LEFT_IN2 = 5;
const int LEFT_ENC_A = 11;             // encoder A, the tick
const int LEFT_ENC_B = 12;             // encoder B, the direction
const bool INVERT_LEFT_ENCODER = false;

// Motor1, right wheel. Forward is pin 9, back is pin 10.
const int RIGHT_IN1 = 9;
const int RIGHT_IN2 = 10;
const int RIGHT_ENC_A = 4;
const int RIGHT_ENC_B = 3;
const bool INVERT_RIGHT_ENCODER = false;

// All three follow the light sensor. Headlights are both lamps on one pin.
const int REAR_LIGHT_1 = 7;
const int REAR_LIGHT_2 = 8;
const int HEADLIGHTS = 13;

// Lidar TX goes to Serial1 RX. A4/A5 are a different I2C bus; the sensors
// use the header pins labeled SDA and SCL, which Wire already owns.
const int LIDAR_RX = 0;
