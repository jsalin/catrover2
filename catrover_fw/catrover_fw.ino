//
// Cat Rover 2 firmware (VC0706 camera version)
// Copyright (C) 2016 Jussi Salin <salinjus@gmail.com>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//
// Defined variables:
// ENA, ENB, MOT1-4 are pins for LM387 motor controller
// *_BRIGHTNESS values are for LED voltage control (0-255) = (0-5V)
// HORN_VOLTAGE is the same for the buzzer
// CAM is pin for enabling camera with a relay (reverse logic)
// LIGHT is pin for enabling a flashlight style LED
// LASER is pin for enabling a laser diode that attracts cats
// HORN is pin for a buzzer
//
// Connect by bluetooth module on ULTRASONIC pins and use following commands:
// amount <n>        Next movement will be this much in ms (time)
// left              Turn left
// right             Turn right
// forward           Go forward
// backward          Go backward
// camera <on/off>   Turn camera on or off to save battery
// light <on/off>    Turn light on or off
// laser <on/off>    Turn laser on or off
// photo             Take and send a photo with OV528 camera
//
// Connect camera relay board control pin to LINEFOLLOWER 1
// Connect light LED(s) + to LINEFOLLOWER 2
// Connect laser diode + to LINEFOLLOWER 3
// Connect buzzer's + to LINEFOLLOWER 4
// Connect serial camera's TX to INFRARED S
// Connect serial camera's RX to LINEFOLLOWER 1
//
// This Arduino project requires LowPower library available on GitHub.
// Alternatively you can undefine USE_POWERDOWN.
//
// This project also supports Adafruit_VC0706 serial camera library
// also available on GitHub. Can be disabled by undefining
// USE_SERIAL_CAMERA.

// Major compile time flags
//#define USE_POWERDOWN 1     // Use power saving code (does not work with PSU_WAKE)
#define DEBUG 1               // You need extra UART or SoftwareSerial for printing debug messages, if enabled
//#define LOW_FREQUENCY 1     // Lower the clock divider to save power (limits maximum software baud rates)
#define USE_PSU_WAKE 1        // Create button presses to keep USB battery pack turned on
#define USE_SERIAL_CAMERA 1   // Use Adafruit VC0706 serial camera

// Include needed headers
#include <SoftwareSerial.h>
#ifdef USE_POWERDOWN
#include <LowPower.h>
#endif
#ifdef USE_SERIAL_CAMERA
#include <Adafruit_VC0706.h>                                                                                                                      
#include <SPI.h>
#endif

// Analog values for different speeds in turning and moving situations
#define LIGHT_BRIGHTNESS 153
#define LASER_BRIGHTNESS 255
#define HORN_VOLTAGE 130

// Motor(s) on first side
#define ENA   10
#define MOT1  8
#define MOT2  7

// Motor(s) on the other side
#define ENB   5
#define MOT3  9
#define MOT4  6

// Other outputs
#define CAM   A0
#define LIGHT A1
#define LASER A2
bool camera_state = false;
bool light_state = false;

// "Push" button of a USB battery pack so it does not turn itself off
#define PSU_WAKE A3                   // Pin that is parallel on the switch of the batter pack
#define PSU_WAKE_VOLTAGE 153          // 3.0 volts
#define PSU_WAKE_INTERVAL 60*60*1000  // ms between the button presses
#define PSU_WAKE_DURATION 100         // ms to keep the button pressed
unsigned long last_psu_wake = 0;      // ms since last press (begin from 0)

// Global variables
#define SERIAL_CLASS SoftwareSerial   // NewSoftSerial or SoftwareSerial
String cmd = "";                      // Collect received characters here to finally have a full command
int amount = 100;                     // How long duration to move by default
int start_speed = 63;                 // Default starting speed of movement (0-255)
int move_speed = 127;                 // Default moving speed after acceleration (0-255)
SERIAL_CLASS mySerial(4, 3);          // RX, TX pins for second serial port (bluetooth)
SERIAL_CLASS mySerial2(11, A0);       // RX, TX pins for third serial port (serial camera)

// Serial ports
#define CAM_SERIAL mySerial2    // Serial port for camera module
#define BT_SERIAL mySerial      // Serial port for HC-05 bluetooth module
#define DEBUG_SERIAL Serial     // Serial port for debug messages, if debugging is enabled
#define CAM_BPS 38400           // Serial camera bps (some modules have 9600 default, some 38400, 115200 always maximum if set)
#define BT_BPS 115200           // Same as set in BT module. Can be very low, but use CAM_BPS or higher if using serial camera.
#define DEBUG_BPS 115200        // Can be anything your terminal supports

// Serial camera
#ifdef USE_SERIAL_CAMERA
Adafruit_VC0706 cam = Adafruit_VC0706(&CAM_SERIAL);
#endif

/**
 * Debug message printing code
 */
#ifdef DEBUG
void debug(String msg)
{
  DEBUG_SERIAL.println(msg);
}
#else
void debug(String msg)
{
}
#endif

/*
 * Setup code, ran at start up of Arduino
 */
void setup()
{
  // Start serial connection for debugging information
#ifdef DEBUG
  DEBUG_SERIAL.begin(DEBUG_BPS);
#endif
  
  // Some power savings
#ifdef LOW_FREQUENCY
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV128);
#endif

  // Set pin modes and start values
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(MOT1, OUTPUT);
  pinMode(MOT2, OUTPUT);
  pinMode(MOT3, OUTPUT);
  pinMode(MOT4, OUTPUT);
  pinMode(CAM, OUTPUT);
  camera_off();
  pinMode(LIGHT, OUTPUT);
  light_off();
  pinMode(LASER, OUTPUT);
  laser_off();
#ifdef USE_PSU_WAKE
  pinMode(PSU_WAKE, OUTPUT);
  digitalWrite(PSU_WAKE, LOW);
#endif

  // Start bluetooth serial connection
  BT_SERIAL.begin(BT_BPS);

  debug("Cat Rover 2.1 Arduino firmware booted up.");

#ifdef USE_SERIAL_CAMERA
  if (cam.begin(CAM_BPS))
  {
    debug("Camera found");
  }
  else
  {
    debug("Camera not found!");
  }
#endif
}

#ifdef USE_POWERDOWN
void wakeUp()
{
  // Don't have to do anything
}
#endif

/**
 * Arduino main loop function. Receive serial characters,
 * perform power saving and keep the power battery turned on.
 */
void loop()
{
  // call command() when cmd contains a new fully received line
  char chr = 0;
  if (BT_SERIAL.available() > 0) chr = BT_SERIAL.read();
  else if (DEBUG_SERIAL.available() > 0) chr = DEBUG_SERIAL.read();
  if (chr > 0)
  {
#ifdef ECHO
    BT_SERIAL.print(chr);
    DEBUG_SERIAL.print(chr);
#endif
    if ((chr == '\n') || (chr == '\r'))
    {
      command();
      cmd = "";
    }
    else
    {
      cmd.concat(chr);
    }
  }

#ifdef USE_POWERDOWN
  bool received = false;
  // Wait ten seconds for a character actively
  for (int i=0; i<10000; i++)
  {
    if (BT_SERIAL.available() > 0)
    {
      received = true;
      break;
    }
    delay(1);
  }
  // No characters - go to power down mode
  if (received == false)
  {
    attachInterrupt(0, wakeUp, LOW);
    LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
    detachInterrupt(0);
  }
#endif

#ifdef USE_PSU_WAKE
  // "Press" button of a USB battery pack to keep it turned on
  if  (millis()-last_psu_wake >= PSU_WAKE_INTERVAL)
  {
    debug("Performing PSU wakeup");
    digitalWrite(PSU_WAKE, HIGH);
    delay(PSU_WAKE_DURATION);
    digitalWrite(PSU_WAKE, LOW);
    last_psu_wake = millis();
  }
#endif
}

/**
 * Perform action according to a fully received command in "cmd" variable
 */
void command()
{
  cmd.toLowerCase();
  cmd.trim();
  if (cmd == "") return;
  
  debug("Received command: " + cmd);

  // Handle command of setting how much to move when moving
  if (cmd.indexOf("amount") == 0)
  {
    sscanf(cmd.c_str(), "amount %i", &amount);
  }
  else if (cmd.indexOf("start_speed") == 0)
  {
    sscanf(cmd.c_str(), "start_speed %i", &start_speed);
  }
  else if (cmd.indexOf("move_speed") == 0)
  {
    sscanf(cmd.c_str(), "move_speed %i", &move_speed);
  }

  // Handle movement commands
  if (cmd == "left")
  {
    left();
    smooth_speed();
    brake();
  }
  else if (cmd == "right")
  {
    right();
    smooth_speed();
    brake();
  }
  else if (cmd == "forward")
  {
    forward();
    smooth_speed();
    brake();
  }
  else if (cmd == "backward")
  {
    backward();
    smooth_speed();
    brake();
  }

  // Handle other commands
  if (cmd == "camera on")
  {
    camera_on();
  }
  else if (cmd == "camera off")
  {
    camera_off();
  }
  else if (cmd == "light on")
  {
    light_on();
  }
  else if (cmd == "light off")
  {
    light_off();
  }
  else if (cmd == "laser on")
  {
    laser_on();
  }
  else if (cmd == "laser off")
  {
    laser_off();
  }
  else if (cmd == "photo")
  {
    photo();
  }
  // For testing connection
  else if (cmd == "ping")
  {
    BT_SERIAL.println("pong");
  }
}

void right()
{
  digitalWrite(MOT1, HIGH);
  digitalWrite(MOT2, LOW);
  digitalWrite(MOT3, HIGH);
  digitalWrite(MOT4, LOW);
  debug("Turning right");
}

void left() {
  digitalWrite(MOT1, LOW);
  digitalWrite(MOT2, HIGH);
  digitalWrite(MOT3, LOW);
  digitalWrite(MOT4, HIGH);
  debug("Turning left");
}

void brake()
{
  digitalWrite(MOT1, LOW);
  digitalWrite(MOT2, LOW);
  digitalWrite(MOT3, LOW);
  digitalWrite(MOT4, LOW);
  debug("Stopping movement");
}

void backward()
{
  digitalWrite(MOT1, LOW);
  digitalWrite(MOT2, HIGH);
  digitalWrite(MOT3, HIGH);
  digitalWrite(MOT4, LOW);
  debug("Moving backward");
}

void forward()
{
  digitalWrite(MOT1, HIGH);
  digitalWrite(MOT2, LOW);
  digitalWrite(MOT3, LOW);
  digitalWrite(MOT4, HIGH);
  debug("Moving forward");
}

/**
 * Accelerated speed control code (set direction before calling)
 */
void smooth_speed()
{
  float spd = start_speed;
  float incr = (float)(move_speed - start_speed) / (float)(amount/2);
  for (int i=0; i<amount/2; i++)
  {
    analogWrite(ENA, (int)spd);
    analogWrite(ENB, (int)spd);
    spd += incr;
    delay(1);
  }
  for (int i=amount/2; i<amount; i++)
  {
    analogWrite(ENA, (int)spd);
    analogWrite(ENB, (int)spd);
    spd -= incr;
    delay(1);
  }
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

void camera_on()
{
  digitalWrite(CAM, LOW);
  camera_state = true;
  debug("Turning on camera");
}

void camera_off()
{
  digitalWrite(CAM, HIGH);
  camera_state = false;
  debug("Turning off camera");
}

void light_on()
{
  analogWrite(LIGHT, LIGHT_BRIGHTNESS);
  light_state = true;
  debug("Turning on light");
}

void light_off()
{
  digitalWrite(LIGHT, LOW);
  light_state = false;
  debug("Turning off light");
}

void laser_on()
{
  analogWrite(LASER, LASER_BRIGHTNESS);
  debug("Turning on laser");
}

void laser_off()
{
  digitalWrite(LASER, LOW);
  debug("Turning off laser");
}

/**
 * Take a photo using serial camera module and send it to bluetooth serial port
 */
void photo()
{
#ifdef USE_SERIAL_CAMERA
  /* For some reason setting image size doesn't work and always defaults to 320x240
  cam.setImageSize(VC0706_160x120);
  uint8_t imgsize = cam.getImageSize();
  if (imgsize == VC0706_640x480) debug("640x480");
  else if (imgsize == VC0706_320x240) debug("320x240");
  else if (imgsize == VC0706_160x120) debug("160x120");
  else debug("Unknown image size!");
  */
  
  if (!cam.takePicture())
  {                                                                                                                                               
    debug("Failed to take a photo");                                                                                                              
    return;                                                                                                                                       
  }     

  debug("Took a photo, sending to bluetooth");
  uint16_t data_left = cam.frameLength();
  while(data_left > 0)
  {
    uint8_t read_amount = min(64, data_left); // TODO: try 64
    uint8_t *buf = cam.readPicture(read_amount);
    BT_SERIAL.write(buf, read_amount);
    data_left -= read_amount;
  }

  // Send four bytes "EOF\n" finally to signal end of JPEG - highly improbable to occur naturally in JPEG data
  BT_SERIAL.print("EOF\n");

  // Need this in order to take a new photo later
  cam.resumeVideo();

  debug("Photo sent");
#else
  debug("Serial camera support not enabled");
#endif
}
