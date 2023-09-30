#include <Arduino.h>
#include <RPLidar.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

RPLidar lidar;

// The PWM pin for control the speed of RPLIDAR's motor.
constexpr int RPLIDAR_MOTOR = 3;

constexpr float MAX_lidarDistanceCm_CM = 20.0;
constexpr int MAX_MOTOR_SPEED = 255;


#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// On an arduino MEGA 2560: 20(SDA), 21(SCL)
// On an arduino LEONARDO:   2(SDA),  3(SCL), ...
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  // Bind the RPLIDAR driver to the Arduino hardware serial
  lidar.begin(Serial1);

  pinMode(RPLIDAR_MOTOR, OUTPUT);

  // Show initial display buffer contents on the screen --
  // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000); // Pause for 2 seconds

  // Clear the buffer
  display.clearDisplay();

  // Draw a single pixel in white
  display.drawPixel(10, 10, SSD1306_WHITE);

  testdrawstyles();

  // Show the display buffer on the screen. You MUST call display() after
  // drawing commands to make them visible on screen!
  display.display();
}

void loop() {

  if (IS_OK(lidar.waitPoint())) {
    float lidarDistanceCm = lidar.getCurrentPoint().distance / 10.0;
    float lidarAngleDeg = lidar.getCurrentPoint().angle;
    //bool  startBit = lidar.getCurrentPoint().startBit;
    byte  quality  = lidar.getCurrentPoint().quality;

    if (quality == 0 || lidarDistanceCm == 0.00) {
      return;
    }

    if (lidarDistanceCm <= MAX_lidarDistanceCm_CM) {

      display.clearDisplay();

      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0,0);
      display.print(F("Angle: "));
      display.println(lidarAngleDeg);
      display.print(F("Distance: "));
      display.println(lidarDistanceCm);
      display.display();

    }
  } else {
    // Stop the rplidar motor
    analogWrite(RPLIDAR_MOTOR, 0);
    Serial.println("Nope :(");

    // try to detect RPLIDAR...
    rplidar_response_device_info_t info;
    if (IS_OK(lidar.getDeviceInfo(info, 100))) {
      // detected...
      lidar.startScan();

      // start motor rotating at max allowed speed
      analogWrite(RPLIDAR_MOTOR, MAX_MOTOR_SPEED);
      delay(1000);
    }
  }
}

void testdrawstyles(void) {
  display.clearDisplay();

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("Hello, world!"));

  display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  display.println(3.141592);

  display.setTextSize(2);             // Draw 2X-scale text
  display.setTextColor(SSD1306_WHITE);
  display.print(F("0x")); display.println(0xDEADBEEF, HEX);

  display.display();
  delay(2000);
}
