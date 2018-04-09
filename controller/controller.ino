/*********************************************************************
This is an example for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

This example is for a 128x64 size display using I2C to communicate
3 pins are required to interface (2 I2C and one reset)

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
 
#include <SoftwareSerial.h>
SoftwareSerial BTserial(2, 3); // RX | TX


//#define OLED_RESET 4
Adafruit_SSD1306 display(4);

const int ledPin =  13;    
const int buttonUp = 12;
const int buttonLeft = 11;
const int buttonRight = 10;
const int buttonDown = 9;


void setup()   { 
  
  pinMode(buttonUp, INPUT);
  pinMode(buttonLeft, INPUT);
  pinMode(buttonRight, INPUT);
  pinMode(buttonDown, INPUT);
              
  Serial.begin(9600);
  BTserial.begin(9600);
   
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
 // display.display();
  //delay(2000);

    // Clear the buffer.
    display.clearDisplay();
  
    // draw a single pixel
    display.drawPixel(10, 10, WHITE);
    // Show the display buffer on the hardware.
  // NOTE: You _must_ call display after making any drawing commands
  // to make them visible on the display hardware!
  display.display();
  delay(2000);
  display.clearDisplay();

  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println(F("Hello, world!"));
  display.setTextColor(BLACK, WHITE); // 'inverted' text
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.println(F("from")); 
  display.println(F("TERRIERBOT"));
  display.display();
  delay(2000);
  display.clearDisplay();
}

void loop() {

  //Serial.println("Terrierbot");
  
  if (digitalRead(buttonUp) == HIGH) {
    //Serial.println("up");
    BTserial.print("up");
  }
  else if (digitalRead(buttonLeft) == HIGH)
  {
    //Serial.println("Left");
    BTserial.print("left");
  }
  else if (digitalRead(buttonRight) == HIGH)
  {
    //Serial.println("Right");
    BTserial.print("right");
  }
  else if (digitalRead(buttonDown) == HIGH)
  {
    //Serial.println("Down");
    BTserial.print("down");
  }


  
 //transmitDetails(info);
 
 delay(100);
}

void transmitDetails(String joyY)
{
  BTserial.print(joyY);
}

void send_state(){
 
 
 BTserial.write('S');
 BTserial.write('E');
 return;
}

void mainDisplay()
{

 
  //BTserial.print(joyX);
  //BTserial.println(Serial.readBytes(buffer,5));
  Serial.println("");
  //Serial.println(xState);
  //Serial.println(yState);
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);

  display.setCursor(0,56);
  display.println("terrierbot v0.01");
  display.display();
  delay(100);

}

