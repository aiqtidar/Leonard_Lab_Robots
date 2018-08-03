#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS1      150
#define NUMPIXELS2      450

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.


Adafruit_NeoPixel pixels2 = Adafruit_NeoPixel(NUMPIXELS2, PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixels1 = Adafruit_NeoPixel(NUMPIXELS1, PIN, NEO_GRB + NEO_KHZ800);

int delayval = 500; // delay for half a second

void setup() {

  pixels1.begin(); // This initializes the NeoPixel library.
  pixels2.begin(); // This initializes the NeoPixel library.
}

void loop() {

  for(int i=0;i<NUMPIXELS1;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels1.setPixelColor(i, pixels1.Color(255,0,0)); // Moderately bright green color.

    pixels1.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).

  }

    for(int i= NUMPIXELS2-150;i<NUMPIXELS2;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels2.setPixelColor(i, pixels2.Color(0,0,255)); // Moderately bright green color.

    pixels2.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).

  }

}
