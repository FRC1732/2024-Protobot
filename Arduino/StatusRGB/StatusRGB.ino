#include <Adafruit_NeoPixel.h>

#define LED_DATA 2

#define NUMPIXELS 69  // number of neopixels in strip
#define NUM_SEG 23    // number of neopixels in a segment
#define DELAY_TIME 200
#define INTENSITY 255

Adafruit_NeoPixel pixels(NUMPIXELS, LED_DATA, NEO_GRB + NEO_KHZ800);

#define DIGITAL_D0 3
#define DIGITAL_D1 4
#define DIGITAL_D2 5
#define DIGITAL_D3 6
#define DIGITAL_D4 7

#define OUTPUT_D0 8
#define OUTPUT_D1 9
#define OUTPUT_D2 10
#define OUTPUT_D3 11
#define OUTPUT_D4 12

int mode = 0;
int timer = 0;

void setup() {
  Serial.begin(250000);

  pinMode(DIGITAL_D0, INPUT_PULLUP);
  pinMode(DIGITAL_D1, INPUT_PULLUP);
  pinMode(DIGITAL_D2, INPUT_PULLUP);
  pinMode(DIGITAL_D3, INPUT_PULLUP);
  pinMode(DIGITAL_D4, INPUT_PULLUP);

  pinMode(OUTPUT_D0, OUTPUT);
  pinMode(OUTPUT_D1, OUTPUT);
  pinMode(OUTPUT_D2, OUTPUT);
  pinMode(OUTPUT_D3, OUTPUT);
  pinMode(OUTPUT_D4, OUTPUT);

  pixels.begin();
}

void loop() {
  bool b0, b1, b2, b3, b4;

  // HIGH is 0, LOW is 1 on the inputs
  b0 = !digitalRead(DIGITAL_D0);
  b1 = !digitalRead(DIGITAL_D1);
  b2 = !digitalRead(DIGITAL_D2);
  b3 = !digitalRead(DIGITAL_D3);
  b4 = !digitalRead(DIGITAL_D4);

  digitalWrite(OUTPUT_D0, b0);
  digitalWrite(OUTPUT_D1, b1);
  digitalWrite(OUTPUT_D2, b2);
  digitalWrite(OUTPUT_D3, b3);
  digitalWrite(OUTPUT_D4, b4);

  mode = ((int)b0 << 0) + ((int)b1 << 1) + ((int)b2 << 2) + ((int)b3 << 3) + ((int)b4 << 4);
  Serial.print("Mode: ");
  Serial.println(mode);}