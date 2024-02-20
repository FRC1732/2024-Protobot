#include <Adafruit_NeoPixel.h>

#define DIGITAL_D0 8
#define DIGITAL_D1 9
#define DIGITAL_D2 10
#define DIGITAL_D3 11
#define DIGITAL_D4 12

#define OUTPUT_D0 3
#define OUTPUT_D1 4
#define OUTPUT_D2 5
#define OUTPUT_D3 6
#define OUTPUT_D4 7

#define LEDSTRIP_1 A6
#define LEDSTRIP_2 A7

#define NUMPIXELS 20  // number of neopixels in strip
#define DELAY_TIME 200
#define INTENSITY 255

Adafruit_NeoPixel pixels1(NUMPIXELS, LEDSTRIP_1, NEO_GRB + NEO_KHZ800);

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

  pixels1.begin();
}

void setColor(bool red, bool green, bool blue) {
  pixels1.clear();
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels1.setPixelColor(i, pixels1.Color(INTENSITY * (int)red, INTENSITY * (int)green * .50, INTENSITY * (int)blue));
  }
  pixels1.show();
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
  Serial.println(mode);

  if (mode == 1) {
    setColor(false, true, false);
  }
}

