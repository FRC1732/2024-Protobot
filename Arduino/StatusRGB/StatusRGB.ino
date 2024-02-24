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

#define LEDSTRIP_1 A5
#define LEDSTRIP_2 A7  // FIXME change this when the board is rewired

#define NUMPIXELS 28  // number of neopixels in strip
#define DELAY_TIME 200
#define INTENSITY 255

#define IDLE_CYCLE 100
#define IDLE_BLOCK 8

Adafruit_NeoPixel pixels1(NUMPIXELS, LEDSTRIP_1, NEO_GRB + NEO_KHZ800);
uint32_t lowBlue = pixels1.Color(0, 0, INTENSITY / 3);
uint32_t highBlue = pixels1.Color(0, 0, INTENSITY);
uint32_t lowGold = pixels1.Color(INTENSITY / 3, INTENSITY / 6, 0);
uint32_t highGold = pixels1.Color(INTENSITY, INTENSITY / 2, 0);


int mode = 0;
int timer = 0;

unsigned long myTime;

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

void setColorInt(int red, int green, int blue) {
  pixels1.clear();
  for (int i = 0; i < NUMPIXELS; i++) {
    pixels1.setPixelColor(i, pixels1.Color(red, green, blue));
  }
  pixels1.show();
}

int elapsedTime = 0;

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

  if (mode >= 16) {
    flashFast(false, true, false);
  } else {
    switch (mode) {
      case 0:  // idle
        idleMode();
        break;

      case 1:  // mode == speaker
        setColorInt(255, 255, 255);
        break;

      case 2:  // !hasClearance
        setColorInt(255, 0, 0);
        break;

      case 3:  // target ready
        setColorInt(0, 125, 0);
        break;

      case 4:  // climbing
        //climberColors(true, true, true); unsure which one of these to use
        climberColorsRainbow();
        break;

      default:
        break;
    }
  }

  myTime = millis();
  timer++;
  delay(1);
  elapsedTime += myTime - millis();
}

void idleMode() {
  pixels1.clear();

  if (timer > IDLE_BLOCK * IDLE_CYCLE || timer < 0) {
    timer = 0;
  }

  int offset = timer / IDLE_CYCLE;

  for (int i = 0; i < NUMPIXELS; i++) {
    uint32_t color;
    int pos = (offset + i) % IDLE_BLOCK;
    if (pos == 0 || pos == 3) {
      color = lowBlue;
    } else if (pos == 1 || pos == 2) {
      color = highBlue;
    } else if (pos == 4 || pos == 7) {
      color = lowGold;
    } else if (pos == 5 || pos == 6) {
      color = highGold;
    } else {
      color = pixels1.Color(0, 0, 0);
    }
    pixels1.setPixelColor(i, color);
  }

  pixels1.show();
}

void flashFast(bool red, bool green, bool blue) {
  if (timer < 15) {
    setColor(red, green, blue);
  }

  if (timer < 30 && timer > 15) {
    setColor(false, false, false);
  }

  if (timer > 31 || timer < 0) {
    timer = 0;
  }
}

int currentlyFlashing[NUMPIXELS];
void climberColors(bool red, bool green, bool blue) {
  if (myTime % 15 == 0) {
    pixels1.clear();
    for (int i = 0; i < NUMPIXELS; i++) {
      int rng = random(1, 25);
      if (rng == 1 && currentlyFlashing[i] == 0) {
        currentlyFlashing[i] = 250;
      } else if (currentlyFlashing[i] > 0) {
        currentlyFlashing[i] = max(0, currentlyFlashing[i] - 20);
      }

      pixels1.setPixelColor(i, pixels1.Color(red * currentlyFlashing[i], green * currentlyFlashing[i], blue * currentlyFlashing[i]));
    }
    pixels1.show();
  }
}

// RAINBOW verison of the climber
int currentlyFlashingColor[NUMPIXELS][3];
void climberColorsRainbow() {
  if (myTime % 20 == 0) {
    pixels1.clear();
    for (int i = 0; i < NUMPIXELS; i++) {
      int rng = random(1, 15);

      if (rng == 1 && currentlyFlashing[i] == 0) {
        currentlyFlashing[i] = 250;
        currentlyFlashingColor[i][0] = random(5, 255);
        currentlyFlashingColor[i][1] = random(5, 255);
        currentlyFlashingColor[i][2] = random(5, 255);

      } else if (currentlyFlashing[i] > 0) {
        currentlyFlashing[i] = max(0, currentlyFlashing[i] - 20);
      }

      pixels1.setPixelColor(i, pixels1.Color(currentlyFlashingColor[i][0] * (currentlyFlashing[i] / 250.0), currentlyFlashingColor[i][1] * (currentlyFlashing[i] / 250.0), currentlyFlashingColor[i][2] * (currentlyFlashing[i] / 250.0)));
    }
    pixels1.show();
  }
}


int currentlyFlashingColorOld[NUMPIXELS][3];
int waitTime = 50;
bool colorDirection = true;

// different take on the idle state, currently broken
void topperLine() {
  if (myTime % waitTime == 0) {
    if (colorDirection == false) {
      waitTime++;
    } else {
      waitTime--;
    }

    if (waitTime == 1 || waitTime == 55) {
      colorDirection = !colorDirection;
    }
    
    bool doColor = false;
    if (timer >= 125 && waitTime <= 60) {
      timer = 0;
      doColor = true;
    }
    pixels1.clear();

    for (int i = 0; i < NUMPIXELS; i++) {
      if (currentlyFlashingColor[i][0] == 0 && currentlyFlashingColor[i][2] == 0) {
          currentlyFlashingColor[i][0] = 0;
          currentlyFlashingColor[i][1] = 0;
          currentlyFlashingColor[i][2] = 255;
      }
    }

    for (int i = 0; i < NUMPIXELS; i++) {
      currentlyFlashingColorOld[i][0] = currentlyFlashingColor[i][0];
      currentlyFlashingColorOld[i][1] = currentlyFlashingColor[i][1];
      currentlyFlashingColorOld[i][2] = currentlyFlashingColor[i][2];      
    }

    for (int i = 0; i < NUMPIXELS ; i++) {
      int putPosition = i;

      if (putPosition == 0) {
        if (!doColor) {
          currentlyFlashingColor[putPosition][0] = 0;
          currentlyFlashingColor[putPosition][1] = 0;
          currentlyFlashingColor[putPosition][2] = 255;
        } else {
          currentlyFlashingColor[putPosition][0] = 255;
          currentlyFlashingColor[putPosition][1] = 125;
          currentlyFlashingColor[putPosition][2] = 0;
        }

      } else {
        shiftColor(putPosition, 1);
      }

      pixels1.setPixelColor(putPosition, pixels1.Color(currentlyFlashingColor[putPosition][0], currentlyFlashingColor[putPosition][1], currentlyFlashingColor[putPosition][2]));
    }
    pixels1.show();
  }
}

void shiftColor(int pos, int direction) {
  currentlyFlashingColor[pos][0] = currentlyFlashingColorOld[pos - direction][0];
  currentlyFlashingColor[pos][1] = currentlyFlashingColorOld[pos - direction][1];
  currentlyFlashingColor[pos][2] = currentlyFlashingColorOld[pos - direction][2];
}
