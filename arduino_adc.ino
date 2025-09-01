static constexpr uint8_t xCoordIn = A0;
static constexpr uint8_t yCoordIn = A1;

static constexpr uint8_t outPortLength = 4;
static constexpr uint8_t maxOutValue = pow(2, outPortLength) - 1;
static constexpr float maxAnalogIn = 1023.0f;

#define REC_LENGTH 15
static int xReadings[REC_LENGTH];
static size_t xPtr = 0;

static int yReadings[REC_LENGTH];
static size_t yPtr = 0;

void setup() {

  pinMode(xCoordIn, INPUT);
  pinMode(yCoordIn, INPUT);

  DDRD = 0xFF;
}

void writeOutAdc(uint8_t analogXIn, uint8_t analogYIn) {
  uint8_t combinedOut = (analogYIn << outPortLength) | (0x0F & analogXIn);
  PORTD = combinedOut;
}

//just applies low-pass filter to the stick data
int filterAndAverage(const int *data, size_t length, float alpha) {
  float smoothAccum = alpha * data[0];
  float smoothPrev = smoothAccum;

  for (size_t u = 1; u < length; ++u) {
    smoothPrev = smoothPrev + alpha * (static_cast<float>(data[u]) - smoothPrev);
    smoothAccum += smoothPrev;
  }

  return ceil(smoothAccum / length);
}

void loop() {
  const int analogXIn = (static_cast<float>(analogRead(xCoordIn)) * maxOutValue) / maxAnalogIn;
  const int analogYIn = (static_cast<float>(analogRead(yCoordIn)) * maxOutValue) / maxAnalogIn;

  xReadings[xPtr++] = analogXIn;
  yReadings[yPtr++] = analogYIn;

  if (xPtr == REC_LENGTH) {
    const int smoothX = filterAndAverage(xReadings, REC_LENGTH, 0.85f);
    const int smoothY = filterAndAverage(yReadings, REC_LENGTH, 0.85f);
    xPtr = 0;
    yPtr = 0;

    writeOutAdc(smoothX, smoothY);
  }
}
