/* Persistence Of Vision sphere code.
   One SPI channel is sent to a CD4052 multiplexer to drive four strips of DotStar LEDs on a ring.
   Break beam optical sensor monitors motor speed and a PID loop locks it to 420rpm to sync a
   frame buffer.  Transistor/MOSFET drivers using PWM control the motor speed.

   dlf 11/27/2025
*/

#include <Arduino.h>
#include <SPI.h>

// Pins (adjust to your wiring)
const uint8_t SEL_A = 21; // CD4052 A (LSB)
const uint8_t SEL_B = 22; // CD4052 B (MSB)
const uint8_t MOSI_PIN = 13; // ESP32 default MOSI (HSPI) or wire SPI pins
const uint8_t SCK_PIN  = 14; // ESP32 SCK (shared to all strip clocks)

// LED geometry
const int LEDS_PER_STRIP = 48;
const int STRIPS = 4;
const int BYTES_PER_LED = 4; // APA102: 1 brightness + B + G + R
const int BYTES_PER_STRIP = LEDS_PER_STRIP * BYTES_PER_LED;

//##########################
// PID motor speed control
//##########################
const int motorPwmPin = 25;
const int sensorPin = 34;   // Use INPUT_PULLDOWN on a real project
uint32_t lastPID = 0;

// Sensor produces TWO rising edges per revolution
const int pulsesPerRev = 2;

const uint32_t MIN_DT_US = 10000; // ignore pulses closer than 2ms => noise filter
const uint32_t RPM_TIMEOUT_MS = 200; // if no pulses for 200ms, zero RPM

// --- shared state (ISR/main)
volatile uint32_t lastPulseTime = 0; // micros() at last valid pulse
volatile uint32_t pulseDt = 0;       // dt in microseconds between last two valid pulses
volatile uint32_t lastPulseMillis = 0; // millis() when last valid pulse arrived
volatile bool frameSyncFlag = false; // optional flag to synchronize frame updates

// Smoothed RPM (not volatile — updated in main loop)
float motorRPM = 0.0f;

// ====== PID control ======
float targetRPM = 420.0;
//float Kp = 0.7;
//float Ki = 0.3;
//float Kd = 0.05;
float Kp = 0.4;
float Ki = 0.8;
float Kd = 0.2;
float pidIntegral = 460;
float pidLastErr = 0;

// Framebuffer: 128x48 assumed earlier; here we only build per-strip segments
// Example: full framebuffer exists elsewhere; this shows sending one column-worth per strip
uint8_t stripBuf[STRIPS][BYTES_PER_STRIP];

//#############################################
// Shaft speed sensor, motor control functions
//#############################################
// 
void IRAM_ATTR shaftISR() {
  uint32_t now = micros();
  uint32_t dt = now - lastPulseTime;

  // basic debounce/noise filter
  if (dt < MIN_DT_US) {
    return; // ignore likely noise
  }

  // Accept this pulse as valid
  pulseDt = dt;
  lastPulseTime = now;
  lastPulseMillis = millis();
  frameSyncFlag = true; // if you use it to sync LED updates
}

void updateRPMfromPulseDt() {

   // Safely copy volatile pulseDt
  uint32_t dt;
  noInterrupts();
  dt = pulseDt;
  interrupts();

  if (dt == 0) {
    // no valid pulse yet
    return;
  }

  // Compute RPM (dt is microseconds between pulses)
  // pulses_per_sec = 1e6 / dt
  // revolutions_per_sec = pulses_per_sec / pulsesPerRev
  // RPM = revolutions_per_sec * 60
  float pulses_per_sec = 1000000.0f / (float)dt;
  float rpm = (pulses_per_sec * 60.0f) / (float)pulsesPerRev;
  motorRPM = rpm;
}

// Check if shaft is not moving
void checkRPMTimeout() {
  uint32_t nowMs = millis();
  noInterrupts();
  uint32_t lastMs = lastPulseMillis;
  interrupts();

  if ((nowMs - lastMs) > RPM_TIMEOUT_MS) {
    motorRPM = 0.0f;
  }
}

float updatePID(float rpm) {
    float error = targetRPM - rpm;

    pidIntegral += error * 0.05;      // Integral term
    pidIntegral = constrain(pidIntegral, -255/Ki, 255/Ki);

    float derivative = (error - pidLastErr);
    pidLastErr = error;

    float P, I, D;
    P = Kp * error;
    I = Ki * pidIntegral;
    D = Kd * derivative;

    float output = P + I + D;
    output = constrain(output, 200, 255);

    Serial.printf("P: %.2f   I: %.2f    D: %.2f    PWM: %.2f\n", P, I, D, output);
    return output;
}

void setupMotor() {
    ledcSetup(0, 20000, 8);   // 20 kHz PWM, 8-bit resolution
    ledcAttachPin(motorPwmPin, 0);
    ledcWrite(0, 0);
}



//###########################################
// LED strip control functions
//###########################################
void selectStrip(int n) {
  // n = 0..3
  digitalWrite(SEL_A, (n & 0x1));
  digitalWrite(SEL_B, (n >> 1) & 0x1);

  // let the analog switch settle
  delayMicroseconds(2);
}

/*
void sendAPA102Strip(uint8_t *buf) {
  // Start frame: 8 bytes 0x00
  for (int i = 0; i < 8; i++) SPI.transfer(0x00); // 8 bytes = 64 bits

  // Pixel frames: each pixel is [0xE0|brightness, B, G, R]
  // Here assume buf holds [R,G,B] triplets. We will send 0xFF brightness + BGR
  for (int i = 0; i < LEDS_PER_STRIP; ++i) {
    uint8_t r = buf[i*3 + 0];
    uint8_t g = buf[i*3 + 1];
    uint8_t b = buf[i*3 + 2];

    SPI.transfer(0xE0 | 16);  // Brightness (set 31-0)
    SPI.transfer(b);          // <— CHANGED ORDER HERE
    SPI.transfer(g);
    SPI.transfer(r);
  }
  // End frame must be >= number_of_leds/2 bits.
  // To be safe for all APA102 clones, always send 4 bytes.
  for (int i = 0; i < 4; i++) SPI.transfer(0xFF);

}
*/

//###############
// Setup Code
//###############
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("SPI + CD4052 DotStar test");

  // SPI multiplexer control inputs to control four LED strips
  pinMode(SEL_A, OUTPUT);
  pinMode(SEL_B, OUTPUT);
  digitalWrite(SEL_A, LOW);
  digitalWrite(SEL_B, LOW);

  // Set up the motor control
  pinMode(sensorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(sensorPin), shaftISR, FALLING);
  setupMotor();
  ledcWrite(0, 230);  //Start the motor
  delay(1000); // Wait for it to spin up

  // Configure SPI: use SPIClass VSPI or HSPI as appropriate
  // HSPI default pins: SCK=14 MOSI=13 MISO=12 (depends on board). We explicitly set pins below.
  //dlf SPI.begin(SCK_PIN, -1, MOSI_PIN, -1); // SCK, MISO(not used), MOSI, SS(not used)

  // Choose SPI settings (mode 0) - APA102 reads on rising edge, mode 0 is fine
  //dlf SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

/* dlf
  // Fill buffers with simple colors
  for (int s = 0; s < STRIPS; ++s) {
    for (int i = 0; i < LEDS_PER_STRIP; ++i) {
      //if (s == 0) { stripBuf[s][i*3+0] = 255; stripBuf[s][i*3+1] = 0;   stripBuf[s][i*3+2] = 0; } // red
      if (s == 0) { stripBuf[s][i*3+0] = 0; stripBuf[s][i*3+1] = 0;   stripBuf[s][i*3+2] = 0; } // red
      if (s == 1) { stripBuf[s][i*3+0] = 0;   stripBuf[s][i*3+1] = 255; stripBuf[s][i*3+2] = 0; } // green
      if (s == 2) { stripBuf[s][i*3+0] = 0;   stripBuf[s][i*3+1] = 0;   stripBuf[s][i*3+2] = 255; } // blue
      if (s == 3) { stripBuf[s][i*3+0] = 255; stripBuf[s][i*3+1] = 255; stripBuf[s][i*3+2] = 0; }   // yellow
    }
  }
*/
}

//###############
// Loop Code
//###############
void loop() {

  // If ISR flagged a frame sync, update RPM and generate a frame if you need:
  if (frameSyncFlag) {
    frameSyncFlag = false;
    updateRPMfromPulseDt();

  /* dlf
    // send to each strip sequentially
    for (int s = 0; s < STRIPS; ++s) {
      selectStrip(s);
      sendAPA102Strip(stripBuf[s]);
    }
  dlf */
  }

  // Always check timeout to zero rpm if motor stops
  checkRPMTimeout();

  // ====== PID speed regulation ======
  if (millis() - lastPID > 100) {
    lastPID = millis();
    float pwm = updatePID(motorRPM);
    ledcWrite(0, (int)pwm);
    Serial.printf("RPM: %.2f   PWM: %.2f\n", motorRPM, pwm);
  }
}
