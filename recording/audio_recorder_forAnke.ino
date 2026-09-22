/*
  TUI 2026 — record on one object, play on another
  SKETCH B — THE RECORDER (the object with the microphone)

  A hall-effect sensor replaces the push button. Bring the RECORD magnet near,
  then pull all magnets away (the sensor settles into the OPEN band - the
  cube's lid is open): recording starts immediately, no hold delay, and
  continues for as long as every reading stays in the OPEN band. The instant
  the sensor reads anything else at all (closing the lid, or any other
  magnet), recording stops right there and the clip is finalized and sent to
  the other board over WiFi.

  This board does NOT decide when to play back - it has no speaker of its
  own. Instead it continuously broadcasts its raw sensor reading to the
  player board (see "TUI2" below) every time that reading changes, and the
  player does its own band classification and decides when to start/stop
  playback. This keeps both boards using one shared protocol/threshold set
  instead of the recorder unilaterally telling the player what to do.

  The other board runs audio_player.ino, which makes its own little WiFi
  network; this board joins it. There is no router involved, no internet, no
  password.

  ---------------------------------------------------------------------
  HALL SENSOR THRESHOLDS
  ---------------------------------------------------------------------
  Raw analog readings (12-bit ADC, 0-4095) fall into one of three measured
  bands, or none of them. These must match audio_player.ino's
  CUBE_RECORD_LO/HI, CUBE_OPEN_LO/HI, CUBE_LISTEN_LO/HI exactly, since each
  board does its own classification from the same raw value:
      800-1200    RECORD magnet present
      1700-2600   OPEN - no magnet nearby (cube's lid open)
      2700-4300   LISTEN magnet present

  Presenting RECORD "arms" recording: the moment OPEN is seen next, it
  starts, and it ends the moment the sensor reads anything other than OPEN -
  checked against the immediate reading, not smoothed, so it reacts right
  away. LISTEN has no logic here at all - the player board handles it
  entirely, from the raw readings this board broadcasts.

  What the LED tells you:
      slow blink      not connected to the player yet
      steady on       recording
      fast flicker    sending
      three flashes   sent and acknowledged
      long-short-long the player did not take it; it will try again by itself

  Board:  Arduino Nano ESP32 (needs PSRAM).
  Parts:  INMP441 I2S microphone, a hall-effect sensor, a red LED.

  Wiring:
      INMP441          Nano ESP32
      VDD  ──────────  3V3    <- 3.3 V ONLY. 5 V is above its absolute maximum.
      GND  ──────────  GND
      SCK  ──────────  D2
      WS   ──────────  D3
      SD   ──────────  D4
      L/R  ──────────  GND    (makes it the left channel — see README)

      hall-effect sensor:
          VCC ── 3V3
          GND ── GND
          OUT ── A1

      red recording LED:
          D5 ── 330 ohm resistor ── LED anode
          LED cathode ── GND
*/

#include <Arduino.h>

// The ESP_I2S library arrived with ESP32 board package 3.0. On a 2.x package
// the include below fails with "ESP_I2S.h: No such file or directory". See the
// README section "If ESP_I2S.h is not found".
#ifndef ESP_ARDUINO_VERSION_MAJOR
#error "ESP32 board package not detected. Tools > Board > select Arduino Nano ESP32."
#elif ESP_ARDUINO_VERSION_MAJOR < 3
#error "Needs ESP32 board package 3.0 or newer. Boards Manager > install 'esp32' by Espressif Systems, then pick Arduino Nano ESP32 under it."
#endif

#include <ESP_I2S.h>
#include <WiFi.h>

// ---------- things you might want to change ----------

// Must match B_player exactly. The password has to be at least 8 characters.
const char *LINK_SSID = "tui-audio";
const char *LINK_PASS = "tui2026bergamo";

const int I2S_BCLK = D2;      // to mic SCK
const int I2S_WS   = D3;      // to mic WS
const int I2S_DIN  = D4;      // from mic SD

const int SENSOR_PIN = A1;    // hall-effect sensor OUT - replaces the old button
const int RED_LED_PIN = D5;   // blinks while recording - see wiring note above

// Measured hall-sensor bands - see the HALL SENSOR THRESHOLDS note above.
const int BAND_RECORD_LO = 800, BAND_RECORD_HI = 1200;
const int BAND_OPEN_LO = 1700, BAND_OPEN_HI = 2600;
const int BAND_LISTEN_LO = 2700, BAND_LISTEN_HI = 4300;

const unsigned long HALL_DEBOUNCE_MS = 50;         // ordinary band-classification debounce

const uint32_t SAMPLE_RATE = 16000;
const uint32_t MAX_SECONDS = 60;

// The INMP441 gives 24 bits of which we keep the top 16. 
// Raise if recordings are faint, lower if they clip.
const float MIC_GAIN = 4.0;

// Which of the two words in each I2S frame the microphone actually fills.
//   0 = the first word
//   1 = the second word
//
// Tie the microphone's L/R pin to GND regardless of what you set here. The
// datasheet requires that pin hard-tied; left floating it is undefined, and it
// can land differently from one power-up to the next. 

#define MIC_ON_RIGHT 1

// -----------------------------------------------------

const char *PLAYER_IP   = "192.168.4.1";   // the player is always this address
const uint16_t LINK_PORT = 5001;

// WHY THE BUS RUNS AT 32 BITS
//
// The INMP441 datasheet is strict about this: "The INMP441 must always have 64
// clock cycles for every stereo data-word", i.e. 32 clocks per channel. Ask the
// ESP32 for 16-bit slots and the microphone is being clocked in a way it does
// not support, which is where most of the "it records silence" and "it records
// noise" stories come from.
//
// So the bus runs at 32 bits and we keep the top half of every sample as we
// store it. Everything downstream — PSRAM, the network, the WAV — stays plain
// 16-bit mono, so the player needs to know nothing about this.

const size_t WAV_HEADER_BYTES = 44;
const size_t BYTES_PER_SECOND = SAMPLE_RATE * 2;               // stored, 16-bit
const size_t MAX_AUDIO_BYTES  = BYTES_PER_SECOND * MAX_SECONDS;
const size_t CLIP_CAPACITY    = WAV_HEADER_BYTES + MAX_AUDIO_BYTES;

const size_t STORE_CHUNK = 2048;     // bytes of 16-bit audio per pass
const size_t SEND_CHUNK  = 1460;     // one ethernet-ish packet of payload

// The bus carries a stereo frame: two 32-bit words per sample, of which the
// microphone fills exactly one. This board has no output board on it, so it
// does not strictly need stereo — but A_bench_test does, and running both the
// same way means whatever you proved on the bench is what runs here. One
// capture path.
const size_t BUS_SLOTS = 2;

// Staging buffer for the raw 32-bit frames. Ordinary RAM, not PSRAM.
// Worst case: one pass of 16-bit output needs four times as many raw bytes.
static uint8_t rawIn[STORE_CHUNK * 4];

const unsigned long RETRY_EVERY_MS = 3000;
const unsigned long SEND_TIMEOUT_MS = 20000;

I2SClass i2s;

uint8_t *clip      = nullptr;
size_t   clipBytes = 0;

bool          clipWaitingToSend = false;
unsigned long lastSendAttemptMs = 0;

// ---- hall sensor state machine ----
enum Band { BAND_NONE, BAND_RECORD, BAND_OPEN, BAND_LISTEN };
Band rawLast = BAND_NONE;      // last raw reading, for debounce timing
Band stableBand = BAND_NONE;   // current debounced band
unsigned long lastChangeAt = 0;

Band prevBandForArm = BAND_NONE;
bool justEnteredRecord = false;

enum HallState { HALL_IDLE, HALL_RECORD_ARMED };
HallState hallState = HALL_IDLE;

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  // This board makes its own USB port, so the port vanishes and comes back
  // every time the board resets, and the Serial Monitor needs about a second
  // to reattach. Anything printed before that is simply lost 
  //
  // So wait for the monitor, but with a deadline.
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 2000) { }
  delay(100);   // the port is open; give the monitor a moment to start listening

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  digitalWrite(RED_LED_PIN, LOW);

  if (ESP.getPsramSize() == 0) {
    haltWith("No PSRAM. Check the board really is an Arduino Nano ESP32.");
  }
  clip = (uint8_t *) ps_malloc(CLIP_CAPACITY);
  if (clip == nullptr) haltWith("Could not reserve the clip buffer in PSRAM.");

  i2s.setPins(I2S_BCLK, I2S_WS, -1, I2S_DIN);     // -1: no output on this board
  if (!i2s.begin(I2S_MODE_STD, SAMPLE_RATE, I2S_DATA_BIT_WIDTH_32BIT,
                 I2S_SLOT_MODE_STEREO)) {
    haltWith("I2S would not start. Check the pin numbers.");
  }

  // Start joining the player's network. We do NOT sit and wait for it here —
  // the button should work whether or not the other object is switched on.
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);                 // keeps the link quick to respond
  WiFi.begin(LINK_SSID, LINK_PASS);

  Serial.println("Ready. Present the RECORD magnet, then open the cube to start recording.");
}

void loop() {
  keepWifiAlive();
  showStatusOnLed();

  updateStableBand();
  updateArmEdges();
  updateHallStateMachine();

  // Retry on a timer rather than blocking. If the player is switched off, the
  // clip simply waits in PSRAM until it comes back.
  if (clipWaitingToSend && millis() - lastSendAttemptMs > RETRY_EVERY_MS) {
    lastSendAttemptMs = millis();
    if (WiFi.status() == WL_CONNECTED && sendClip()) {
      clipWaitingToSend = false;
      flashLed(3, 120);
      Serial.println("sent");
    } else {
      Serial.println("not sent, will try again");
    }
  }
}

// ---------------------------------------------------------------------------
// Hall sensor: classification, debounce, arm edges, and the state machine
// that starts local recording. LISTEN/playback decisions live entirely on
// the player board - see sendCubeState() further down.

Band classify(int val) {
  if (val >= BAND_RECORD_LO && val <= BAND_RECORD_HI) return BAND_RECORD;
  if (val >= BAND_OPEN_LO && val <= BAND_OPEN_HI) return BAND_OPEN;
  if (val >= BAND_LISTEN_LO && val <= BAND_LISTEN_HI) return BAND_LISTEN;
  return BAND_NONE;
}

// Keeps stableBand current every loop, debounced against sensor noise, and
// broadcasts the raw reading to the player board over TUI2 whenever it
// changes - see sendCubeState() below.
void updateStableBand() {
  int val = analogRead(SENSOR_PIN);
  Band raw = classify(val);

  if (raw != rawLast) {
    lastChangeAt = millis();
    rawLast = raw;
  }
  if ((millis() - lastChangeAt) >= HALL_DEBOUNCE_MS && raw != stableBand) {
    stableBand = raw;
    Serial.print("hall band changed, raw=");
    Serial.println(val);
    sendCubeState(val);
  }
}

// Arming only fires the moment the reading freshly enters RECORD, not for as
// long as it happens to sit there.
void updateArmEdges() {
  justEnteredRecord = (stableBand == BAND_RECORD && prevBandForArm != BAND_RECORD);
  prevBandForArm = stableBand;
}

void updateHallStateMachine() {
  switch (hallState) {
    case HALL_IDLE:
      if (justEnteredRecord) hallState = HALL_RECORD_ARMED;
      break;

    case HALL_RECORD_ARMED:
      if (stableBand == BAND_OPEN) {
        recordWhileOpen();             // blocking - runs until any non-OPEN reading
        clipWaitingToSend = true;
        lastSendAttemptMs = 0;         // try sending immediately
        hallState = HALL_IDLE;
      } else if (stableBand != BAND_RECORD) {
        hallState = HALL_IDLE;         // left RECORD without ever reaching OPEN - cancelled
      }
      break;
  }
}

// ---------------------------------------------------------------------------
// Recording - starts the instant RECORD is followed by OPEN, no hold delay.
// Ends the instant the sensor reads anything other than OPEN - checked here
// against the immediate, undebounced reading (rawLast, refreshed by
// updateStableBand() every iteration) rather than the smoothed stableBand,
// so a stray reading right at a band edge can't leave recording running on
// a stale "still open" value.

void recordWhileOpen() {
  Serial.println("recording...");
  digitalWrite(LED_BUILTIN, HIGH);

  size_t audioBytes = 0;
  updateStableBand();
  while (rawLast == BAND_OPEN && audioBytes < MAX_AUDIO_BYTES) {
    size_t room    = MAX_AUDIO_BYTES - audioBytes;
    size_t wantOut = room < STORE_CHUNK ? room : STORE_CHUNK;

    size_t wantRaw = (wantOut / 2) * BUS_SLOTS * 4;   // stereo, 32-bit, on the bus
    size_t gotRaw  = i2s.readBytes((char *)rawIn, wantRaw);
    audioBytes += shrinkTo16Bit(rawIn, gotRaw,
                                clip + WAV_HEADER_BYTES + audioBytes, MIC_GAIN);

    // Blinks the red LED at a ~250ms rate for as long as recording continues -
    // same non-blocking millis()-based approach used for the "flicker while
    // sending" LED_BUILTIN pattern further down in this file.
    digitalWrite(RED_LED_PIN, (millis() / 250) % 2);

    updateStableBand();   // refreshes rawLast (immediate) for the next check above
  }

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(RED_LED_PIN, LOW);

  writeWavHeader(clip, audioBytes, SAMPLE_RATE);
  clipBytes = WAV_HEADER_BYTES + audioBytes;

  Serial.printf("recorded %u bytes, %.1f seconds\n",
                (unsigned)audioBytes, audioBytes / (float)BYTES_PER_SECOND);
}

// ---------------------------------------------------------------------------
// Cube-state broadcast - a short "TUI2" + 2-byte raw sensor reading message,
// sent every time the debounced band changes (see updateStableBand() above).
// The player does its own band classification from this raw value and
// decides entirely on its own when to start/stop playback - this board has
// no say in that beyond reporting what the sensor actually reads.

void sendCubeState(int rawValue) {
  WiFiClient client;
  client.setTimeout(3);
  if (!client.connect(PLAYER_IP, LINK_PORT)) {
    Serial.println("player did not answer (cube state)");
    return;
  }

  uint8_t msg[6];
  memcpy(msg, "TUI2", 4);
  msg[4] = (uint8_t)(rawValue & 0xFF);
  msg[5] = (uint8_t)((rawValue >> 8) & 0xFF);
  client.write(msg, 6);
  client.flush();

  unsigned long waitStart = millis();
  while (client.connected() && !client.available()) {
    if (millis() - waitStart > 2000) {
      Serial.println("cube state — no acknowledgement");
      client.stop();
      return;
    }
    delay(1);
  }
  client.read();   // just the 'K' - nothing further to act on here
  client.stop();
}

// ---------------------------------------------------------------------------
// Sending
//
// The format on the wire is deliberately tiny:
//
//     "TUI1"  4 bytes, so the player knows this is us and not a stray browser
//     length  4 bytes, how many bytes of WAV follow, smallest byte first
//     the WAV itself
//
// and the player answers with a single 'K' once it has all of it. Without the
// length the player would have no way of knowing when a clip has finished.

bool sendClip() {
  if (clipBytes <= WAV_HEADER_BYTES) return true;   // nothing worth sending

  WiFiClient client;
  client.setTimeout(5);                              // seconds, for the reply
  if (!client.connect(PLAYER_IP, LINK_PORT)) {
    Serial.println("player did not answer");
    return false;
  }

  uint8_t head[8];
  memcpy(head, "TUI1", 4);
  head[4] = (uint8_t)( clipBytes        & 0xFF);
  head[5] = (uint8_t)((clipBytes >>  8) & 0xFF);
  head[6] = (uint8_t)((clipBytes >> 16) & 0xFF);
  head[7] = (uint8_t)((clipBytes >> 24) & 0xFF);
  client.write(head, 8);

  Serial.printf("sending %u bytes", (unsigned)clipBytes);

  size_t        sent   = 0;
  unsigned long startMs = millis();
  while (sent < clipBytes) {
    size_t left = clipBytes - sent;
    size_t want = left < SEND_CHUNK ? left : SEND_CHUNK;

    size_t n = client.write(clip + sent, want);
    if (n > 0) {
      sent += n;
    } else if (!client.connected() || millis() - startMs > SEND_TIMEOUT_MS) {
      Serial.println(" — link dropped part way");
      client.stop();
      return false;
    }
    // A short breath so the radio and the watchdog both get a turn. This is a
    // yield, not a pause — without it a tight write loop can reset the board.
    delay(1);

    digitalWrite(LED_BUILTIN, (millis() / 60) % 2);  // flicker while sending
  }
  client.flush();

  // Wait for the player's 'K'. If it never comes, treat the clip as unsent and
  // let the retry timer have another go.
  unsigned long waitStart = millis();
  while (client.connected() && !client.available()) {
    if (millis() - waitStart > 5000) {
      Serial.println(" — no acknowledgement");
      client.stop();
      return false;
    }
    delay(1);
  }
  int reply = client.read();
  client.stop();
  digitalWrite(LED_BUILTIN, LOW);

  if (reply == 'K') {
    // Worth watching the first few times: it tells you how long the object is
    // busy for, which is a design fact and not just a technical one.
    unsigned long tookMs = millis() - startMs;
    Serial.printf(" — ok in %lu ms (%.0f kB/s)\n",
                  tookMs, clipBytes / (float)tookMs);
    return true;
  }
  Serial.println(" — player refused it");
  return false;
}

// ---------------------------------------------------------------------------
// WiFi housekeeping, all on millis()

void keepWifiAlive() {
  static unsigned long lastCheckMs = 0;
  static bool wasConnected = false;

  if (millis() - lastCheckMs < 2000) return;
  lastCheckMs = millis();

  bool connected = (WiFi.status() == WL_CONNECTED);
  if (connected != wasConnected) {
    Serial.println(connected ? "joined the player's network" : "lost the player");
    wasConnected = connected;
  }
  if (!connected) WiFi.begin(LINK_SSID, LINK_PASS);
}

void showStatusOnLed() {
  // Only the idle states are drawn here; recording and sending drive the LED
  // themselves while they are busy.
  if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_BUILTIN, (millis() / 500) % 2);   // slow blink
  }
}

// ---------------------------------------------------------------------------
// Helpers — same as A_bench_test

// Each 32-bit word from the bus holds the microphone's 24-bit sample at the
// top. Shifting right by 16 keeps the loudest 16 bits of it, which is all a
// small speaker can reproduce anyway. The frame holds two words and the
// microphone fills one of them; the other is dropped. If everything records
// silence, that is the first thing to check — run C_diagnose, which prints
// both halves. Returns how many bytes it wrote.
size_t shrinkTo16Bit(const uint8_t *from32, size_t byteCount32,
                     uint8_t *to16, float gain) {
  const int32_t *in  = (const int32_t *)from32;
  int16_t       *out = (int16_t *)to16;
  size_t samples = (byteCount32 / 4) / BUS_SLOTS;

  const size_t micWord = MIC_ON_RIGHT ? 1 : 0;

  for (size_t i = 0; i < samples; i++) {
    float v = (float)(in[i * BUS_SLOTS + micWord] >> 16) * gain;
    if (v >  32767.0) v =  32767.0;       // stop at the limit rather than
    if (v < -32768.0) v = -32768.0;       // wrapping round into noise
    out[i] = (int16_t)v;
  }
  return samples * 2;
}

void writeWavHeader(uint8_t *h, size_t audioBytes, uint32_t rate) {
  memcpy(h, "RIFF", 4);
  *(uint32_t *)(h +  4) = 36 + audioBytes;
  memcpy(h + 8, "WAVEfmt ", 8);
  *(uint32_t *)(h + 16) = 16;
  *(uint16_t *)(h + 20) = 1;
  *(uint16_t *)(h + 22) = 1;
  *(uint32_t *)(h + 24) = rate;
  *(uint32_t *)(h + 28) = rate * 2;
  *(uint16_t *)(h + 32) = 2;
  *(uint16_t *)(h + 34) = 16;
  memcpy(h + 36, "data", 4);
  *(uint32_t *)(h + 40) = audioBytes;
}

void flashLed(int times, int ms) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_BUILTIN, HIGH); delay(ms);
    digitalWrite(LED_BUILTIN, LOW);  delay(ms);
  }
}

void haltWith(const char *message) {
  pinMode(LED_BUILTIN, OUTPUT);
  while (true) {
    Serial.println(message);
    digitalWrite(LED_BUILTIN, HIGH); delay(150);
    digitalWrite(LED_BUILTIN, LOW);  delay(850);
  }
}
