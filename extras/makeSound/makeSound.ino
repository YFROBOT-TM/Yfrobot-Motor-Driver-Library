// make sound
#define SOUND_PIN    12       // arduino pin number

extern void MSound(byte cNotes, ...);


void setup() {
  // put your setup code here, to run once:
  MSound(3, 60, 2000, 80, 2250, 100, 2500);
  delay(500);
  MSound(3, 100, 2500, 80, 2250, 60, 2000);
  delay(500);
  //  MSound(3, 60, 300, 80, 400, 100, 500);
  //  delay(500);
  //  MSound(3, 60, 300, 80, 400, 100, 500);
  //  delay(500);
  //  MSound(3, 100, 500, 80, 400, 60, 300);
  //  delay(500);
  //  MSound(6, 100, 500, 100, 600, 100, 700, 100, 800, 100, 900, 100, 1000); // 开机提示音
  //  delay(500);
  //  MSound(6, 100, 1000, 100, 1100, 100, 1200, 100, 1300, 100, 1400, 100, 1500); // 开机提示音
  //  delay(500);
  //  for (int i = 0; i < 10; i++) {
  //    MSound(1, 100, 1800);
  //    delay(500);
  //  }
  MSound(6, 100, 1600, 100, 1700, 100, 1800, 100, 1900, 100, 2000, 100, 2100); // 开机提示音
  delay(500);
  //  MSound(6, 60, 1000, 60, 1100, 60, 1200, 60, 1300, 60, 1400, 80, 1500); // 开机提示音
}

void loop() {
  // put your main code here, to run repeatedly:

}


// BUGBUG:: Move to some library...
//==============================================================================
//    SoundNoTimer - Quick and dirty tone function to try to output a frequency
//            to a speaker for some simple sounds.
//    快速而粗糙的音调功能试图向扬声器输出一些频率以获得一些简单的声音。
//==============================================================================
#ifdef SOUND_PIN
void SoundNoTimer(unsigned long duration,  unsigned int frequency) {
#ifdef __AVR__
  volatile uint8_t *pin_port;
  volatile uint8_t pin_mask;
#else
  volatile uint32_t *pin_port;
  volatile uint16_t pin_mask;
#endif
  long toggle_count = 0;
  long lusDelayPerHalfCycle;

  // Set the pinMode as OUTPUT
  pinMode(SOUND_PIN, OUTPUT);

  pin_port = portOutputRegister(digitalPinToPort(SOUND_PIN));
  pin_mask = digitalPinToBitMask(SOUND_PIN);

  toggle_count = 2 * frequency * duration / 1000;
  lusDelayPerHalfCycle = 1000000L / (frequency * 2);

  // if we are using an 8 bit timer, scan through prescalars to find the best fit
  while (toggle_count--) {
    // toggle the pin
    *pin_port ^= pin_mask;

    // delay a half cycle
    delayMicroseconds(lusDelayPerHalfCycle);
  }
  *pin_port &= ~(pin_mask);  // keep pin low after stop
  //  *pin_port |= pin_mask;  // keep pin HIGH after stop 保持高电平，否则电路一直导通大电流消耗
}

void MSound(byte cNotes, ...) {
  va_list ap;
  unsigned int uDur;
  unsigned int uFreq;
  va_start(ap, cNotes);

  while (cNotes > 0) {
    uDur = va_arg(ap, unsigned int);
    uFreq = va_arg(ap, unsigned int);
    SoundNoTimer(uDur, uFreq);
    cNotes--;
  }
  va_end(ap);
}
#else
void MSound(byte cNotes, ...)
{
};
#endif
