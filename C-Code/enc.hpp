#ifndef ENC_H
#define ENC_H

#include "drivers/encoder/encoder.hpp"

using namespace encoder;

static Encoder init_encoder(void)
{
  // Create an encoder on the 3 ADC pins, using PIO 0 and State Machine 0
  constexpr uint PIN_A = 26;  // The A channel pin
  constexpr uint PIN_B = 28;  // The B channel pin
  constexpr uint PIN_C = 27;  // The common pin

  Encoder enc(pio0, 0, {PIN_A, PIN_B}, PIN_C);
     // enc.direction(REVERSED_DIR); // Uncomment to reverse the counting direction
  enc.init();

  return enc;
}

#endif // ENC_H
