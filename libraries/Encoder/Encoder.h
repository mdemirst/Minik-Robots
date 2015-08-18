/* Encoder Library, for measuring quadrature encoded signals
* http://www.pjrc.com/teensy/td_libs_Encoder.html
* Copyright (c) 2011,2013 PJRC.COM, LLC - Paul Stoffregen <paul@pjrc.com>
*
* Version 1.2 - fix -2 bug in C-only code
* Version 1.1 - expand to support boards with up to 60 interrupts
* Version 1.0 - initial release
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/


#ifndef Encoder_h_
#define Encoder_h_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#elif defined(WIRING)
#include "Wiring.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#include "utility/direct_pin_read.h"

#if defined(ENCODER_USE_INTERRUPTS) || !defined(ENCODER_DO_NOT_USE_INTERRUPTS)
#define ENCODER_USE_INTERRUPTS
#define ENCODER_ARGLIST_SIZE CORE_NUM_INTERRUPT
#include "utility/interrupt_pins.h"
#ifdef ENCODER_OPTIMIZE_INTERRUPTS
#include "utility/interrupt_config.h"
#endif
#else
#define ENCODER_ARGLIST_SIZE 0
#endif



// All the data needed by interrupts is consolidated into this ugly struct
// to facilitate assembly language optimizing of the speed critical update.
// The assembly code uses auto-incrementing addressing modes, so the struct
// must remain in exactly this order.
typedef struct {
  volatile IO_REG_TYPE * pin2_register;
  IO_REG_TYPE            pin2_bitmask;
  int32_t                position;
} Encoder_internal_state_t;

class Encoder
{
public:
  Encoder(uint8_t pin1, uint8_t pin2) {

    pinMode(pin1, INPUT);
    digitalWrite(pin1, HIGH);
    pinMode(pin2, INPUT);
    digitalWrite(pin2, HIGH);

    encoder.pin2_register = PIN_TO_BASEREG(pin2);
    encoder.pin2_bitmask = PIN_TO_BITMASK(pin2);
    encoder.position = 0;
    // allow time for a passive R-C filter to charge
    // through the pullup resistors, before reading
    // the initial state
    delayMicroseconds(2000);

    attach_interrupt(pin1, &encoder);
  }


  inline int32_t read() {
    noInterrupts();
    int32_t ret = encoder.position;
    interrupts();
    return ret;
  }
  inline void write(int32_t p) {
    noInterrupts();
    encoder.position = p;
    interrupts();
  }

private:
  Encoder_internal_state_t encoder;

public:
  static Encoder_internal_state_t * interruptArgs[ENCODER_ARGLIST_SIZE];

  //                           _______         _______       
  //               Pin1 ______|       |_______|       |______ Pin1
  // negative <---         _______         _______         __      --> positive
  //               Pin2 __|       |_______|       |_______|   Pin2

  //	new	new	old	old
  //	pin2	pin1	pin2	pin1	Result
  //	----	----	----	----	------
  //	0	0	0	0	no movement
  //	0	0	0	1	+1
  //	0	0	1	0	-1
  //	0	0	1	1	+2  (assume pin1 edges only)
  //	0	1	0	0	-1
  //	0	1	0	1	no movement
  //	0	1	1	0	-2  (assume pin1 edges only)
  //	0	1	1	1	+1
  //	1	0	0	0	+1
  //	1	0	0	1	-2  (assume pin1 edges only)
  //	1	0	1	0	no movement
  //	1	0	1	1	-1
  //	1	1	0	0	+2  (assume pin1 edges only)
  //	1	1	0	1	-1
  //	1	1	1	0	+1
  //	1	1	1	1	no movement
  /*
  // Simple, easy-to-read "documentation" version :-)
  //
  void update(void) {
  uint8_t s = state & 3;
  if (digitalRead(pin1)) s |= 4;
  if (digitalRead(pin2)) s |= 8;
  switch (s) {
  case 0: case 5: case 10: case 15:
  break;
  case 1: case 7: case 8: case 14:
  position++; break;
  case 2: case 4: case 11: case 13:
  position--; break;
  case 3: case 12:
  position += 2; break;
  default:
  position -= 2; break;
  }
  state = (s >> 2);
  }
  */

public:
  static void update(Encoder_internal_state_t *arg) {
    // The compiler believes this is just 1 line of code, so
    // it will inline this function into each interrupt
    // handler.  That's a tiny bit faster, but grows the code.
    // Especially when used with ENCODER_OPTIMIZE_INTERRUPTS,
    // the inline nature allows the ISR prologue and epilogue
    // to only save/restore necessary registers, for very nice
    // speed increase.
    asm volatile (
      "ld	r30, X+"		"\n\t"
      "ld	r31, X+"		"\n\t"
      "ld	r25, Z"			"\n\t"  // r25 = pin2 input
      "ld	r31, X+"		"\n\t"	// r31 = pin2 mask
      "and	r25, r31"	"\n\t"
      "breq	L%=DECPOS"		"\n\t"	// if (pin2)
      "L%=INCPOS:"			"\n\t"
      "ldi	r30, lo8(pm(L%=plus1))"	"\n\t"
      "ldi	r31, hi8(pm(L%=plus1))"	"\n\t"
      "rjmp	L%=CONT"       "\n\t"
      "L%=DECPOS:"			"\n\t"
      "ldi	r30, lo8(pm(L%=minus1))"	"\n\t"
      "ldi	r31, hi8(pm(L%=minus1))"	"\n\t"
      "L%=CONT:"			"\n\t"
      "ld	r22, X+"		"\n\t"
      "ld	r23, X+"		"\n\t"
      "ld	r24, X+"		"\n\t"
      "ld	r25, X+"		"\n\t"
      "ijmp"				"\n\t"	// jumps to update_finishup()
      // TODO move this table to another static function,
      // so it doesn't get needlessly duplicated.  Easier
      // said than done, due to linker issues and inlining
      "L%=minus1:"				"\n\t"
      "subi	r22, 1"			"\n\t"
      "sbci	r23, 0"			"\n\t"
      "sbci	r24, 0"			"\n\t"
      "sbci	r25, 0"			"\n\t"
      "rjmp	L%=store"		"\n\t"
      "L%=plus1:"				"\n\t"
      "subi	r22, 255"		"\n\t"
      "sbci	r23, 255"		"\n\t"
      "sbci	r24, 255"		"\n\t"
      "sbci	r25, 255"		"\n\t"
      "L%=store:"				"\n\t"
      "st	-X, r25"		"\n\t"
      "st	-X, r24"		"\n\t"
      "st	-X, r23"		"\n\t"
      "st	-X, r22"		"\n\t"
      "L%=end:"				"\n"
      : : "x" (arg) : "r22", "r23", "r24", "r25", "r30", "r31");
  }


  static uint8_t attach_interrupt(uint8_t pin, Encoder_internal_state_t *state) {
    switch (pin) {
#ifdef CORE_INT0_PIN
    case CORE_INT0_PIN:
      interruptArgs[0] = state;
      attachInterrupt(0, isr0, RISING);
      break;
#endif
#ifdef CORE_INT1_PIN
    case CORE_INT1_PIN:
      interruptArgs[1] = state;
      attachInterrupt(1, isr1, RISING);
      break;
#endif
#ifdef CORE_INT2_PIN
    case CORE_INT2_PIN:
      interruptArgs[2] = state;
      attachInterrupt(2, isr2, RISING);
      break;
#endif
#ifdef CORE_INT3_PIN
    case CORE_INT3_PIN:
      interruptArgs[3] = state;
      attachInterrupt(3, isr3, RISING);
      break;
#endif
#ifdef CORE_INT4_PIN
    case CORE_INT4_PIN:
      interruptArgs[4] = state;
      attachInterrupt(4, isr4, RISING);
      break;
#endif
#ifdef CORE_INT5_PIN
    case CORE_INT5_PIN:
      interruptArgs[5] = state;
      attachInterrupt(5, isr5, RISING);
      break;
#endif
    default:
      return 0;
    }
    return 1;
  }


#if defined(ENCODER_USE_INTERRUPTS) && !defined(ENCODER_OPTIMIZE_INTERRUPTS)
#ifdef CORE_INT0_PIN
  static void isr0(void) { update(interruptArgs[0]); }
#endif
#ifdef CORE_INT1_PIN
  static void isr1(void) { update(interruptArgs[1]); }
#endif
#ifdef CORE_INT2_PIN
  static void isr2(void) { update(interruptArgs[2]); }
#endif
#ifdef CORE_INT3_PIN
  static void isr3(void) { update(interruptArgs[3]); }
#endif
#ifdef CORE_INT4_PIN
  static void isr4(void) { update(interruptArgs[4]); }
#endif
#ifdef CORE_INT5_PIN
  static void isr5(void) { update(interruptArgs[5]); }
#endif
#endif
};

#if defined(ENCODER_USE_INTERRUPTS) && defined(ENCODER_OPTIMIZE_INTERRUPTS)
#if defined(__AVR__)
#if defined(INT0_vect) && CORE_NUM_INTERRUPT > 0
ISR(INT0_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(0)]); }
#endif
#if defined(INT1_vect) && CORE_NUM_INTERRUPT > 1
ISR(INT1_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(1)]); }
#endif
#if defined(INT2_vect) && CORE_NUM_INTERRUPT > 2
ISR(INT2_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(2)]); }
#endif
#if defined(INT3_vect) && CORE_NUM_INTERRUPT > 3
ISR(INT3_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(3)]); }
#endif
#if defined(INT4_vect) && CORE_NUM_INTERRUPT > 4
ISR(INT4_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(4)]); }
#endif
#if defined(INT5_vect) && CORE_NUM_INTERRUPT > 5
ISR(INT5_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(5)]); }
#endif
#if defined(INT6_vect) && CORE_NUM_INTERRUPT > 6
ISR(INT6_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(6)]); }
#endif
#if defined(INT7_vect) && CORE_NUM_INTERRUPT > 7
ISR(INT7_vect) { Encoder::update(Encoder::interruptArgs[SCRAMBLE_INT_ORDER(7)]); }
#endif
#endif // AVR
#endif // ENCODER_OPTIMIZE_INTERRUPTS


#endif