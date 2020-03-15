;

// NOTE: Sometimes it's nice to try to see which one is best to have some not low, must try some combinations
// USE_REG$(global variable name);
// USE_REG$(global variable name, low);

// TUNE_FUNCTION$(function name, pure, no_inline);

///////////////////////////////////////////////////////////////////
// Useful functions for rapid development for AVR microcontrollers.
// 2017 (C) Akshaal, Apache License
///////////////////////////////////////////////////////////////////

#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#define i8 int8_t
#define i16 int16_t
#define i24 __int24
#define i32 int32_t

#define u8 uint8_t
#define u16 uint16_t
#define u24 __uint24
#define u32 uint32_t

#define H(v)                 (1 << (v))
#define L(v)                 (0)

#define AKAT_FORCE_INLINE    __attribute__((always_inline)) inline
#define AKAT_NO_INLINE       __attribute__((noinline))
#define AKAT_UNUSED          __attribute__((unused))
#define AKAT_NO_RETURN       __ATTR_NORETURN__
#define AKAT_CONST           __ATTR_CONST__
#define AKAT_PURE            __ATTR_PURE__
#define AKAT_ERROR(msg)      __attribute__((error(msg))) extern void

#define AKAT_CONCAT(a, b)     a##b
#define AKAT_FORCE_CONCAT(a, b)     AKAT_CONCAT(a, b)

#define AKAT_HOT_CODE        AKAT_FORCE_CONCAT(akat_hot_code__, __COUNTER__): __attribute__((hot, unused));
#define AKAT_COLD_CODE       AKAT_FORCE_CONCAT(akat_cold_code__, __COUNTER__): __attribute__((cold, unused));

#define AKAT_FLUSH_REG_VAR(vvv)     asm volatile ("" : "=r" (vvv));

#define AKAT_COROUTINE_S_START   0
#define AKAT_COROUTINE_S_END     255

static AKAT_FORCE_INLINE AKAT_CONST uint32_t akat_cpu_freq_hz();

register u8 __akat_one__ asm ("r2");

;
;


// To prevent assignment
#define AKAT_ONE  __akat_one__

#define AKAT_TRUE   AKAT_ONE
#define AKAT_FALSE  0

// ============================================================================================================================
// Compatibility

#ifndef TIMSK1
#define TIMSK1 TIMSK
#endif

// ============================================================================================================================
// DELAY

// Delay. Delay function is non atomic!
// Routines are borrowed from avr-lib
__attribute__((error("akat_delay_us must be used with -O compiler flag and constant argument!")))
extern void akat_delay_us_error_nc__();

__attribute__((error("akat_delay_us can't perform such a small delay!")))
extern void akat_delay_us_error_delay__();

__attribute__((error("akat_delay_us can't perform such a long delay!")))
extern void akat_delay_us_error_bdelay__();

static AKAT_FORCE_INLINE void akat_delay_us(uint32_t us) {
    if (!__builtin_constant_p(us)) {
        akat_delay_us_error_nc__ ();
    }

    uint64_t cycles = (uint64_t)us * (uint64_t)akat_cpu_freq_hz () / (uint64_t)1000000L;

    if (cycles / 3 == 0) {
        akat_delay_us_error_delay__ ();
    } else if (cycles / 3 < 256) {
        uint8_t __count = cycles / 3;
        __asm__ volatile (
            "1: dec %0" "\n\t"
            "brne 1b"
            : "=r" (__count)
            : "0" (__count)
        );
    } else if (cycles / 4 > 65535) {
        akat_delay_us_error_bdelay__ ();
    } else {
        uint16_t __count = cycles / 4;
        __asm__ volatile (
            "1: sbiw %0,1" "\n\t"
            "brne 1b"
            : "=w" (__count)
            : "0" (__count)
        );
    }
}

///////////////////////////////////////////////////////////////////
// Useful functions for rapid development for AVR microcontrollers.
// 2019 (C) Akshaal, Apache License
///////////////////////////////////////////////////////////////////

static AKAT_PURE u8 akat_crc_add(u8 const crc, u8 const byte);
static u8 akat_crc_add_bytes(u8 const crc, u8 const *bytes, const u8 size);

///////////////////////////////////////////////////////////////////
// Useful functions for rapid development for AVR microcontrollers.
// 2017 (C) Akshaal, Apache License
///////////////////////////////////////////////////////////////////

#define AKAT_BCD_GET_L(x)     ((x) & 15)
#define AKAT_BCD_GET_H(x)     (((x) / 16))
#define AKAT_BCD(h, l)        (((h) * 16) + (l))

///////////////////////////////////////////////////////////////////
// Useful functions for rapid development for AVR microcontrollers.
// 2017 (C) Akshaal, Apache License
///////////////////////////////////////////////////////////////////

#define AKAT_X_BUTTON_CHECKS 255

typedef void (*akat_x_button_cbk_t)();

typedef enum {AKAT_X_BUTTON_ACTION_NOTHING = 0, AKAT_X_BUTTON_ACTION_KEYPRESS = 1, AKAT_X_BUTTON_ACTION_KEYRELEASE = 2} akat_x_button_action_t;

typedef struct {
    uint8_t awaiting_key_press;
    uint8_t checks_left;
} akat_x_button_state_t;

static AKAT_UNUSED akat_x_button_action_t akat_x_button_handle_pin_state(akat_x_button_state_t * const state, uint8_t const pin_state);

///////////////////////////////////////////////////////////////////
// Useful functions for rapid development for AVR microcontrollers.
// 2017 (C) Akshaal, Apache License
///////////////////////////////////////////////////////////////////

//
//     a
//    ----
//   |    | b
//  f|    |
//   |-g--|
//   |    | c
//  e|    |
//    ----
//     d        : h
//


//                                    hgfedcba
#define AKAT_X_TM1637_C_0           0b00111111
#define AKAT_X_TM1637_C_1           0b00000110
#define AKAT_X_TM1637_C_2           0b01011011
#define AKAT_X_TM1637_C_3           0b01001111
#define AKAT_X_TM1637_C_4           0b01100110
#define AKAT_X_TM1637_C_5           0b01101101
#define AKAT_X_TM1637_C_6           0b01111101
#define AKAT_X_TM1637_C_7           0b00000111
#define AKAT_X_TM1637_C_8           0b01111111
#define AKAT_X_TM1637_C_9           0b01101111

#define AKAT_X_TM1637_C_D           0b00111111
#define AKAT_X_TM1637_C_o           0b01011100
#define AKAT_X_TM1637_C_n           0b01010100
#define AKAT_X_TM1637_C_E           0b01111001

#define AKAT_X_TM1637_COLON_MASK    0b10000000

static AKAT_PURE u8 akat_x_tm1637_encode_digit(u8 const  digit, u8 const  colon);

typedef enum {AKAT_X_TM1637_POS_1 = 0, AKAT_X_TM1637_POS_2 = 1, AKAT_X_TM1637_POS_3 = 2, AKAT_X_TM1637_POS_4 = 3} akat_x_tm1637_pos_t;

///////////////////////////////////////////////////////////////////
// Useful functions for rapid development for AVR microcontrollers.
// 2017 (C) Akshaal, Apache License
///////////////////////////////////////////////////////////////////

typedef enum {
    AKAT_X_TIMESTAMP_LEVEL_DECISECOND,
    AKAT_X_TIMESTAMP_LEVEL_SECOND,
    AKAT_X_TIMESTAMP_LEVEL_MINUTE,
    AKAT_X_TIMESTAMP_LEVEL_HOUR
} akat_x_timestamp_level_t;

///////////////////////////////////////////////////////////////////
// Useful functions for rapid development for AVR microcontrollers.
// 2017 (C) Akshaal, Apache License
///////////////////////////////////////////////////////////////////

typedef struct {
    uint8_t const cs;
    uint8_t const ocr;
    uint8_t const deciseconds;
} akat_x_buzzer_sound_t;

typedef void (*akat_x_buzzer_finish_cbk_t)(u8 interrupted);

///////////////////////////////////////////////////////////////////
// Useful functions for rapid development for AVR microcontrollers.
// 2017 (C) Akshaal, Apache License
///////////////////////////////////////////////////////////////////

static AKAT_PURE AKAT_UNUSED u8 akat_bcd_inc(u8 bcd) {
    if (AKAT_BCD_GET_L(bcd) == 9) {
        bcd += 16 - 9;
    } else {
        bcd++;
    }

    return bcd;
}

;




static AKAT_PURE AKAT_UNUSED u8 akat_bcd_dec(u8 bcd) {
    if (AKAT_BCD_GET_L(bcd)) {
        bcd--;
    } else {
        bcd += -16 + 9;
    }

    return bcd;
}

;




///////////////////////////////////////////////////////////////////
// Useful functions for rapid development for AVR microcontrollers.
// 2019 (C) Akshaal, Apache License
///////////////////////////////////////////////////////////////////

static AKAT_UNUSED AKAT_PURE u8 akat_crc_add(u8 const orig_crc, u8 const orig_byte) {
    u8 crc = orig_crc;
    u8 byte = orig_byte;

    for (u8 j = 0; j < 8; j++) {
        u8 m = (crc ^ byte) & AKAT_ONE;
        crc >>= 1;

        if (m) {
            crc ^= 0x8C;
        }

        byte >>= 1;
    }

    return crc;
}

static AKAT_UNUSED u8 akat_crc_add_bytes(u8 const orig_crc, u8 const *bytes, const u8 size) {
    u8 crc = orig_crc;

    for (u8 i = 0; i < size; i++) {
        crc = akat_crc_add(crc, bytes[i]);
    }

    return crc;
}

///////////////////////////////////////////////////////////////////
// Useful functions for rapid development for AVR microcontrollers.
// 2017 (C) Akshaal, Apache License
///////////////////////////////////////////////////////////////////

static AKAT_UNUSED akat_x_button_action_t akat_x_button_handle_pin_state(akat_x_button_state_t * const state, uint8_t const pin_state) {
    akat_x_button_action_t action = AKAT_X_BUTTON_ACTION_NOTHING;

    if (state->awaiting_key_press) {
        AKAT_HOT_CODE; // Usually we are awaiting a key press

        if (pin_state) {
            AKAT_HOT_CODE; // Often pin is high, it means that key is not pressed, reset counter
            state->checks_left = AKAT_X_BUTTON_CHECKS;
        } else {
            AKAT_COLD_CODE; // Sometimes pin is low, it means that key is pressed

            if (state->checks_left) {
                AKAT_HOT_CODE; // Usually we need to do more checks
                state->checks_left--;
            } else {
                AKAT_COLD_CODE; // Sometimes we find out that key is pressed and stable enough
                // Notify about key-press event
                action = AKAT_X_BUTTON_ACTION_KEYPRESS;
                // Wait for key-release event
                state->awaiting_key_press = 0;
                state->checks_left = AKAT_X_BUTTON_CHECKS;
            }
        }
    } else {
        AKAT_COLD_CODE; // Sometimes we are awaiting for key to be released

        if (pin_state) {
            AKAT_COLD_CODE; // After a long keypress, pin can be high, it means that key is released

            if (state->checks_left) {
                AKAT_HOT_CODE; // Often we have to check again to make sure that state is stable, not bouncing
                state->checks_left--;
            } else {
                AKAT_COLD_CODE; // When we checked enough times, wait for key-press event
                // Notify about key-press event
                action = AKAT_X_BUTTON_ACTION_KEYRELEASE;
                // Wait for key press
                state->awaiting_key_press = AKAT_ONE;
                state->checks_left = AKAT_X_BUTTON_CHECKS;
            }
        } else {
            AKAT_HOT_CODE; // Pin is low, it means that key is pressed
            state->checks_left = AKAT_X_BUTTON_CHECKS;
        }
    }

    return action;
}

///////////////////////////////////////////////////////////////////
// Useful functions for rapid development for AVR microcontrollers.
// 2017 (C) Akshaal, Apache License
///////////////////////////////////////////////////////////////////

#include <avr/pgmspace.h>

static PROGMEM AKAT_UNUSED u8 const akat_x_tm1637_digits_map[] = {
    AKAT_X_TM1637_C_0,
    AKAT_X_TM1637_C_1,
    AKAT_X_TM1637_C_2,
    AKAT_X_TM1637_C_3,
    AKAT_X_TM1637_C_4,
    AKAT_X_TM1637_C_5,
    AKAT_X_TM1637_C_6,
    AKAT_X_TM1637_C_7,
    AKAT_X_TM1637_C_8,
    AKAT_X_TM1637_C_9
};

static AKAT_UNUSED AKAT_PURE u8 akat_x_tm1637_encode_digit(u8 const digit, u8 const colon) {
    return pgm_read_byte(akat_x_tm1637_digits_map + digit) | (colon ? AKAT_X_TM1637_COLON_MASK : 0);
}


#include <avr/io.h>

// 16Mhz, that's external oscillator on Mega 2560.
// This doesn't configure it here, it just tells to our build system
// what we is actually using! Configuration is done using fuses (see flash-avr-fuses).
// Actual value might be a different one, one can actually measure it to provide
// some kind of accuracy (calibration) if needed.
static AKAT_FORCE_INLINE AKAT_CONST uint32_t akat_cpu_freq_hz() {
    return 16000000;
}
;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// PINS

// For safety reasons we set unused pins to read with pull up

typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D3_unused__port_t;

extern D3_unused__port_t const D3_unused__port;

static AKAT_FORCE_INLINE void D3_unused__port__set__impl(u8 state) {
#define set__impl D3_unused__port__set__impl

    if (state) {
        PORTD |= 1 << 3;  //Set PORTD of D3 to 1
    } else {
        PORTD &= ~(1 << 3);  //Set PORTD of D3 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D3_unused__port__is_set__impl() {
#define is_set__impl D3_unused__port__is_set__impl
#define set__impl D3_unused__port__set__impl
    return PORTD & (1 << 3);  //Get value of PORTD for D3
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D3_unused__port__is_set__impl
#define set__impl D3_unused__port__set__impl

D3_unused__port_t const D3_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl D3_unused__port__is_set__impl
#define set__impl D3_unused__port__set__impl


;

#define is_set__impl D3_unused__port__is_set__impl
#define set__impl D3_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D3_unused__ddr_t;

extern D3_unused__ddr_t const D3_unused__ddr;

static AKAT_FORCE_INLINE void D3_unused__ddr__set__impl(u8 state) {
#define set__impl D3_unused__ddr__set__impl

    if (state) {
        DDRD |= 1 << 3;  //Set DDRD of D3 to 1
    } else {
        DDRD &= ~(1 << 3);  //Set DDRD of D3 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D3_unused__ddr__is_set__impl() {
#define is_set__impl D3_unused__ddr__is_set__impl
#define set__impl D3_unused__ddr__set__impl
    return DDRD & (1 << 3);  //Get value of DDRD for D3
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D3_unused__ddr__is_set__impl
#define set__impl D3_unused__ddr__set__impl

D3_unused__ddr_t const D3_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D3_unused__ddr__is_set__impl
#define set__impl D3_unused__ddr__set__impl


;

#define is_set__impl D3_unused__ddr__is_set__impl
#define set__impl D3_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D3_unused__pin_t;

extern D3_unused__pin_t const D3_unused__pin;

static AKAT_FORCE_INLINE void D3_unused__pin__set__impl(u8 state) {
#define set__impl D3_unused__pin__set__impl

    if (state) {
        PIND |= 1 << 3;  //Set PIND of D3 to 1
    } else {
        PIND &= ~(1 << 3);  //Set PIND of D3 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D3_unused__pin__is_set__impl() {
#define is_set__impl D3_unused__pin__is_set__impl
#define set__impl D3_unused__pin__set__impl
    return PIND & (1 << 3);  //Get value of PIND for D3
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D3_unused__pin__is_set__impl
#define set__impl D3_unused__pin__set__impl

D3_unused__pin_t const D3_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D3_unused__pin__is_set__impl
#define set__impl D3_unused__pin__set__impl


;

#define is_set__impl D3_unused__pin__is_set__impl
#define set__impl D3_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void D3_unused__init() {
    D3_unused__ddr.set(0);
    D3_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} D3_unused_t;

extern D3_unused_t const D3_unused;

static AKAT_FORCE_INLINE u8 D3_unused__is_set__impl() {
#define is_set__impl D3_unused__is_set__impl
    return D3_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl D3_unused__is_set__impl

D3_unused_t const D3_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl D3_unused__is_set__impl


;

#define is_set__impl D3_unused__is_set__impl




#undef is_set__impl
;



;
; // 1 PD3
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D4_unused__port_t;

extern D4_unused__port_t const D4_unused__port;

static AKAT_FORCE_INLINE void D4_unused__port__set__impl(u8 state) {
#define set__impl D4_unused__port__set__impl

    if (state) {
        PORTD |= 1 << 4;  //Set PORTD of D4 to 1
    } else {
        PORTD &= ~(1 << 4);  //Set PORTD of D4 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D4_unused__port__is_set__impl() {
#define is_set__impl D4_unused__port__is_set__impl
#define set__impl D4_unused__port__set__impl
    return PORTD & (1 << 4);  //Get value of PORTD for D4
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D4_unused__port__is_set__impl
#define set__impl D4_unused__port__set__impl

D4_unused__port_t const D4_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl D4_unused__port__is_set__impl
#define set__impl D4_unused__port__set__impl


;

#define is_set__impl D4_unused__port__is_set__impl
#define set__impl D4_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D4_unused__ddr_t;

extern D4_unused__ddr_t const D4_unused__ddr;

static AKAT_FORCE_INLINE void D4_unused__ddr__set__impl(u8 state) {
#define set__impl D4_unused__ddr__set__impl

    if (state) {
        DDRD |= 1 << 4;  //Set DDRD of D4 to 1
    } else {
        DDRD &= ~(1 << 4);  //Set DDRD of D4 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D4_unused__ddr__is_set__impl() {
#define is_set__impl D4_unused__ddr__is_set__impl
#define set__impl D4_unused__ddr__set__impl
    return DDRD & (1 << 4);  //Get value of DDRD for D4
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D4_unused__ddr__is_set__impl
#define set__impl D4_unused__ddr__set__impl

D4_unused__ddr_t const D4_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D4_unused__ddr__is_set__impl
#define set__impl D4_unused__ddr__set__impl


;

#define is_set__impl D4_unused__ddr__is_set__impl
#define set__impl D4_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D4_unused__pin_t;

extern D4_unused__pin_t const D4_unused__pin;

static AKAT_FORCE_INLINE void D4_unused__pin__set__impl(u8 state) {
#define set__impl D4_unused__pin__set__impl

    if (state) {
        PIND |= 1 << 4;  //Set PIND of D4 to 1
    } else {
        PIND &= ~(1 << 4);  //Set PIND of D4 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D4_unused__pin__is_set__impl() {
#define is_set__impl D4_unused__pin__is_set__impl
#define set__impl D4_unused__pin__set__impl
    return PIND & (1 << 4);  //Get value of PIND for D4
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D4_unused__pin__is_set__impl
#define set__impl D4_unused__pin__set__impl

D4_unused__pin_t const D4_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D4_unused__pin__is_set__impl
#define set__impl D4_unused__pin__set__impl


;

#define is_set__impl D4_unused__pin__is_set__impl
#define set__impl D4_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void D4_unused__init() {
    D4_unused__ddr.set(0);
    D4_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} D4_unused_t;

extern D4_unused_t const D4_unused;

static AKAT_FORCE_INLINE u8 D4_unused__is_set__impl() {
#define is_set__impl D4_unused__is_set__impl
    return D4_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl D4_unused__is_set__impl

D4_unused_t const D4_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl D4_unused__is_set__impl


;

#define is_set__impl D4_unused__is_set__impl




#undef is_set__impl
;



;
; // 2 PD4
// .................. 3 GND
// .................. 4 VCC
// .................. 5 GND
// .................. 6 VCC
// .................. 7 XT1
// .................. 8 XT2
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D5_unused__port_t;

extern D5_unused__port_t const D5_unused__port;

static AKAT_FORCE_INLINE void D5_unused__port__set__impl(u8 state) {
#define set__impl D5_unused__port__set__impl

    if (state) {
        PORTD |= 1 << 5;  //Set PORTD of D5 to 1
    } else {
        PORTD &= ~(1 << 5);  //Set PORTD of D5 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D5_unused__port__is_set__impl() {
#define is_set__impl D5_unused__port__is_set__impl
#define set__impl D5_unused__port__set__impl
    return PORTD & (1 << 5);  //Get value of PORTD for D5
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D5_unused__port__is_set__impl
#define set__impl D5_unused__port__set__impl

D5_unused__port_t const D5_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl D5_unused__port__is_set__impl
#define set__impl D5_unused__port__set__impl


;

#define is_set__impl D5_unused__port__is_set__impl
#define set__impl D5_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D5_unused__ddr_t;

extern D5_unused__ddr_t const D5_unused__ddr;

static AKAT_FORCE_INLINE void D5_unused__ddr__set__impl(u8 state) {
#define set__impl D5_unused__ddr__set__impl

    if (state) {
        DDRD |= 1 << 5;  //Set DDRD of D5 to 1
    } else {
        DDRD &= ~(1 << 5);  //Set DDRD of D5 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D5_unused__ddr__is_set__impl() {
#define is_set__impl D5_unused__ddr__is_set__impl
#define set__impl D5_unused__ddr__set__impl
    return DDRD & (1 << 5);  //Get value of DDRD for D5
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D5_unused__ddr__is_set__impl
#define set__impl D5_unused__ddr__set__impl

D5_unused__ddr_t const D5_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D5_unused__ddr__is_set__impl
#define set__impl D5_unused__ddr__set__impl


;

#define is_set__impl D5_unused__ddr__is_set__impl
#define set__impl D5_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D5_unused__pin_t;

extern D5_unused__pin_t const D5_unused__pin;

static AKAT_FORCE_INLINE void D5_unused__pin__set__impl(u8 state) {
#define set__impl D5_unused__pin__set__impl

    if (state) {
        PIND |= 1 << 5;  //Set PIND of D5 to 1
    } else {
        PIND &= ~(1 << 5);  //Set PIND of D5 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D5_unused__pin__is_set__impl() {
#define is_set__impl D5_unused__pin__is_set__impl
#define set__impl D5_unused__pin__set__impl
    return PIND & (1 << 5);  //Get value of PIND for D5
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D5_unused__pin__is_set__impl
#define set__impl D5_unused__pin__set__impl

D5_unused__pin_t const D5_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D5_unused__pin__is_set__impl
#define set__impl D5_unused__pin__set__impl


;

#define is_set__impl D5_unused__pin__is_set__impl
#define set__impl D5_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void D5_unused__init() {
    D5_unused__ddr.set(0);
    D5_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} D5_unused_t;

extern D5_unused_t const D5_unused;

static AKAT_FORCE_INLINE u8 D5_unused__is_set__impl() {
#define is_set__impl D5_unused__is_set__impl
    return D5_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl D5_unused__is_set__impl

D5_unused_t const D5_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl D5_unused__is_set__impl


;

#define is_set__impl D5_unused__is_set__impl




#undef is_set__impl
;



;
; // 9 PD5
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D6_unused__port_t;

extern D6_unused__port_t const D6_unused__port;

static AKAT_FORCE_INLINE void D6_unused__port__set__impl(u8 state) {
#define set__impl D6_unused__port__set__impl

    if (state) {
        PORTD |= 1 << 6;  //Set PORTD of D6 to 1
    } else {
        PORTD &= ~(1 << 6);  //Set PORTD of D6 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D6_unused__port__is_set__impl() {
#define is_set__impl D6_unused__port__is_set__impl
#define set__impl D6_unused__port__set__impl
    return PORTD & (1 << 6);  //Get value of PORTD for D6
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D6_unused__port__is_set__impl
#define set__impl D6_unused__port__set__impl

D6_unused__port_t const D6_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl D6_unused__port__is_set__impl
#define set__impl D6_unused__port__set__impl


;

#define is_set__impl D6_unused__port__is_set__impl
#define set__impl D6_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D6_unused__ddr_t;

extern D6_unused__ddr_t const D6_unused__ddr;

static AKAT_FORCE_INLINE void D6_unused__ddr__set__impl(u8 state) {
#define set__impl D6_unused__ddr__set__impl

    if (state) {
        DDRD |= 1 << 6;  //Set DDRD of D6 to 1
    } else {
        DDRD &= ~(1 << 6);  //Set DDRD of D6 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D6_unused__ddr__is_set__impl() {
#define is_set__impl D6_unused__ddr__is_set__impl
#define set__impl D6_unused__ddr__set__impl
    return DDRD & (1 << 6);  //Get value of DDRD for D6
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D6_unused__ddr__is_set__impl
#define set__impl D6_unused__ddr__set__impl

D6_unused__ddr_t const D6_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D6_unused__ddr__is_set__impl
#define set__impl D6_unused__ddr__set__impl


;

#define is_set__impl D6_unused__ddr__is_set__impl
#define set__impl D6_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D6_unused__pin_t;

extern D6_unused__pin_t const D6_unused__pin;

static AKAT_FORCE_INLINE void D6_unused__pin__set__impl(u8 state) {
#define set__impl D6_unused__pin__set__impl

    if (state) {
        PIND |= 1 << 6;  //Set PIND of D6 to 1
    } else {
        PIND &= ~(1 << 6);  //Set PIND of D6 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D6_unused__pin__is_set__impl() {
#define is_set__impl D6_unused__pin__is_set__impl
#define set__impl D6_unused__pin__set__impl
    return PIND & (1 << 6);  //Get value of PIND for D6
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D6_unused__pin__is_set__impl
#define set__impl D6_unused__pin__set__impl

D6_unused__pin_t const D6_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D6_unused__pin__is_set__impl
#define set__impl D6_unused__pin__set__impl


;

#define is_set__impl D6_unused__pin__is_set__impl
#define set__impl D6_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void D6_unused__init() {
    D6_unused__ddr.set(0);
    D6_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} D6_unused_t;

extern D6_unused_t const D6_unused;

static AKAT_FORCE_INLINE u8 D6_unused__is_set__impl() {
#define is_set__impl D6_unused__is_set__impl
    return D6_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl D6_unused__is_set__impl

D6_unused_t const D6_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl D6_unused__is_set__impl


;

#define is_set__impl D6_unused__is_set__impl




#undef is_set__impl
;



;
; // 10 PD6
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D7_unused__port_t;

extern D7_unused__port_t const D7_unused__port;

static AKAT_FORCE_INLINE void D7_unused__port__set__impl(u8 state) {
#define set__impl D7_unused__port__set__impl

    if (state) {
        PORTD |= 1 << 7;  //Set PORTD of D7 to 1
    } else {
        PORTD &= ~(1 << 7);  //Set PORTD of D7 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D7_unused__port__is_set__impl() {
#define is_set__impl D7_unused__port__is_set__impl
#define set__impl D7_unused__port__set__impl
    return PORTD & (1 << 7);  //Get value of PORTD for D7
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D7_unused__port__is_set__impl
#define set__impl D7_unused__port__set__impl

D7_unused__port_t const D7_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl D7_unused__port__is_set__impl
#define set__impl D7_unused__port__set__impl


;

#define is_set__impl D7_unused__port__is_set__impl
#define set__impl D7_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D7_unused__ddr_t;

extern D7_unused__ddr_t const D7_unused__ddr;

static AKAT_FORCE_INLINE void D7_unused__ddr__set__impl(u8 state) {
#define set__impl D7_unused__ddr__set__impl

    if (state) {
        DDRD |= 1 << 7;  //Set DDRD of D7 to 1
    } else {
        DDRD &= ~(1 << 7);  //Set DDRD of D7 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D7_unused__ddr__is_set__impl() {
#define is_set__impl D7_unused__ddr__is_set__impl
#define set__impl D7_unused__ddr__set__impl
    return DDRD & (1 << 7);  //Get value of DDRD for D7
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D7_unused__ddr__is_set__impl
#define set__impl D7_unused__ddr__set__impl

D7_unused__ddr_t const D7_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D7_unused__ddr__is_set__impl
#define set__impl D7_unused__ddr__set__impl


;

#define is_set__impl D7_unused__ddr__is_set__impl
#define set__impl D7_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D7_unused__pin_t;

extern D7_unused__pin_t const D7_unused__pin;

static AKAT_FORCE_INLINE void D7_unused__pin__set__impl(u8 state) {
#define set__impl D7_unused__pin__set__impl

    if (state) {
        PIND |= 1 << 7;  //Set PIND of D7 to 1
    } else {
        PIND &= ~(1 << 7);  //Set PIND of D7 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D7_unused__pin__is_set__impl() {
#define is_set__impl D7_unused__pin__is_set__impl
#define set__impl D7_unused__pin__set__impl
    return PIND & (1 << 7);  //Get value of PIND for D7
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D7_unused__pin__is_set__impl
#define set__impl D7_unused__pin__set__impl

D7_unused__pin_t const D7_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D7_unused__pin__is_set__impl
#define set__impl D7_unused__pin__set__impl


;

#define is_set__impl D7_unused__pin__is_set__impl
#define set__impl D7_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void D7_unused__init() {
    D7_unused__ddr.set(0);
    D7_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} D7_unused_t;

extern D7_unused_t const D7_unused;

static AKAT_FORCE_INLINE u8 D7_unused__is_set__impl() {
#define is_set__impl D7_unused__is_set__impl
    return D7_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl D7_unused__is_set__impl

D7_unused_t const D7_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl D7_unused__is_set__impl


;

#define is_set__impl D7_unused__is_set__impl




#undef is_set__impl
;



;
; // 11 PD7
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B0_unused__port_t;

extern B0_unused__port_t const B0_unused__port;

static AKAT_FORCE_INLINE void B0_unused__port__set__impl(u8 state) {
#define set__impl B0_unused__port__set__impl

    if (state) {
        PORTB |= 1 << 0;  //Set PORTB of B0 to 1
    } else {
        PORTB &= ~(1 << 0);  //Set PORTB of B0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B0_unused__port__is_set__impl() {
#define is_set__impl B0_unused__port__is_set__impl
#define set__impl B0_unused__port__set__impl
    return PORTB & (1 << 0);  //Get value of PORTB for B0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B0_unused__port__is_set__impl
#define set__impl B0_unused__port__set__impl

B0_unused__port_t const B0_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl B0_unused__port__is_set__impl
#define set__impl B0_unused__port__set__impl


;

#define is_set__impl B0_unused__port__is_set__impl
#define set__impl B0_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B0_unused__ddr_t;

extern B0_unused__ddr_t const B0_unused__ddr;

static AKAT_FORCE_INLINE void B0_unused__ddr__set__impl(u8 state) {
#define set__impl B0_unused__ddr__set__impl

    if (state) {
        DDRB |= 1 << 0;  //Set DDRB of B0 to 1
    } else {
        DDRB &= ~(1 << 0);  //Set DDRB of B0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B0_unused__ddr__is_set__impl() {
#define is_set__impl B0_unused__ddr__is_set__impl
#define set__impl B0_unused__ddr__set__impl
    return DDRB & (1 << 0);  //Get value of DDRB for B0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B0_unused__ddr__is_set__impl
#define set__impl B0_unused__ddr__set__impl

B0_unused__ddr_t const B0_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl B0_unused__ddr__is_set__impl
#define set__impl B0_unused__ddr__set__impl


;

#define is_set__impl B0_unused__ddr__is_set__impl
#define set__impl B0_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B0_unused__pin_t;

extern B0_unused__pin_t const B0_unused__pin;

static AKAT_FORCE_INLINE void B0_unused__pin__set__impl(u8 state) {
#define set__impl B0_unused__pin__set__impl

    if (state) {
        PINB |= 1 << 0;  //Set PINB of B0 to 1
    } else {
        PINB &= ~(1 << 0);  //Set PINB of B0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B0_unused__pin__is_set__impl() {
#define is_set__impl B0_unused__pin__is_set__impl
#define set__impl B0_unused__pin__set__impl
    return PINB & (1 << 0);  //Get value of PINB for B0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B0_unused__pin__is_set__impl
#define set__impl B0_unused__pin__set__impl

B0_unused__pin_t const B0_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl B0_unused__pin__is_set__impl
#define set__impl B0_unused__pin__set__impl


;

#define is_set__impl B0_unused__pin__is_set__impl
#define set__impl B0_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void B0_unused__init() {
    B0_unused__ddr.set(0);
    B0_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} B0_unused_t;

extern B0_unused_t const B0_unused;

static AKAT_FORCE_INLINE u8 B0_unused__is_set__impl() {
#define is_set__impl B0_unused__is_set__impl
    return B0_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl B0_unused__is_set__impl

B0_unused_t const B0_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl B0_unused__is_set__impl


;

#define is_set__impl B0_unused__is_set__impl




#undef is_set__impl
;



;
; // 12 PB0
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B1_unused__port_t;

extern B1_unused__port_t const B1_unused__port;

static AKAT_FORCE_INLINE void B1_unused__port__set__impl(u8 state) {
#define set__impl B1_unused__port__set__impl

    if (state) {
        PORTB |= 1 << 1;  //Set PORTB of B1 to 1
    } else {
        PORTB &= ~(1 << 1);  //Set PORTB of B1 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B1_unused__port__is_set__impl() {
#define is_set__impl B1_unused__port__is_set__impl
#define set__impl B1_unused__port__set__impl
    return PORTB & (1 << 1);  //Get value of PORTB for B1
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B1_unused__port__is_set__impl
#define set__impl B1_unused__port__set__impl

B1_unused__port_t const B1_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl B1_unused__port__is_set__impl
#define set__impl B1_unused__port__set__impl


;

#define is_set__impl B1_unused__port__is_set__impl
#define set__impl B1_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B1_unused__ddr_t;

extern B1_unused__ddr_t const B1_unused__ddr;

static AKAT_FORCE_INLINE void B1_unused__ddr__set__impl(u8 state) {
#define set__impl B1_unused__ddr__set__impl

    if (state) {
        DDRB |= 1 << 1;  //Set DDRB of B1 to 1
    } else {
        DDRB &= ~(1 << 1);  //Set DDRB of B1 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B1_unused__ddr__is_set__impl() {
#define is_set__impl B1_unused__ddr__is_set__impl
#define set__impl B1_unused__ddr__set__impl
    return DDRB & (1 << 1);  //Get value of DDRB for B1
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B1_unused__ddr__is_set__impl
#define set__impl B1_unused__ddr__set__impl

B1_unused__ddr_t const B1_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl B1_unused__ddr__is_set__impl
#define set__impl B1_unused__ddr__set__impl


;

#define is_set__impl B1_unused__ddr__is_set__impl
#define set__impl B1_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B1_unused__pin_t;

extern B1_unused__pin_t const B1_unused__pin;

static AKAT_FORCE_INLINE void B1_unused__pin__set__impl(u8 state) {
#define set__impl B1_unused__pin__set__impl

    if (state) {
        PINB |= 1 << 1;  //Set PINB of B1 to 1
    } else {
        PINB &= ~(1 << 1);  //Set PINB of B1 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B1_unused__pin__is_set__impl() {
#define is_set__impl B1_unused__pin__is_set__impl
#define set__impl B1_unused__pin__set__impl
    return PINB & (1 << 1);  //Get value of PINB for B1
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B1_unused__pin__is_set__impl
#define set__impl B1_unused__pin__set__impl

B1_unused__pin_t const B1_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl B1_unused__pin__is_set__impl
#define set__impl B1_unused__pin__set__impl


;

#define is_set__impl B1_unused__pin__is_set__impl
#define set__impl B1_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void B1_unused__init() {
    B1_unused__ddr.set(0);
    B1_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} B1_unused_t;

extern B1_unused_t const B1_unused;

static AKAT_FORCE_INLINE u8 B1_unused__is_set__impl() {
#define is_set__impl B1_unused__is_set__impl
    return B1_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl B1_unused__is_set__impl

B1_unused_t const B1_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl B1_unused__is_set__impl


;

#define is_set__impl B1_unused__is_set__impl




#undef is_set__impl
;



;
; // 13 PB1
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B2_unused__port_t;

extern B2_unused__port_t const B2_unused__port;

static AKAT_FORCE_INLINE void B2_unused__port__set__impl(u8 state) {
#define set__impl B2_unused__port__set__impl

    if (state) {
        PORTB |= 1 << 2;  //Set PORTB of B2 to 1
    } else {
        PORTB &= ~(1 << 2);  //Set PORTB of B2 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B2_unused__port__is_set__impl() {
#define is_set__impl B2_unused__port__is_set__impl
#define set__impl B2_unused__port__set__impl
    return PORTB & (1 << 2);  //Get value of PORTB for B2
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B2_unused__port__is_set__impl
#define set__impl B2_unused__port__set__impl

B2_unused__port_t const B2_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl B2_unused__port__is_set__impl
#define set__impl B2_unused__port__set__impl


;

#define is_set__impl B2_unused__port__is_set__impl
#define set__impl B2_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B2_unused__ddr_t;

extern B2_unused__ddr_t const B2_unused__ddr;

static AKAT_FORCE_INLINE void B2_unused__ddr__set__impl(u8 state) {
#define set__impl B2_unused__ddr__set__impl

    if (state) {
        DDRB |= 1 << 2;  //Set DDRB of B2 to 1
    } else {
        DDRB &= ~(1 << 2);  //Set DDRB of B2 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B2_unused__ddr__is_set__impl() {
#define is_set__impl B2_unused__ddr__is_set__impl
#define set__impl B2_unused__ddr__set__impl
    return DDRB & (1 << 2);  //Get value of DDRB for B2
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B2_unused__ddr__is_set__impl
#define set__impl B2_unused__ddr__set__impl

B2_unused__ddr_t const B2_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl B2_unused__ddr__is_set__impl
#define set__impl B2_unused__ddr__set__impl


;

#define is_set__impl B2_unused__ddr__is_set__impl
#define set__impl B2_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B2_unused__pin_t;

extern B2_unused__pin_t const B2_unused__pin;

static AKAT_FORCE_INLINE void B2_unused__pin__set__impl(u8 state) {
#define set__impl B2_unused__pin__set__impl

    if (state) {
        PINB |= 1 << 2;  //Set PINB of B2 to 1
    } else {
        PINB &= ~(1 << 2);  //Set PINB of B2 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B2_unused__pin__is_set__impl() {
#define is_set__impl B2_unused__pin__is_set__impl
#define set__impl B2_unused__pin__set__impl
    return PINB & (1 << 2);  //Get value of PINB for B2
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B2_unused__pin__is_set__impl
#define set__impl B2_unused__pin__set__impl

B2_unused__pin_t const B2_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl B2_unused__pin__is_set__impl
#define set__impl B2_unused__pin__set__impl


;

#define is_set__impl B2_unused__pin__is_set__impl
#define set__impl B2_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void B2_unused__init() {
    B2_unused__ddr.set(0);
    B2_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} B2_unused_t;

extern B2_unused_t const B2_unused;

static AKAT_FORCE_INLINE u8 B2_unused__is_set__impl() {
#define is_set__impl B2_unused__is_set__impl
    return B2_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl B2_unused__is_set__impl

B2_unused_t const B2_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl B2_unused__is_set__impl


;

#define is_set__impl B2_unused__is_set__impl




#undef is_set__impl
;



;
; // 14 PB2
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B3_unused__port_t;

extern B3_unused__port_t const B3_unused__port;

static AKAT_FORCE_INLINE void B3_unused__port__set__impl(u8 state) {
#define set__impl B3_unused__port__set__impl

    if (state) {
        PORTB |= 1 << 3;  //Set PORTB of B3 to 1
    } else {
        PORTB &= ~(1 << 3);  //Set PORTB of B3 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B3_unused__port__is_set__impl() {
#define is_set__impl B3_unused__port__is_set__impl
#define set__impl B3_unused__port__set__impl
    return PORTB & (1 << 3);  //Get value of PORTB for B3
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B3_unused__port__is_set__impl
#define set__impl B3_unused__port__set__impl

B3_unused__port_t const B3_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl B3_unused__port__is_set__impl
#define set__impl B3_unused__port__set__impl


;

#define is_set__impl B3_unused__port__is_set__impl
#define set__impl B3_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B3_unused__ddr_t;

extern B3_unused__ddr_t const B3_unused__ddr;

static AKAT_FORCE_INLINE void B3_unused__ddr__set__impl(u8 state) {
#define set__impl B3_unused__ddr__set__impl

    if (state) {
        DDRB |= 1 << 3;  //Set DDRB of B3 to 1
    } else {
        DDRB &= ~(1 << 3);  //Set DDRB of B3 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B3_unused__ddr__is_set__impl() {
#define is_set__impl B3_unused__ddr__is_set__impl
#define set__impl B3_unused__ddr__set__impl
    return DDRB & (1 << 3);  //Get value of DDRB for B3
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B3_unused__ddr__is_set__impl
#define set__impl B3_unused__ddr__set__impl

B3_unused__ddr_t const B3_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl B3_unused__ddr__is_set__impl
#define set__impl B3_unused__ddr__set__impl


;

#define is_set__impl B3_unused__ddr__is_set__impl
#define set__impl B3_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B3_unused__pin_t;

extern B3_unused__pin_t const B3_unused__pin;

static AKAT_FORCE_INLINE void B3_unused__pin__set__impl(u8 state) {
#define set__impl B3_unused__pin__set__impl

    if (state) {
        PINB |= 1 << 3;  //Set PINB of B3 to 1
    } else {
        PINB &= ~(1 << 3);  //Set PINB of B3 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B3_unused__pin__is_set__impl() {
#define is_set__impl B3_unused__pin__is_set__impl
#define set__impl B3_unused__pin__set__impl
    return PINB & (1 << 3);  //Get value of PINB for B3
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B3_unused__pin__is_set__impl
#define set__impl B3_unused__pin__set__impl

B3_unused__pin_t const B3_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl B3_unused__pin__is_set__impl
#define set__impl B3_unused__pin__set__impl


;

#define is_set__impl B3_unused__pin__is_set__impl
#define set__impl B3_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void B3_unused__init() {
    B3_unused__ddr.set(0);
    B3_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} B3_unused_t;

extern B3_unused_t const B3_unused;

static AKAT_FORCE_INLINE u8 B3_unused__is_set__impl() {
#define is_set__impl B3_unused__is_set__impl
    return B3_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl B3_unused__is_set__impl

B3_unused_t const B3_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl B3_unused__is_set__impl


;

#define is_set__impl B3_unused__is_set__impl




#undef is_set__impl
;



;
; // 15 PB3, MOSI
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B4_unused__port_t;

extern B4_unused__port_t const B4_unused__port;

static AKAT_FORCE_INLINE void B4_unused__port__set__impl(u8 state) {
#define set__impl B4_unused__port__set__impl

    if (state) {
        PORTB |= 1 << 4;  //Set PORTB of B4 to 1
    } else {
        PORTB &= ~(1 << 4);  //Set PORTB of B4 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B4_unused__port__is_set__impl() {
#define is_set__impl B4_unused__port__is_set__impl
#define set__impl B4_unused__port__set__impl
    return PORTB & (1 << 4);  //Get value of PORTB for B4
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B4_unused__port__is_set__impl
#define set__impl B4_unused__port__set__impl

B4_unused__port_t const B4_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl B4_unused__port__is_set__impl
#define set__impl B4_unused__port__set__impl


;

#define is_set__impl B4_unused__port__is_set__impl
#define set__impl B4_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B4_unused__ddr_t;

extern B4_unused__ddr_t const B4_unused__ddr;

static AKAT_FORCE_INLINE void B4_unused__ddr__set__impl(u8 state) {
#define set__impl B4_unused__ddr__set__impl

    if (state) {
        DDRB |= 1 << 4;  //Set DDRB of B4 to 1
    } else {
        DDRB &= ~(1 << 4);  //Set DDRB of B4 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B4_unused__ddr__is_set__impl() {
#define is_set__impl B4_unused__ddr__is_set__impl
#define set__impl B4_unused__ddr__set__impl
    return DDRB & (1 << 4);  //Get value of DDRB for B4
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B4_unused__ddr__is_set__impl
#define set__impl B4_unused__ddr__set__impl

B4_unused__ddr_t const B4_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl B4_unused__ddr__is_set__impl
#define set__impl B4_unused__ddr__set__impl


;

#define is_set__impl B4_unused__ddr__is_set__impl
#define set__impl B4_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} B4_unused__pin_t;

extern B4_unused__pin_t const B4_unused__pin;

static AKAT_FORCE_INLINE void B4_unused__pin__set__impl(u8 state) {
#define set__impl B4_unused__pin__set__impl

    if (state) {
        PINB |= 1 << 4;  //Set PINB of B4 to 1
    } else {
        PINB &= ~(1 << 4);  //Set PINB of B4 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 B4_unused__pin__is_set__impl() {
#define is_set__impl B4_unused__pin__is_set__impl
#define set__impl B4_unused__pin__set__impl
    return PINB & (1 << 4);  //Get value of PINB for B4
#undef is_set__impl
#undef set__impl
}
#define is_set__impl B4_unused__pin__is_set__impl
#define set__impl B4_unused__pin__set__impl

B4_unused__pin_t const B4_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl B4_unused__pin__is_set__impl
#define set__impl B4_unused__pin__set__impl


;

#define is_set__impl B4_unused__pin__is_set__impl
#define set__impl B4_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void B4_unused__init() {
    B4_unused__ddr.set(0);
    B4_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} B4_unused_t;

extern B4_unused_t const B4_unused;

static AKAT_FORCE_INLINE u8 B4_unused__is_set__impl() {
#define is_set__impl B4_unused__is_set__impl
    return B4_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl B4_unused__is_set__impl

B4_unused_t const B4_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl B4_unused__is_set__impl


;

#define is_set__impl B4_unused__is_set__impl




#undef is_set__impl
;



;
; // 16 PB4, MISO
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D2_unused__port_t;

extern D2_unused__port_t const D2_unused__port;

static AKAT_FORCE_INLINE void D2_unused__port__set__impl(u8 state) {
#define set__impl D2_unused__port__set__impl

    if (state) {
        PORTD |= 1 << 2;  //Set PORTD of D2 to 1
    } else {
        PORTD &= ~(1 << 2);  //Set PORTD of D2 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D2_unused__port__is_set__impl() {
#define is_set__impl D2_unused__port__is_set__impl
#define set__impl D2_unused__port__set__impl
    return PORTD & (1 << 2);  //Get value of PORTD for D2
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D2_unused__port__is_set__impl
#define set__impl D2_unused__port__set__impl

D2_unused__port_t const D2_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl D2_unused__port__is_set__impl
#define set__impl D2_unused__port__set__impl


;

#define is_set__impl D2_unused__port__is_set__impl
#define set__impl D2_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D2_unused__ddr_t;

extern D2_unused__ddr_t const D2_unused__ddr;

static AKAT_FORCE_INLINE void D2_unused__ddr__set__impl(u8 state) {
#define set__impl D2_unused__ddr__set__impl

    if (state) {
        DDRD |= 1 << 2;  //Set DDRD of D2 to 1
    } else {
        DDRD &= ~(1 << 2);  //Set DDRD of D2 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D2_unused__ddr__is_set__impl() {
#define is_set__impl D2_unused__ddr__is_set__impl
#define set__impl D2_unused__ddr__set__impl
    return DDRD & (1 << 2);  //Get value of DDRD for D2
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D2_unused__ddr__is_set__impl
#define set__impl D2_unused__ddr__set__impl

D2_unused__ddr_t const D2_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D2_unused__ddr__is_set__impl
#define set__impl D2_unused__ddr__set__impl


;

#define is_set__impl D2_unused__ddr__is_set__impl
#define set__impl D2_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D2_unused__pin_t;

extern D2_unused__pin_t const D2_unused__pin;

static AKAT_FORCE_INLINE void D2_unused__pin__set__impl(u8 state) {
#define set__impl D2_unused__pin__set__impl

    if (state) {
        PIND |= 1 << 2;  //Set PIND of D2 to 1
    } else {
        PIND &= ~(1 << 2);  //Set PIND of D2 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D2_unused__pin__is_set__impl() {
#define is_set__impl D2_unused__pin__is_set__impl
#define set__impl D2_unused__pin__set__impl
    return PIND & (1 << 2);  //Get value of PIND for D2
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D2_unused__pin__is_set__impl
#define set__impl D2_unused__pin__set__impl

D2_unused__pin_t const D2_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D2_unused__pin__is_set__impl
#define set__impl D2_unused__pin__set__impl


;

#define is_set__impl D2_unused__pin__is_set__impl
#define set__impl D2_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void D2_unused__init() {
    D2_unused__ddr.set(0);
    D2_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} D2_unused_t;

extern D2_unused_t const D2_unused;

static AKAT_FORCE_INLINE u8 D2_unused__is_set__impl() {
#define is_set__impl D2_unused__is_set__impl
    return D2_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl D2_unused__is_set__impl

D2_unused_t const D2_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl D2_unused__is_set__impl


;

#define is_set__impl D2_unused__is_set__impl




#undef is_set__impl
;



;
; // 32 PD2
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D1_unused__port_t;

extern D1_unused__port_t const D1_unused__port;

static AKAT_FORCE_INLINE void D1_unused__port__set__impl(u8 state) {
#define set__impl D1_unused__port__set__impl

    if (state) {
        PORTD |= 1 << 1;  //Set PORTD of D1 to 1
    } else {
        PORTD &= ~(1 << 1);  //Set PORTD of D1 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D1_unused__port__is_set__impl() {
#define is_set__impl D1_unused__port__is_set__impl
#define set__impl D1_unused__port__set__impl
    return PORTD & (1 << 1);  //Get value of PORTD for D1
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D1_unused__port__is_set__impl
#define set__impl D1_unused__port__set__impl

D1_unused__port_t const D1_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl D1_unused__port__is_set__impl
#define set__impl D1_unused__port__set__impl


;

#define is_set__impl D1_unused__port__is_set__impl
#define set__impl D1_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D1_unused__ddr_t;

extern D1_unused__ddr_t const D1_unused__ddr;

static AKAT_FORCE_INLINE void D1_unused__ddr__set__impl(u8 state) {
#define set__impl D1_unused__ddr__set__impl

    if (state) {
        DDRD |= 1 << 1;  //Set DDRD of D1 to 1
    } else {
        DDRD &= ~(1 << 1);  //Set DDRD of D1 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D1_unused__ddr__is_set__impl() {
#define is_set__impl D1_unused__ddr__is_set__impl
#define set__impl D1_unused__ddr__set__impl
    return DDRD & (1 << 1);  //Get value of DDRD for D1
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D1_unused__ddr__is_set__impl
#define set__impl D1_unused__ddr__set__impl

D1_unused__ddr_t const D1_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D1_unused__ddr__is_set__impl
#define set__impl D1_unused__ddr__set__impl


;

#define is_set__impl D1_unused__ddr__is_set__impl
#define set__impl D1_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D1_unused__pin_t;

extern D1_unused__pin_t const D1_unused__pin;

static AKAT_FORCE_INLINE void D1_unused__pin__set__impl(u8 state) {
#define set__impl D1_unused__pin__set__impl

    if (state) {
        PIND |= 1 << 1;  //Set PIND of D1 to 1
    } else {
        PIND &= ~(1 << 1);  //Set PIND of D1 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D1_unused__pin__is_set__impl() {
#define is_set__impl D1_unused__pin__is_set__impl
#define set__impl D1_unused__pin__set__impl
    return PIND & (1 << 1);  //Get value of PIND for D1
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D1_unused__pin__is_set__impl
#define set__impl D1_unused__pin__set__impl

D1_unused__pin_t const D1_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D1_unused__pin__is_set__impl
#define set__impl D1_unused__pin__set__impl


;

#define is_set__impl D1_unused__pin__is_set__impl
#define set__impl D1_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void D1_unused__init() {
    D1_unused__ddr.set(0);
    D1_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} D1_unused_t;

extern D1_unused_t const D1_unused;

static AKAT_FORCE_INLINE u8 D1_unused__is_set__impl() {
#define is_set__impl D1_unused__is_set__impl
    return D1_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl D1_unused__is_set__impl

D1_unused_t const D1_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl D1_unused__is_set__impl


;

#define is_set__impl D1_unused__is_set__impl




#undef is_set__impl
;



;
; // 31 PD1, TxD TODO: Connect to rasp
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D0_unused__port_t;

extern D0_unused__port_t const D0_unused__port;

static AKAT_FORCE_INLINE void D0_unused__port__set__impl(u8 state) {
#define set__impl D0_unused__port__set__impl

    if (state) {
        PORTD |= 1 << 0;  //Set PORTD of D0 to 1
    } else {
        PORTD &= ~(1 << 0);  //Set PORTD of D0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D0_unused__port__is_set__impl() {
#define is_set__impl D0_unused__port__is_set__impl
#define set__impl D0_unused__port__set__impl
    return PORTD & (1 << 0);  //Get value of PORTD for D0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D0_unused__port__is_set__impl
#define set__impl D0_unused__port__set__impl

D0_unused__port_t const D0_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl D0_unused__port__is_set__impl
#define set__impl D0_unused__port__set__impl


;

#define is_set__impl D0_unused__port__is_set__impl
#define set__impl D0_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D0_unused__ddr_t;

extern D0_unused__ddr_t const D0_unused__ddr;

static AKAT_FORCE_INLINE void D0_unused__ddr__set__impl(u8 state) {
#define set__impl D0_unused__ddr__set__impl

    if (state) {
        DDRD |= 1 << 0;  //Set DDRD of D0 to 1
    } else {
        DDRD &= ~(1 << 0);  //Set DDRD of D0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D0_unused__ddr__is_set__impl() {
#define is_set__impl D0_unused__ddr__is_set__impl
#define set__impl D0_unused__ddr__set__impl
    return DDRD & (1 << 0);  //Get value of DDRD for D0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D0_unused__ddr__is_set__impl
#define set__impl D0_unused__ddr__set__impl

D0_unused__ddr_t const D0_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D0_unused__ddr__is_set__impl
#define set__impl D0_unused__ddr__set__impl


;

#define is_set__impl D0_unused__ddr__is_set__impl
#define set__impl D0_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} D0_unused__pin_t;

extern D0_unused__pin_t const D0_unused__pin;

static AKAT_FORCE_INLINE void D0_unused__pin__set__impl(u8 state) {
#define set__impl D0_unused__pin__set__impl

    if (state) {
        PIND |= 1 << 0;  //Set PIND of D0 to 1
    } else {
        PIND &= ~(1 << 0);  //Set PIND of D0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 D0_unused__pin__is_set__impl() {
#define is_set__impl D0_unused__pin__is_set__impl
#define set__impl D0_unused__pin__set__impl
    return PIND & (1 << 0);  //Get value of PIND for D0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl D0_unused__pin__is_set__impl
#define set__impl D0_unused__pin__set__impl

D0_unused__pin_t const D0_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl D0_unused__pin__is_set__impl
#define set__impl D0_unused__pin__set__impl


;

#define is_set__impl D0_unused__pin__is_set__impl
#define set__impl D0_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void D0_unused__init() {
    D0_unused__ddr.set(0);
    D0_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} D0_unused_t;

extern D0_unused_t const D0_unused;

static AKAT_FORCE_INLINE u8 D0_unused__is_set__impl() {
#define is_set__impl D0_unused__is_set__impl
    return D0_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl D0_unused__is_set__impl

D0_unused_t const D0_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl D0_unused__is_set__impl


;

#define is_set__impl D0_unused__is_set__impl




#undef is_set__impl
;



;
; // 30 PD0, RxD TODO: Connect to rasp
// .................. 29 Reset
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C5_unused__port_t;

extern C5_unused__port_t const C5_unused__port;

static AKAT_FORCE_INLINE void C5_unused__port__set__impl(u8 state) {
#define set__impl C5_unused__port__set__impl

    if (state) {
        PORTC |= 1 << 5;  //Set PORTC of C5 to 1
    } else {
        PORTC &= ~(1 << 5);  //Set PORTC of C5 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C5_unused__port__is_set__impl() {
#define is_set__impl C5_unused__port__is_set__impl
#define set__impl C5_unused__port__set__impl
    return PORTC & (1 << 5);  //Get value of PORTC for C5
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C5_unused__port__is_set__impl
#define set__impl C5_unused__port__set__impl

C5_unused__port_t const C5_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl C5_unused__port__is_set__impl
#define set__impl C5_unused__port__set__impl


;

#define is_set__impl C5_unused__port__is_set__impl
#define set__impl C5_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C5_unused__ddr_t;

extern C5_unused__ddr_t const C5_unused__ddr;

static AKAT_FORCE_INLINE void C5_unused__ddr__set__impl(u8 state) {
#define set__impl C5_unused__ddr__set__impl

    if (state) {
        DDRC |= 1 << 5;  //Set DDRC of C5 to 1
    } else {
        DDRC &= ~(1 << 5);  //Set DDRC of C5 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C5_unused__ddr__is_set__impl() {
#define is_set__impl C5_unused__ddr__is_set__impl
#define set__impl C5_unused__ddr__set__impl
    return DDRC & (1 << 5);  //Get value of DDRC for C5
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C5_unused__ddr__is_set__impl
#define set__impl C5_unused__ddr__set__impl

C5_unused__ddr_t const C5_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C5_unused__ddr__is_set__impl
#define set__impl C5_unused__ddr__set__impl


;

#define is_set__impl C5_unused__ddr__is_set__impl
#define set__impl C5_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C5_unused__pin_t;

extern C5_unused__pin_t const C5_unused__pin;

static AKAT_FORCE_INLINE void C5_unused__pin__set__impl(u8 state) {
#define set__impl C5_unused__pin__set__impl

    if (state) {
        PINC |= 1 << 5;  //Set PINC of C5 to 1
    } else {
        PINC &= ~(1 << 5);  //Set PINC of C5 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C5_unused__pin__is_set__impl() {
#define is_set__impl C5_unused__pin__is_set__impl
#define set__impl C5_unused__pin__set__impl
    return PINC & (1 << 5);  //Get value of PINC for C5
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C5_unused__pin__is_set__impl
#define set__impl C5_unused__pin__set__impl

C5_unused__pin_t const C5_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C5_unused__pin__is_set__impl
#define set__impl C5_unused__pin__set__impl


;

#define is_set__impl C5_unused__pin__is_set__impl
#define set__impl C5_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void C5_unused__init() {
    C5_unused__ddr.set(0);
    C5_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} C5_unused_t;

extern C5_unused_t const C5_unused;

static AKAT_FORCE_INLINE u8 C5_unused__is_set__impl() {
#define is_set__impl C5_unused__is_set__impl
    return C5_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl C5_unused__is_set__impl

C5_unused_t const C5_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl C5_unused__is_set__impl


;

#define is_set__impl C5_unused__is_set__impl




#undef is_set__impl
;



;
; // 28 PC5, ADC5
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C4_unused__port_t;

extern C4_unused__port_t const C4_unused__port;

static AKAT_FORCE_INLINE void C4_unused__port__set__impl(u8 state) {
#define set__impl C4_unused__port__set__impl

    if (state) {
        PORTC |= 1 << 4;  //Set PORTC of C4 to 1
    } else {
        PORTC &= ~(1 << 4);  //Set PORTC of C4 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C4_unused__port__is_set__impl() {
#define is_set__impl C4_unused__port__is_set__impl
#define set__impl C4_unused__port__set__impl
    return PORTC & (1 << 4);  //Get value of PORTC for C4
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C4_unused__port__is_set__impl
#define set__impl C4_unused__port__set__impl

C4_unused__port_t const C4_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl C4_unused__port__is_set__impl
#define set__impl C4_unused__port__set__impl


;

#define is_set__impl C4_unused__port__is_set__impl
#define set__impl C4_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C4_unused__ddr_t;

extern C4_unused__ddr_t const C4_unused__ddr;

static AKAT_FORCE_INLINE void C4_unused__ddr__set__impl(u8 state) {
#define set__impl C4_unused__ddr__set__impl

    if (state) {
        DDRC |= 1 << 4;  //Set DDRC of C4 to 1
    } else {
        DDRC &= ~(1 << 4);  //Set DDRC of C4 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C4_unused__ddr__is_set__impl() {
#define is_set__impl C4_unused__ddr__is_set__impl
#define set__impl C4_unused__ddr__set__impl
    return DDRC & (1 << 4);  //Get value of DDRC for C4
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C4_unused__ddr__is_set__impl
#define set__impl C4_unused__ddr__set__impl

C4_unused__ddr_t const C4_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C4_unused__ddr__is_set__impl
#define set__impl C4_unused__ddr__set__impl


;

#define is_set__impl C4_unused__ddr__is_set__impl
#define set__impl C4_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C4_unused__pin_t;

extern C4_unused__pin_t const C4_unused__pin;

static AKAT_FORCE_INLINE void C4_unused__pin__set__impl(u8 state) {
#define set__impl C4_unused__pin__set__impl

    if (state) {
        PINC |= 1 << 4;  //Set PINC of C4 to 1
    } else {
        PINC &= ~(1 << 4);  //Set PINC of C4 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C4_unused__pin__is_set__impl() {
#define is_set__impl C4_unused__pin__is_set__impl
#define set__impl C4_unused__pin__set__impl
    return PINC & (1 << 4);  //Get value of PINC for C4
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C4_unused__pin__is_set__impl
#define set__impl C4_unused__pin__set__impl

C4_unused__pin_t const C4_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C4_unused__pin__is_set__impl
#define set__impl C4_unused__pin__set__impl


;

#define is_set__impl C4_unused__pin__is_set__impl
#define set__impl C4_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void C4_unused__init() {
    C4_unused__ddr.set(0);
    C4_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} C4_unused_t;

extern C4_unused_t const C4_unused;

static AKAT_FORCE_INLINE u8 C4_unused__is_set__impl() {
#define is_set__impl C4_unused__is_set__impl
    return C4_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl C4_unused__is_set__impl

C4_unused_t const C4_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl C4_unused__is_set__impl


;

#define is_set__impl C4_unused__is_set__impl




#undef is_set__impl
;



;
; // 27 PC4, ADC4
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C3_unused__port_t;

extern C3_unused__port_t const C3_unused__port;

static AKAT_FORCE_INLINE void C3_unused__port__set__impl(u8 state) {
#define set__impl C3_unused__port__set__impl

    if (state) {
        PORTC |= 1 << 3;  //Set PORTC of C3 to 1
    } else {
        PORTC &= ~(1 << 3);  //Set PORTC of C3 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C3_unused__port__is_set__impl() {
#define is_set__impl C3_unused__port__is_set__impl
#define set__impl C3_unused__port__set__impl
    return PORTC & (1 << 3);  //Get value of PORTC for C3
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C3_unused__port__is_set__impl
#define set__impl C3_unused__port__set__impl

C3_unused__port_t const C3_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl C3_unused__port__is_set__impl
#define set__impl C3_unused__port__set__impl


;

#define is_set__impl C3_unused__port__is_set__impl
#define set__impl C3_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C3_unused__ddr_t;

extern C3_unused__ddr_t const C3_unused__ddr;

static AKAT_FORCE_INLINE void C3_unused__ddr__set__impl(u8 state) {
#define set__impl C3_unused__ddr__set__impl

    if (state) {
        DDRC |= 1 << 3;  //Set DDRC of C3 to 1
    } else {
        DDRC &= ~(1 << 3);  //Set DDRC of C3 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C3_unused__ddr__is_set__impl() {
#define is_set__impl C3_unused__ddr__is_set__impl
#define set__impl C3_unused__ddr__set__impl
    return DDRC & (1 << 3);  //Get value of DDRC for C3
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C3_unused__ddr__is_set__impl
#define set__impl C3_unused__ddr__set__impl

C3_unused__ddr_t const C3_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C3_unused__ddr__is_set__impl
#define set__impl C3_unused__ddr__set__impl


;

#define is_set__impl C3_unused__ddr__is_set__impl
#define set__impl C3_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C3_unused__pin_t;

extern C3_unused__pin_t const C3_unused__pin;

static AKAT_FORCE_INLINE void C3_unused__pin__set__impl(u8 state) {
#define set__impl C3_unused__pin__set__impl

    if (state) {
        PINC |= 1 << 3;  //Set PINC of C3 to 1
    } else {
        PINC &= ~(1 << 3);  //Set PINC of C3 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C3_unused__pin__is_set__impl() {
#define is_set__impl C3_unused__pin__is_set__impl
#define set__impl C3_unused__pin__set__impl
    return PINC & (1 << 3);  //Get value of PINC for C3
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C3_unused__pin__is_set__impl
#define set__impl C3_unused__pin__set__impl

C3_unused__pin_t const C3_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C3_unused__pin__is_set__impl
#define set__impl C3_unused__pin__set__impl


;

#define is_set__impl C3_unused__pin__is_set__impl
#define set__impl C3_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void C3_unused__init() {
    C3_unused__ddr.set(0);
    C3_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} C3_unused_t;

extern C3_unused_t const C3_unused;

static AKAT_FORCE_INLINE u8 C3_unused__is_set__impl() {
#define is_set__impl C3_unused__is_set__impl
    return C3_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl C3_unused__is_set__impl

C3_unused_t const C3_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl C3_unused__is_set__impl


;

#define is_set__impl C3_unused__is_set__impl




#undef is_set__impl
;



;
; // 26 PC3, ADC3
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C2_unused__port_t;

extern C2_unused__port_t const C2_unused__port;

static AKAT_FORCE_INLINE void C2_unused__port__set__impl(u8 state) {
#define set__impl C2_unused__port__set__impl

    if (state) {
        PORTC |= 1 << 2;  //Set PORTC of C2 to 1
    } else {
        PORTC &= ~(1 << 2);  //Set PORTC of C2 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C2_unused__port__is_set__impl() {
#define is_set__impl C2_unused__port__is_set__impl
#define set__impl C2_unused__port__set__impl
    return PORTC & (1 << 2);  //Get value of PORTC for C2
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C2_unused__port__is_set__impl
#define set__impl C2_unused__port__set__impl

C2_unused__port_t const C2_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl C2_unused__port__is_set__impl
#define set__impl C2_unused__port__set__impl


;

#define is_set__impl C2_unused__port__is_set__impl
#define set__impl C2_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C2_unused__ddr_t;

extern C2_unused__ddr_t const C2_unused__ddr;

static AKAT_FORCE_INLINE void C2_unused__ddr__set__impl(u8 state) {
#define set__impl C2_unused__ddr__set__impl

    if (state) {
        DDRC |= 1 << 2;  //Set DDRC of C2 to 1
    } else {
        DDRC &= ~(1 << 2);  //Set DDRC of C2 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C2_unused__ddr__is_set__impl() {
#define is_set__impl C2_unused__ddr__is_set__impl
#define set__impl C2_unused__ddr__set__impl
    return DDRC & (1 << 2);  //Get value of DDRC for C2
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C2_unused__ddr__is_set__impl
#define set__impl C2_unused__ddr__set__impl

C2_unused__ddr_t const C2_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C2_unused__ddr__is_set__impl
#define set__impl C2_unused__ddr__set__impl


;

#define is_set__impl C2_unused__ddr__is_set__impl
#define set__impl C2_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C2_unused__pin_t;

extern C2_unused__pin_t const C2_unused__pin;

static AKAT_FORCE_INLINE void C2_unused__pin__set__impl(u8 state) {
#define set__impl C2_unused__pin__set__impl

    if (state) {
        PINC |= 1 << 2;  //Set PINC of C2 to 1
    } else {
        PINC &= ~(1 << 2);  //Set PINC of C2 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C2_unused__pin__is_set__impl() {
#define is_set__impl C2_unused__pin__is_set__impl
#define set__impl C2_unused__pin__set__impl
    return PINC & (1 << 2);  //Get value of PINC for C2
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C2_unused__pin__is_set__impl
#define set__impl C2_unused__pin__set__impl

C2_unused__pin_t const C2_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C2_unused__pin__is_set__impl
#define set__impl C2_unused__pin__set__impl


;

#define is_set__impl C2_unused__pin__is_set__impl
#define set__impl C2_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void C2_unused__init() {
    C2_unused__ddr.set(0);
    C2_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} C2_unused_t;

extern C2_unused_t const C2_unused;

static AKAT_FORCE_INLINE u8 C2_unused__is_set__impl() {
#define is_set__impl C2_unused__is_set__impl
    return C2_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl C2_unused__is_set__impl

C2_unused_t const C2_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl C2_unused__is_set__impl


;

#define is_set__impl C2_unused__is_set__impl




#undef is_set__impl
;



;
; // 25 PC2, ADC2
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C1_unused__port_t;

extern C1_unused__port_t const C1_unused__port;

static AKAT_FORCE_INLINE void C1_unused__port__set__impl(u8 state) {
#define set__impl C1_unused__port__set__impl

    if (state) {
        PORTC |= 1 << 1;  //Set PORTC of C1 to 1
    } else {
        PORTC &= ~(1 << 1);  //Set PORTC of C1 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C1_unused__port__is_set__impl() {
#define is_set__impl C1_unused__port__is_set__impl
#define set__impl C1_unused__port__set__impl
    return PORTC & (1 << 1);  //Get value of PORTC for C1
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C1_unused__port__is_set__impl
#define set__impl C1_unused__port__set__impl

C1_unused__port_t const C1_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl C1_unused__port__is_set__impl
#define set__impl C1_unused__port__set__impl


;

#define is_set__impl C1_unused__port__is_set__impl
#define set__impl C1_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C1_unused__ddr_t;

extern C1_unused__ddr_t const C1_unused__ddr;

static AKAT_FORCE_INLINE void C1_unused__ddr__set__impl(u8 state) {
#define set__impl C1_unused__ddr__set__impl

    if (state) {
        DDRC |= 1 << 1;  //Set DDRC of C1 to 1
    } else {
        DDRC &= ~(1 << 1);  //Set DDRC of C1 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C1_unused__ddr__is_set__impl() {
#define is_set__impl C1_unused__ddr__is_set__impl
#define set__impl C1_unused__ddr__set__impl
    return DDRC & (1 << 1);  //Get value of DDRC for C1
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C1_unused__ddr__is_set__impl
#define set__impl C1_unused__ddr__set__impl

C1_unused__ddr_t const C1_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C1_unused__ddr__is_set__impl
#define set__impl C1_unused__ddr__set__impl


;

#define is_set__impl C1_unused__ddr__is_set__impl
#define set__impl C1_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C1_unused__pin_t;

extern C1_unused__pin_t const C1_unused__pin;

static AKAT_FORCE_INLINE void C1_unused__pin__set__impl(u8 state) {
#define set__impl C1_unused__pin__set__impl

    if (state) {
        PINC |= 1 << 1;  //Set PINC of C1 to 1
    } else {
        PINC &= ~(1 << 1);  //Set PINC of C1 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C1_unused__pin__is_set__impl() {
#define is_set__impl C1_unused__pin__is_set__impl
#define set__impl C1_unused__pin__set__impl
    return PINC & (1 << 1);  //Get value of PINC for C1
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C1_unused__pin__is_set__impl
#define set__impl C1_unused__pin__set__impl

C1_unused__pin_t const C1_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C1_unused__pin__is_set__impl
#define set__impl C1_unused__pin__set__impl


;

#define is_set__impl C1_unused__pin__is_set__impl
#define set__impl C1_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void C1_unused__init() {
    C1_unused__ddr.set(0);
    C1_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} C1_unused_t;

extern C1_unused_t const C1_unused;

static AKAT_FORCE_INLINE u8 C1_unused__is_set__impl() {
#define is_set__impl C1_unused__is_set__impl
    return C1_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl C1_unused__is_set__impl

C1_unused_t const C1_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl C1_unused__is_set__impl


;

#define is_set__impl C1_unused__is_set__impl




#undef is_set__impl
;



;
; // 24 PC1, ADC1
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C0_unused__port_t;

extern C0_unused__port_t const C0_unused__port;

static AKAT_FORCE_INLINE void C0_unused__port__set__impl(u8 state) {
#define set__impl C0_unused__port__set__impl

    if (state) {
        PORTC |= 1 << 0;  //Set PORTC of C0 to 1
    } else {
        PORTC &= ~(1 << 0);  //Set PORTC of C0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C0_unused__port__is_set__impl() {
#define is_set__impl C0_unused__port__is_set__impl
#define set__impl C0_unused__port__set__impl
    return PORTC & (1 << 0);  //Get value of PORTC for C0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C0_unused__port__is_set__impl
#define set__impl C0_unused__port__set__impl

C0_unused__port_t const C0_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl C0_unused__port__is_set__impl
#define set__impl C0_unused__port__set__impl


;

#define is_set__impl C0_unused__port__is_set__impl
#define set__impl C0_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C0_unused__ddr_t;

extern C0_unused__ddr_t const C0_unused__ddr;

static AKAT_FORCE_INLINE void C0_unused__ddr__set__impl(u8 state) {
#define set__impl C0_unused__ddr__set__impl

    if (state) {
        DDRC |= 1 << 0;  //Set DDRC of C0 to 1
    } else {
        DDRC &= ~(1 << 0);  //Set DDRC of C0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C0_unused__ddr__is_set__impl() {
#define is_set__impl C0_unused__ddr__is_set__impl
#define set__impl C0_unused__ddr__set__impl
    return DDRC & (1 << 0);  //Get value of DDRC for C0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C0_unused__ddr__is_set__impl
#define set__impl C0_unused__ddr__set__impl

C0_unused__ddr_t const C0_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C0_unused__ddr__is_set__impl
#define set__impl C0_unused__ddr__set__impl


;

#define is_set__impl C0_unused__ddr__is_set__impl
#define set__impl C0_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C0_unused__pin_t;

extern C0_unused__pin_t const C0_unused__pin;

static AKAT_FORCE_INLINE void C0_unused__pin__set__impl(u8 state) {
#define set__impl C0_unused__pin__set__impl

    if (state) {
        PINC |= 1 << 0;  //Set PINC of C0 to 1
    } else {
        PINC &= ~(1 << 0);  //Set PINC of C0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C0_unused__pin__is_set__impl() {
#define is_set__impl C0_unused__pin__is_set__impl
#define set__impl C0_unused__pin__set__impl
    return PINC & (1 << 0);  //Get value of PINC for C0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C0_unused__pin__is_set__impl
#define set__impl C0_unused__pin__set__impl

C0_unused__pin_t const C0_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C0_unused__pin__is_set__impl
#define set__impl C0_unused__pin__set__impl


;

#define is_set__impl C0_unused__pin__is_set__impl
#define set__impl C0_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void C0_unused__init() {
    C0_unused__ddr.set(0);
    C0_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} C0_unused_t;

extern C0_unused_t const C0_unused;

static AKAT_FORCE_INLINE u8 C0_unused__is_set__impl() {
#define is_set__impl C0_unused__is_set__impl
    return C0_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl C0_unused__is_set__impl

C0_unused_t const C0_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl C0_unused__is_set__impl


;

#define is_set__impl C0_unused__is_set__impl




#undef is_set__impl
;



;
; // 23 PC0, ADC0
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C7_unused__port_t;

extern C7_unused__port_t const C7_unused__port;

static AKAT_FORCE_INLINE void C7_unused__port__set__impl(u8 state) {
#define set__impl C7_unused__port__set__impl

    if (state) {
        PORTC |= 1 << 7;  //Set PORTC of C7 to 1
    } else {
        PORTC &= ~(1 << 7);  //Set PORTC of C7 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C7_unused__port__is_set__impl() {
#define is_set__impl C7_unused__port__is_set__impl
#define set__impl C7_unused__port__set__impl
    return PORTC & (1 << 7);  //Get value of PORTC for C7
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C7_unused__port__is_set__impl
#define set__impl C7_unused__port__set__impl

C7_unused__port_t const C7_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl C7_unused__port__is_set__impl
#define set__impl C7_unused__port__set__impl


;

#define is_set__impl C7_unused__port__is_set__impl
#define set__impl C7_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C7_unused__ddr_t;

extern C7_unused__ddr_t const C7_unused__ddr;

static AKAT_FORCE_INLINE void C7_unused__ddr__set__impl(u8 state) {
#define set__impl C7_unused__ddr__set__impl

    if (state) {
        DDRC |= 1 << 7;  //Set DDRC of C7 to 1
    } else {
        DDRC &= ~(1 << 7);  //Set DDRC of C7 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C7_unused__ddr__is_set__impl() {
#define is_set__impl C7_unused__ddr__is_set__impl
#define set__impl C7_unused__ddr__set__impl
    return DDRC & (1 << 7);  //Get value of DDRC for C7
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C7_unused__ddr__is_set__impl
#define set__impl C7_unused__ddr__set__impl

C7_unused__ddr_t const C7_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C7_unused__ddr__is_set__impl
#define set__impl C7_unused__ddr__set__impl


;

#define is_set__impl C7_unused__ddr__is_set__impl
#define set__impl C7_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C7_unused__pin_t;

extern C7_unused__pin_t const C7_unused__pin;

static AKAT_FORCE_INLINE void C7_unused__pin__set__impl(u8 state) {
#define set__impl C7_unused__pin__set__impl

    if (state) {
        PINC |= 1 << 7;  //Set PINC of C7 to 1
    } else {
        PINC &= ~(1 << 7);  //Set PINC of C7 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C7_unused__pin__is_set__impl() {
#define is_set__impl C7_unused__pin__is_set__impl
#define set__impl C7_unused__pin__set__impl
    return PINC & (1 << 7);  //Get value of PINC for C7
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C7_unused__pin__is_set__impl
#define set__impl C7_unused__pin__set__impl

C7_unused__pin_t const C7_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C7_unused__pin__is_set__impl
#define set__impl C7_unused__pin__set__impl


;

#define is_set__impl C7_unused__pin__is_set__impl
#define set__impl C7_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void C7_unused__init() {
    C7_unused__ddr.set(0);
    C7_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} C7_unused_t;

extern C7_unused_t const C7_unused;

static AKAT_FORCE_INLINE u8 C7_unused__is_set__impl() {
#define is_set__impl C7_unused__is_set__impl
    return C7_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl C7_unused__is_set__impl

C7_unused_t const C7_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl C7_unused__is_set__impl


;

#define is_set__impl C7_unused__is_set__impl




#undef is_set__impl
;



;
; // 22 PC7, ADC7
// .................. 21 AGND
// .................. 20 AREF
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C6_unused__port_t;

extern C6_unused__port_t const C6_unused__port;

static AKAT_FORCE_INLINE void C6_unused__port__set__impl(u8 state) {
#define set__impl C6_unused__port__set__impl

    if (state) {
        PORTC |= 1 << 6;  //Set PORTC of C6 to 1
    } else {
        PORTC &= ~(1 << 6);  //Set PORTC of C6 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C6_unused__port__is_set__impl() {
#define is_set__impl C6_unused__port__is_set__impl
#define set__impl C6_unused__port__set__impl
    return PORTC & (1 << 6);  //Get value of PORTC for C6
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C6_unused__port__is_set__impl
#define set__impl C6_unused__port__set__impl

C6_unused__port_t const C6_unused__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl C6_unused__port__is_set__impl
#define set__impl C6_unused__port__set__impl


;

#define is_set__impl C6_unused__port__is_set__impl
#define set__impl C6_unused__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C6_unused__ddr_t;

extern C6_unused__ddr_t const C6_unused__ddr;

static AKAT_FORCE_INLINE void C6_unused__ddr__set__impl(u8 state) {
#define set__impl C6_unused__ddr__set__impl

    if (state) {
        DDRC |= 1 << 6;  //Set DDRC of C6 to 1
    } else {
        DDRC &= ~(1 << 6);  //Set DDRC of C6 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C6_unused__ddr__is_set__impl() {
#define is_set__impl C6_unused__ddr__is_set__impl
#define set__impl C6_unused__ddr__set__impl
    return DDRC & (1 << 6);  //Get value of DDRC for C6
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C6_unused__ddr__is_set__impl
#define set__impl C6_unused__ddr__set__impl

C6_unused__ddr_t const C6_unused__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C6_unused__ddr__is_set__impl
#define set__impl C6_unused__ddr__set__impl


;

#define is_set__impl C6_unused__ddr__is_set__impl
#define set__impl C6_unused__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} C6_unused__pin_t;

extern C6_unused__pin_t const C6_unused__pin;

static AKAT_FORCE_INLINE void C6_unused__pin__set__impl(u8 state) {
#define set__impl C6_unused__pin__set__impl

    if (state) {
        PINC |= 1 << 6;  //Set PINC of C6 to 1
    } else {
        PINC &= ~(1 << 6);  //Set PINC of C6 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 C6_unused__pin__is_set__impl() {
#define is_set__impl C6_unused__pin__is_set__impl
#define set__impl C6_unused__pin__set__impl
    return PINC & (1 << 6);  //Get value of PINC for C6
#undef is_set__impl
#undef set__impl
}
#define is_set__impl C6_unused__pin__is_set__impl
#define set__impl C6_unused__pin__set__impl

C6_unused__pin_t const C6_unused__pin = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl C6_unused__pin__is_set__impl
#define set__impl C6_unused__pin__set__impl


;

#define is_set__impl C6_unused__pin__is_set__impl
#define set__impl C6_unused__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void C6_unused__init() {
    C6_unused__ddr.set(0);
    C6_unused__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} C6_unused_t;

extern C6_unused_t const C6_unused;

static AKAT_FORCE_INLINE u8 C6_unused__is_set__impl() {
#define is_set__impl C6_unused__is_set__impl
    return C6_unused__pin.is_set();
#undef is_set__impl
}
#define is_set__impl C6_unused__is_set__impl

C6_unused_t const C6_unused = {.is_set = &is_set__impl
                              };


#undef is_set__impl
#define is_set__impl C6_unused__is_set__impl


;

#define is_set__impl C6_unused__is_set__impl




#undef is_set__impl
;



;
; // 19 PC6, ADC6
// .................. 18 AVCC
// B5 - blue led .... 17 PB5, SCK

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Timers

// 16-bit Timer1 is used for 'X_EVERY_DECISECOND$'


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Led pins

typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} blue_led__port_t;

extern blue_led__port_t const blue_led__port;

static AKAT_FORCE_INLINE void blue_led__port__set__impl(u8 state) {
#define set__impl blue_led__port__set__impl

    if (state) {
        PORTB |= 1 << 5;  //Set PORTB of B5 to 1
    } else {
        PORTB &= ~(1 << 5);  //Set PORTB of B5 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 blue_led__port__is_set__impl() {
#define is_set__impl blue_led__port__is_set__impl
#define set__impl blue_led__port__set__impl
    return PORTB & (1 << 5);  //Get value of PORTB for B5
#undef is_set__impl
#undef set__impl
}
#define is_set__impl blue_led__port__is_set__impl
#define set__impl blue_led__port__set__impl

blue_led__port_t const blue_led__port = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl blue_led__port__is_set__impl
#define set__impl blue_led__port__set__impl


;

#define is_set__impl blue_led__port__is_set__impl
#define set__impl blue_led__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} blue_led__ddr_t;

extern blue_led__ddr_t const blue_led__ddr;

static AKAT_FORCE_INLINE void blue_led__ddr__set__impl(u8 state) {
#define set__impl blue_led__ddr__set__impl

    if (state) {
        DDRB |= 1 << 5;  //Set DDRB of B5 to 1
    } else {
        DDRB &= ~(1 << 5);  //Set DDRB of B5 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 blue_led__ddr__is_set__impl() {
#define is_set__impl blue_led__ddr__is_set__impl
#define set__impl blue_led__ddr__set__impl
    return DDRB & (1 << 5);  //Get value of DDRB for B5
#undef is_set__impl
#undef set__impl
}
#define is_set__impl blue_led__ddr__is_set__impl
#define set__impl blue_led__ddr__set__impl

blue_led__ddr_t const blue_led__ddr = {.set = &set__impl
                                       ,
                                       .is_set = &is_set__impl
                                      };


#undef is_set__impl
#undef set__impl
#define is_set__impl blue_led__ddr__is_set__impl
#define set__impl blue_led__ddr__set__impl


;

#define is_set__impl blue_led__ddr__is_set__impl
#define set__impl blue_led__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void blue_led__init() {
    blue_led__ddr.set(1); //Init B5 as output
}

;





typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} blue_led_t;

extern blue_led_t const blue_led;

static AKAT_FORCE_INLINE void blue_led__set__impl(u8 state) {
#define set__impl blue_led__set__impl
    blue_led__port.set(state);
#undef set__impl
}
static AKAT_FORCE_INLINE u8 blue_led__is_set__impl() {
#define is_set__impl blue_led__is_set__impl
#define set__impl blue_led__set__impl
    return blue_led__port.is_set();
#undef is_set__impl
#undef set__impl
}
#define is_set__impl blue_led__is_set__impl
#define set__impl blue_led__set__impl

blue_led_t const blue_led = {.set = &set__impl
                                    ,
                             .is_set = &is_set__impl
                            };


#undef is_set__impl
#undef set__impl
#define is_set__impl blue_led__is_set__impl
#define set__impl blue_led__set__impl


;

#define is_set__impl blue_led__is_set__impl
#define set__impl blue_led__set__impl





#undef is_set__impl
#undef set__impl
;



;


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Watchdog

#include <avr/wdt.h>

static AKAT_FORCE_INLINE void watchdog_init() {
    wdt_enable(WDTO_8S);
}

;





static AKAT_FORCE_INLINE void watchdog_reset() {
    wdt_reset();
}

;




;

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Activity indication

// This piece of code will be executed in akat event/thread loop every 1/10 second.
// We use to turn the blue led ON and OFF

static AKAT_FORCE_INLINE void akat_on_every_decisecond();

// Can't use LOW register here!
/* Using register r16 for akat_every_decisecond_run_required */;

register u8 akat_every_decisecond_run_required asm ("r16");

;
;


static AKAT_FORCE_INLINE void akat_on_every_decisecond_runner() {
//Tell gcc that this variable can be changed somehow (in our case via ISR)
    AKAT_FLUSH_REG_VAR(akat_every_decisecond_run_required);

    if (akat_every_decisecond_run_required) {
        akat_every_decisecond_run_required = AKAT_FALSE;
        akat_on_every_decisecond();
    }
}

;





ISR(TIMER1_COMPA_vect, ISR_NAKED) {
    // NOTE: Make sure that 'akat_every_decisecond_run_required' is not a register under R16!
    // NOTE: Otherwise we have to save SREG. That's why we use assembler directly here.
    asm volatile("ldi %0, 0x01" : "=r" (akat_every_decisecond_run_required));
    asm volatile("reti");
}


static u8 activity_led__state = 0;
static AKAT_FORCE_INLINE void activity_led() {
#define state activity_led__state
    ;
    blue_led.set(state);
    state = !state;
#undef state
}

;






////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Main


static AKAT_FORCE_INLINE void akat_on_every_decisecond() {
    activity_led();
}

static AKAT_FORCE_INLINE void timer1() {
    OCR1A = 24999;
    TIMSK1 |= (1 << OCIE1A) | (0 << OCIE1B);
    //Configuring Timer1 for prescaler 64
    TCCR1B |= ((0 << CS12) | (1 << CS11) | (1 << CS10) /* Prescaler 64 */) | (1 << WGM12);
}

;




;

AKAT_NO_RETURN void main() {
    asm volatile ("EOR r2, r2\nINC r2": "=r"(__akat_one__));
    akat_every_decisecond_run_required = 0;
    D3_unused__init();
    D4_unused__init();
    D5_unused__init();
    D6_unused__init();
    D7_unused__init();
    B0_unused__init();
    B1_unused__init();
    B2_unused__init();
    B3_unused__init();
    B4_unused__init();
    D2_unused__init();
    D1_unused__init();
    D0_unused__init();
    C5_unused__init();
    C4_unused__init();
    C3_unused__init();
    C2_unused__init();
    C1_unused__init();
    C0_unused__init();
    C7_unused__init();
    C6_unused__init();
    blue_led__init();
    watchdog_init();
    timer1();
    //Init
    //Enable interrupts
    sei();

    //Endless loop with threads, tasks and such
    while (1) {
        watchdog_reset();
        akat_on_every_decisecond_runner();
    }
}

;





