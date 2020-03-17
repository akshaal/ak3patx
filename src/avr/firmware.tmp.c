;

// NOTE: Sometimes it's nice to try to see which one is best to have some not low, must try some combinations
// USE_REG$(global variable name);
// USE_REG$(global variable name, low);

/* Using register r16 for usart0_writer__byte_to_send */;
/* Using register r17 for usart0_writer__akat_coroutine_state */;
/* Using register r18 for usart0_writer__u8_to_format_and_send */;
/* Using register r3 for usart0_reader__read_command__dequeue_byte__akat_coroutine_state */;
/* Using register r4 for usart0_writer__send_byte__akat_coroutine_state */;
/* Using register r5 for ds18b20_thread__akat_coroutine_state */;

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

// Here is what we are going to use for communication using USB/serial port
// Frame format is 8N1 (8 bits, no parity, 1 stop bit)
#define AK_USART0_BAUD_RATE     9600
#define AK_USART0_FRAME_FORMAT  (H(UCSZ00) | H(UCSZ01))

// Size of buffer for bytes we receive from USART0/USB.
// RX-Interrupt puts bytes into the given ring buffer if there is space in it.
// A thread takes byte from the buffer and process it.
// Must be power of 2!
#define AK_USART0_RX_BUF_SIZE  128

// 16Mhz, that's external oscillator on Mega 2560.
// This doesn't configure it here, it just tells to our build system
// what we is actually using! Configuration is done using fuses (see flash-avr-fuses).
// Actual value might be a different one, one can actually measure it to provide
// some kind of accuracy (calibration) if needed.
static AKAT_FORCE_INLINE AKAT_CONST uint32_t akat_cpu_freq_hz() {
    return 16000000;
}
;

static const char HEX[16] = "0123456789abcdef";

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
// B0 ext temp ...... 12 PB0
// B1 CASE PWR BTN .. 13 PB1
// B2 onboard temp .. 14 PB2
// B3 CASE HDD LED .. 15 PB3, MOSI
// B4 CASE PWD LED .. 16 PB4, MISO
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
// TX USART ......... 31 PD1, TxD
// RX USART ......... 30 PD0, RxD
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
// C0 ATX PS ON ..... 23 PC0, ADC0
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
// Performance counter

// Measure how many iteration we are able to perform between decisecond ticks
// We use internal counter which we increment in every iteration of main loop
// and then we copy the counter to main_loop_iterations_in_last_decisecond
// every decisecond and reset the internal counter.
// Lowe the value in main_loop_iterations_in_last_decisecond, busyier the loop!

static u32 __current_main_loop_iterations = 0;
static u32 main_loop_iterations_in_last_decisecond = 0;

;
;
;


static AKAT_FORCE_INLINE void performance_runnable() {
    __current_main_loop_iterations += 1;
}

;






static AKAT_FORCE_INLINE void akat_on_every_decisecond();

// Can't use LOW register here!
/* Using register r19 for akat_every_decisecond_run_required */;

register u8 akat_every_decisecond_run_required asm ("r19");

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


static AKAT_FORCE_INLINE void performance_ticker() {
    main_loop_iterations_in_last_decisecond = __current_main_loop_iterations;
    __current_main_loop_iterations = 0;
}

;






////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Uptime

static u32 uptime_deciseconds = 0;

;
;



static AKAT_FORCE_INLINE void uptime_ticker() {
    uptime_deciseconds += 1;
}

;






////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Main logic

static u8 power_should_be_on = 0;

//Main state that dictates whether power should be on now or off as requested by operator.
;
;
;

// DS18B20
// Read temperature from DS18B20.
// DS18B20 is supposed to be connected to the given port and must be properly powered.
// (parasitic powering mode is not supported/tested).
// DS18B20 must be the only device connected to the pin because we use SKIP-ROM command.

static void ds18b20_ticker();

typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} ds18b20_onboard__pin__port_t;

extern ds18b20_onboard__pin__port_t const ds18b20_onboard__pin__port;

static AKAT_FORCE_INLINE void ds18b20_onboard__pin__port__set__impl(u8 state) {
#define set__impl ds18b20_onboard__pin__port__set__impl

    if (state) {
        PORTB |= 1 << 2;  //Set PORTB of B2 to 1
    } else {
        PORTB &= ~(1 << 2);  //Set PORTB of B2 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 ds18b20_onboard__pin__port__is_set__impl() {
#define is_set__impl ds18b20_onboard__pin__port__is_set__impl
#define set__impl ds18b20_onboard__pin__port__set__impl
    return PORTB & (1 << 2);  //Get value of PORTB for B2
#undef is_set__impl
#undef set__impl
}
#define is_set__impl ds18b20_onboard__pin__port__is_set__impl
#define set__impl ds18b20_onboard__pin__port__set__impl

ds18b20_onboard__pin__port_t const ds18b20_onboard__pin__port = {.set = &set__impl
                                                                 ,
                                                                 .is_set = &is_set__impl
                                                                };


#undef is_set__impl
#undef set__impl
#define is_set__impl ds18b20_onboard__pin__port__is_set__impl
#define set__impl ds18b20_onboard__pin__port__set__impl


;

#define is_set__impl ds18b20_onboard__pin__port__is_set__impl
#define set__impl ds18b20_onboard__pin__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} ds18b20_onboard__pin__ddr_t;

extern ds18b20_onboard__pin__ddr_t const ds18b20_onboard__pin__ddr;

static AKAT_FORCE_INLINE void ds18b20_onboard__pin__ddr__set__impl(u8 state) {
#define set__impl ds18b20_onboard__pin__ddr__set__impl

    if (state) {
        DDRB |= 1 << 2;  //Set DDRB of B2 to 1
    } else {
        DDRB &= ~(1 << 2);  //Set DDRB of B2 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 ds18b20_onboard__pin__ddr__is_set__impl() {
#define is_set__impl ds18b20_onboard__pin__ddr__is_set__impl
#define set__impl ds18b20_onboard__pin__ddr__set__impl
    return DDRB & (1 << 2);  //Get value of DDRB for B2
#undef is_set__impl
#undef set__impl
}
#define is_set__impl ds18b20_onboard__pin__ddr__is_set__impl
#define set__impl ds18b20_onboard__pin__ddr__set__impl

ds18b20_onboard__pin__ddr_t const ds18b20_onboard__pin__ddr = {.set = &set__impl
                                                               ,
                                                               .is_set = &is_set__impl
                                                              };


#undef is_set__impl
#undef set__impl
#define is_set__impl ds18b20_onboard__pin__ddr__is_set__impl
#define set__impl ds18b20_onboard__pin__ddr__set__impl


;

#define is_set__impl ds18b20_onboard__pin__ddr__is_set__impl
#define set__impl ds18b20_onboard__pin__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} ds18b20_onboard__pin__pin_t;

extern ds18b20_onboard__pin__pin_t const ds18b20_onboard__pin__pin;

static AKAT_FORCE_INLINE void ds18b20_onboard__pin__pin__set__impl(u8 state) {
#define set__impl ds18b20_onboard__pin__pin__set__impl

    if (state) {
        PINB |= 1 << 2;  //Set PINB of B2 to 1
    } else {
        PINB &= ~(1 << 2);  //Set PINB of B2 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 ds18b20_onboard__pin__pin__is_set__impl() {
#define is_set__impl ds18b20_onboard__pin__pin__is_set__impl
#define set__impl ds18b20_onboard__pin__pin__set__impl
    return PINB & (1 << 2);  //Get value of PINB for B2
#undef is_set__impl
#undef set__impl
}
#define is_set__impl ds18b20_onboard__pin__pin__is_set__impl
#define set__impl ds18b20_onboard__pin__pin__set__impl

ds18b20_onboard__pin__pin_t const ds18b20_onboard__pin__pin = {.set = &set__impl
                                                               ,
                                                               .is_set = &is_set__impl
                                                              };


#undef is_set__impl
#undef set__impl
#define is_set__impl ds18b20_onboard__pin__pin__is_set__impl
#define set__impl ds18b20_onboard__pin__pin__set__impl


;

#define is_set__impl ds18b20_onboard__pin__pin__is_set__impl
#define set__impl ds18b20_onboard__pin__pin__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set_input_mode)();
    void (* const set_output_mode)();
    u8 (* const is_set)();
    void (* const set)(u8 state);
} ds18b20_onboard__pin_t;

extern ds18b20_onboard__pin_t const ds18b20_onboard__pin;

static AKAT_FORCE_INLINE void ds18b20_onboard__pin__set_input_mode__impl() {
#define set_input_mode__impl ds18b20_onboard__pin__set_input_mode__impl
    ds18b20_onboard__pin__ddr.set(0);
    ds18b20_onboard__pin__port.set(1);
#undef set_input_mode__impl
}
static AKAT_FORCE_INLINE void ds18b20_onboard__pin__set_output_mode__impl() {
#define set_input_mode__impl ds18b20_onboard__pin__set_input_mode__impl
#define set_output_mode__impl ds18b20_onboard__pin__set_output_mode__impl
    ds18b20_onboard__pin__ddr.set(1);
#undef set_input_mode__impl
#undef set_output_mode__impl
}
static AKAT_FORCE_INLINE u8 ds18b20_onboard__pin__is_set__impl() {
#define is_set__impl ds18b20_onboard__pin__is_set__impl
#define set_input_mode__impl ds18b20_onboard__pin__set_input_mode__impl
#define set_output_mode__impl ds18b20_onboard__pin__set_output_mode__impl
    return ds18b20_onboard__pin__pin.is_set();
#undef is_set__impl
#undef set_input_mode__impl
#undef set_output_mode__impl
}
static AKAT_FORCE_INLINE void ds18b20_onboard__pin__set__impl(u8 state) {
#define is_set__impl ds18b20_onboard__pin__is_set__impl
#define set__impl ds18b20_onboard__pin__set__impl
#define set_input_mode__impl ds18b20_onboard__pin__set_input_mode__impl
#define set_output_mode__impl ds18b20_onboard__pin__set_output_mode__impl
    ds18b20_onboard__pin__port.set(state);
#undef is_set__impl
#undef set__impl
#undef set_input_mode__impl
#undef set_output_mode__impl
}
#define is_set__impl ds18b20_onboard__pin__is_set__impl
#define set__impl ds18b20_onboard__pin__set__impl
#define set_input_mode__impl ds18b20_onboard__pin__set_input_mode__impl
#define set_output_mode__impl ds18b20_onboard__pin__set_output_mode__impl

ds18b20_onboard__pin_t const ds18b20_onboard__pin = {.set_input_mode = &set_input_mode__impl
                                                     ,
                                                     .set_output_mode = &set_output_mode__impl
                                                             ,
                                                     .is_set = &is_set__impl
                                                             ,
                                                     .set = &set__impl
                                                    };


#undef is_set__impl
#undef set__impl
#undef set_input_mode__impl
#undef set_output_mode__impl
#define is_set__impl ds18b20_onboard__pin__is_set__impl
#define set__impl ds18b20_onboard__pin__set__impl
#define set_input_mode__impl ds18b20_onboard__pin__set_input_mode__impl
#define set_output_mode__impl ds18b20_onboard__pin__set_output_mode__impl


;

#define is_set__impl ds18b20_onboard__pin__is_set__impl
#define set__impl ds18b20_onboard__pin__set__impl
#define set_input_mode__impl ds18b20_onboard__pin__set_input_mode__impl
#define set_output_mode__impl ds18b20_onboard__pin__set_output_mode__impl







#undef is_set__impl
#undef set__impl
#undef set_input_mode__impl
#undef set_output_mode__impl
;



;

static AKAT_FORCE_INLINE void ds18b20_onboard__init() {
//Safe state - input
    ds18b20_onboard__pin.set_input_mode();
}

;





// Static variable for communication between our thread and other parts of code
static u8 ds18b20_onboard__connected = 0;
static u8 ds18b20_onboard__received_byte = 0;
static u8 ds18b20_onboard__scratchpad[9] = {};
static u8 ds18b20_onboard__update_id = 0;
static u8 ds18b20_onboard__updated_deciseconds_ago = 255;
static u8 ds18b20_onboard__crc_errors = 0;
static u8 ds18b20_onboard__disconnects = 0;
static u16 ds18b20_onboard__temperatureX16 = 0;

//Whether reset procedure detect presence pulse or not
;

//Last received byte
;

//Last scratchpad
;

//Statistics
;
;
;
;

//Temperature (must be divided by 16 to convert to degrees)
;
;



static AKAT_FORCE_INLINE void ds18b20_onboard__ticker() {
//Maintain freshness
    ds18b20_onboard__updated_deciseconds_ago += AKAT_ONE;

    if (!ds18b20_onboard__updated_deciseconds_ago) {//We can't go beyond 255
        ds18b20_onboard__updated_deciseconds_ago -= AKAT_ONE;
    }
}

;





typedef struct {
    u8 (* const get_updated_deciseconds_ago)();
    u8 (* const get_update_id)();
    u8 (* const get_disconnects)();
    u8 (* const get_crc_errors)();
    u16 (* const get_temperatureX16)();
} ds18b20_onboard_t;

extern ds18b20_onboard_t const ds18b20_onboard;

static AKAT_FORCE_INLINE u8 ds18b20_onboard__get_updated_deciseconds_ago__impl() {
#define get_updated_deciseconds_ago__impl ds18b20_onboard__get_updated_deciseconds_ago__impl
    return ds18b20_onboard__updated_deciseconds_ago;
#undef get_updated_deciseconds_ago__impl
}
static AKAT_FORCE_INLINE u8 ds18b20_onboard__get_update_id__impl() {
#define get_update_id__impl ds18b20_onboard__get_update_id__impl
#define get_updated_deciseconds_ago__impl ds18b20_onboard__get_updated_deciseconds_ago__impl
    return ds18b20_onboard__update_id;
#undef get_update_id__impl
#undef get_updated_deciseconds_ago__impl
}
static AKAT_FORCE_INLINE u8 ds18b20_onboard__get_disconnects__impl() {
#define get_disconnects__impl ds18b20_onboard__get_disconnects__impl
#define get_update_id__impl ds18b20_onboard__get_update_id__impl
#define get_updated_deciseconds_ago__impl ds18b20_onboard__get_updated_deciseconds_ago__impl
    return ds18b20_onboard__disconnects;
#undef get_disconnects__impl
#undef get_update_id__impl
#undef get_updated_deciseconds_ago__impl
}
static AKAT_FORCE_INLINE u8 ds18b20_onboard__get_crc_errors__impl() {
#define get_crc_errors__impl ds18b20_onboard__get_crc_errors__impl
#define get_disconnects__impl ds18b20_onboard__get_disconnects__impl
#define get_update_id__impl ds18b20_onboard__get_update_id__impl
#define get_updated_deciseconds_ago__impl ds18b20_onboard__get_updated_deciseconds_ago__impl
    return ds18b20_onboard__crc_errors;
#undef get_crc_errors__impl
#undef get_disconnects__impl
#undef get_update_id__impl
#undef get_updated_deciseconds_ago__impl
}
static AKAT_FORCE_INLINE u16 ds18b20_onboard__get_temperatureX16__impl() {
#define get_crc_errors__impl ds18b20_onboard__get_crc_errors__impl
#define get_disconnects__impl ds18b20_onboard__get_disconnects__impl
#define get_temperatureX16__impl ds18b20_onboard__get_temperatureX16__impl
#define get_update_id__impl ds18b20_onboard__get_update_id__impl
#define get_updated_deciseconds_ago__impl ds18b20_onboard__get_updated_deciseconds_ago__impl
    return ds18b20_onboard__temperatureX16;
#undef get_crc_errors__impl
#undef get_disconnects__impl
#undef get_temperatureX16__impl
#undef get_update_id__impl
#undef get_updated_deciseconds_ago__impl
}
#define get_crc_errors__impl ds18b20_onboard__get_crc_errors__impl
#define get_disconnects__impl ds18b20_onboard__get_disconnects__impl
#define get_temperatureX16__impl ds18b20_onboard__get_temperatureX16__impl
#define get_update_id__impl ds18b20_onboard__get_update_id__impl
#define get_updated_deciseconds_ago__impl ds18b20_onboard__get_updated_deciseconds_ago__impl

ds18b20_onboard_t const ds18b20_onboard = {.get_updated_deciseconds_ago = &get_updated_deciseconds_ago__impl
                                           ,
                                           .get_update_id = &get_update_id__impl
                                                   ,
                                           .get_disconnects = &get_disconnects__impl
                                                   ,
                                           .get_crc_errors = &get_crc_errors__impl
                                                   ,
                                           .get_temperatureX16 = &get_temperatureX16__impl
                                          };


#undef get_crc_errors__impl
#undef get_disconnects__impl
#undef get_temperatureX16__impl
#undef get_update_id__impl
#undef get_updated_deciseconds_ago__impl
#define get_crc_errors__impl ds18b20_onboard__get_crc_errors__impl
#define get_disconnects__impl ds18b20_onboard__get_disconnects__impl
#define get_temperatureX16__impl ds18b20_onboard__get_temperatureX16__impl
#define get_update_id__impl ds18b20_onboard__get_update_id__impl
#define get_updated_deciseconds_ago__impl ds18b20_onboard__get_updated_deciseconds_ago__impl


;

#define get_crc_errors__impl ds18b20_onboard__get_crc_errors__impl
#define get_disconnects__impl ds18b20_onboard__get_disconnects__impl
#define get_temperatureX16__impl ds18b20_onboard__get_temperatureX16__impl
#define get_update_id__impl ds18b20_onboard__get_update_id__impl
#define get_updated_deciseconds_ago__impl ds18b20_onboard__get_updated_deciseconds_ago__impl








#undef get_crc_errors__impl
#undef get_disconnects__impl
#undef get_temperatureX16__impl
#undef get_update_id__impl
#undef get_updated_deciseconds_ago__impl
;



;
// Read temperature from DS18B20.
// DS18B20 is supposed to be connected to the given port and must be properly powered.
// (parasitic powering mode is not supported/tested).
// DS18B20 must be the only device connected to the pin because we use SKIP-ROM command.


typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} ds18b20_ext__pin__port_t;

extern ds18b20_ext__pin__port_t const ds18b20_ext__pin__port;

static AKAT_FORCE_INLINE void ds18b20_ext__pin__port__set__impl(u8 state) {
#define set__impl ds18b20_ext__pin__port__set__impl

    if (state) {
        PORTB |= 1 << 0;  //Set PORTB of B0 to 1
    } else {
        PORTB &= ~(1 << 0);  //Set PORTB of B0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 ds18b20_ext__pin__port__is_set__impl() {
#define is_set__impl ds18b20_ext__pin__port__is_set__impl
#define set__impl ds18b20_ext__pin__port__set__impl
    return PORTB & (1 << 0);  //Get value of PORTB for B0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl ds18b20_ext__pin__port__is_set__impl
#define set__impl ds18b20_ext__pin__port__set__impl

ds18b20_ext__pin__port_t const ds18b20_ext__pin__port = {.set = &set__impl
                                                         ,
                                                         .is_set = &is_set__impl
                                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl ds18b20_ext__pin__port__is_set__impl
#define set__impl ds18b20_ext__pin__port__set__impl


;

#define is_set__impl ds18b20_ext__pin__port__is_set__impl
#define set__impl ds18b20_ext__pin__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} ds18b20_ext__pin__ddr_t;

extern ds18b20_ext__pin__ddr_t const ds18b20_ext__pin__ddr;

static AKAT_FORCE_INLINE void ds18b20_ext__pin__ddr__set__impl(u8 state) {
#define set__impl ds18b20_ext__pin__ddr__set__impl

    if (state) {
        DDRB |= 1 << 0;  //Set DDRB of B0 to 1
    } else {
        DDRB &= ~(1 << 0);  //Set DDRB of B0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 ds18b20_ext__pin__ddr__is_set__impl() {
#define is_set__impl ds18b20_ext__pin__ddr__is_set__impl
#define set__impl ds18b20_ext__pin__ddr__set__impl
    return DDRB & (1 << 0);  //Get value of DDRB for B0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl ds18b20_ext__pin__ddr__is_set__impl
#define set__impl ds18b20_ext__pin__ddr__set__impl

ds18b20_ext__pin__ddr_t const ds18b20_ext__pin__ddr = {.set = &set__impl
                                                       ,
                                                       .is_set = &is_set__impl
                                                      };


#undef is_set__impl
#undef set__impl
#define is_set__impl ds18b20_ext__pin__ddr__is_set__impl
#define set__impl ds18b20_ext__pin__ddr__set__impl


;

#define is_set__impl ds18b20_ext__pin__ddr__is_set__impl
#define set__impl ds18b20_ext__pin__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} ds18b20_ext__pin__pin_t;

extern ds18b20_ext__pin__pin_t const ds18b20_ext__pin__pin;

static AKAT_FORCE_INLINE void ds18b20_ext__pin__pin__set__impl(u8 state) {
#define set__impl ds18b20_ext__pin__pin__set__impl

    if (state) {
        PINB |= 1 << 0;  //Set PINB of B0 to 1
    } else {
        PINB &= ~(1 << 0);  //Set PINB of B0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 ds18b20_ext__pin__pin__is_set__impl() {
#define is_set__impl ds18b20_ext__pin__pin__is_set__impl
#define set__impl ds18b20_ext__pin__pin__set__impl
    return PINB & (1 << 0);  //Get value of PINB for B0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl ds18b20_ext__pin__pin__is_set__impl
#define set__impl ds18b20_ext__pin__pin__set__impl

ds18b20_ext__pin__pin_t const ds18b20_ext__pin__pin = {.set = &set__impl
                                                       ,
                                                       .is_set = &is_set__impl
                                                      };


#undef is_set__impl
#undef set__impl
#define is_set__impl ds18b20_ext__pin__pin__is_set__impl
#define set__impl ds18b20_ext__pin__pin__set__impl


;

#define is_set__impl ds18b20_ext__pin__pin__is_set__impl
#define set__impl ds18b20_ext__pin__pin__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set_input_mode)();
    void (* const set_output_mode)();
    u8 (* const is_set)();
    void (* const set)(u8 state);
} ds18b20_ext__pin_t;

extern ds18b20_ext__pin_t const ds18b20_ext__pin;

static AKAT_FORCE_INLINE void ds18b20_ext__pin__set_input_mode__impl() {
#define set_input_mode__impl ds18b20_ext__pin__set_input_mode__impl
    ds18b20_ext__pin__ddr.set(0);
    ds18b20_ext__pin__port.set(1);
#undef set_input_mode__impl
}
static AKAT_FORCE_INLINE void ds18b20_ext__pin__set_output_mode__impl() {
#define set_input_mode__impl ds18b20_ext__pin__set_input_mode__impl
#define set_output_mode__impl ds18b20_ext__pin__set_output_mode__impl
    ds18b20_ext__pin__ddr.set(1);
#undef set_input_mode__impl
#undef set_output_mode__impl
}
static AKAT_FORCE_INLINE u8 ds18b20_ext__pin__is_set__impl() {
#define is_set__impl ds18b20_ext__pin__is_set__impl
#define set_input_mode__impl ds18b20_ext__pin__set_input_mode__impl
#define set_output_mode__impl ds18b20_ext__pin__set_output_mode__impl
    return ds18b20_ext__pin__pin.is_set();
#undef is_set__impl
#undef set_input_mode__impl
#undef set_output_mode__impl
}
static AKAT_FORCE_INLINE void ds18b20_ext__pin__set__impl(u8 state) {
#define is_set__impl ds18b20_ext__pin__is_set__impl
#define set__impl ds18b20_ext__pin__set__impl
#define set_input_mode__impl ds18b20_ext__pin__set_input_mode__impl
#define set_output_mode__impl ds18b20_ext__pin__set_output_mode__impl
    ds18b20_ext__pin__port.set(state);
#undef is_set__impl
#undef set__impl
#undef set_input_mode__impl
#undef set_output_mode__impl
}
#define is_set__impl ds18b20_ext__pin__is_set__impl
#define set__impl ds18b20_ext__pin__set__impl
#define set_input_mode__impl ds18b20_ext__pin__set_input_mode__impl
#define set_output_mode__impl ds18b20_ext__pin__set_output_mode__impl

ds18b20_ext__pin_t const ds18b20_ext__pin = {.set_input_mode = &set_input_mode__impl
                                             ,
                                             .set_output_mode = &set_output_mode__impl
                                                     ,
                                             .is_set = &is_set__impl
                                                     ,
                                             .set = &set__impl
                                            };


#undef is_set__impl
#undef set__impl
#undef set_input_mode__impl
#undef set_output_mode__impl
#define is_set__impl ds18b20_ext__pin__is_set__impl
#define set__impl ds18b20_ext__pin__set__impl
#define set_input_mode__impl ds18b20_ext__pin__set_input_mode__impl
#define set_output_mode__impl ds18b20_ext__pin__set_output_mode__impl


;

#define is_set__impl ds18b20_ext__pin__is_set__impl
#define set__impl ds18b20_ext__pin__set__impl
#define set_input_mode__impl ds18b20_ext__pin__set_input_mode__impl
#define set_output_mode__impl ds18b20_ext__pin__set_output_mode__impl







#undef is_set__impl
#undef set__impl
#undef set_input_mode__impl
#undef set_output_mode__impl
;



;

static AKAT_FORCE_INLINE void ds18b20_ext__init() {
//Safe state - input
    ds18b20_ext__pin.set_input_mode();
}

;





// Static variable for communication between our thread and other parts of code
static u8 ds18b20_ext__connected = 0;
static u8 ds18b20_ext__received_byte = 0;
static u8 ds18b20_ext__scratchpad[9] = {};
static u8 ds18b20_ext__update_id = 0;
static u8 ds18b20_ext__updated_deciseconds_ago = 255;
static u8 ds18b20_ext__crc_errors = 0;
static u8 ds18b20_ext__disconnects = 0;
static u16 ds18b20_ext__temperatureX16 = 0;

//Whether reset procedure detect presence pulse or not
;

//Last received byte
;

//Last scratchpad
;

//Statistics
;
;
;
;

//Temperature (must be divided by 16 to convert to degrees)
;
;



static AKAT_FORCE_INLINE void ds18b20_ext__ticker() {
//Maintain freshness
    ds18b20_ext__updated_deciseconds_ago += AKAT_ONE;

    if (!ds18b20_ext__updated_deciseconds_ago) {//We can't go beyond 255
        ds18b20_ext__updated_deciseconds_ago -= AKAT_ONE;
    }
}

;





typedef struct {
    u8 (* const get_updated_deciseconds_ago)();
    u8 (* const get_update_id)();
    u8 (* const get_disconnects)();
    u8 (* const get_crc_errors)();
    u16 (* const get_temperatureX16)();
} ds18b20_ext_t;

extern ds18b20_ext_t const ds18b20_ext;

static AKAT_FORCE_INLINE u8 ds18b20_ext__get_updated_deciseconds_ago__impl() {
#define get_updated_deciseconds_ago__impl ds18b20_ext__get_updated_deciseconds_ago__impl
    return ds18b20_ext__updated_deciseconds_ago;
#undef get_updated_deciseconds_ago__impl
}
static AKAT_FORCE_INLINE u8 ds18b20_ext__get_update_id__impl() {
#define get_update_id__impl ds18b20_ext__get_update_id__impl
#define get_updated_deciseconds_ago__impl ds18b20_ext__get_updated_deciseconds_ago__impl
    return ds18b20_ext__update_id;
#undef get_update_id__impl
#undef get_updated_deciseconds_ago__impl
}
static AKAT_FORCE_INLINE u8 ds18b20_ext__get_disconnects__impl() {
#define get_disconnects__impl ds18b20_ext__get_disconnects__impl
#define get_update_id__impl ds18b20_ext__get_update_id__impl
#define get_updated_deciseconds_ago__impl ds18b20_ext__get_updated_deciseconds_ago__impl
    return ds18b20_ext__disconnects;
#undef get_disconnects__impl
#undef get_update_id__impl
#undef get_updated_deciseconds_ago__impl
}
static AKAT_FORCE_INLINE u8 ds18b20_ext__get_crc_errors__impl() {
#define get_crc_errors__impl ds18b20_ext__get_crc_errors__impl
#define get_disconnects__impl ds18b20_ext__get_disconnects__impl
#define get_update_id__impl ds18b20_ext__get_update_id__impl
#define get_updated_deciseconds_ago__impl ds18b20_ext__get_updated_deciseconds_ago__impl
    return ds18b20_ext__crc_errors;
#undef get_crc_errors__impl
#undef get_disconnects__impl
#undef get_update_id__impl
#undef get_updated_deciseconds_ago__impl
}
static AKAT_FORCE_INLINE u16 ds18b20_ext__get_temperatureX16__impl() {
#define get_crc_errors__impl ds18b20_ext__get_crc_errors__impl
#define get_disconnects__impl ds18b20_ext__get_disconnects__impl
#define get_temperatureX16__impl ds18b20_ext__get_temperatureX16__impl
#define get_update_id__impl ds18b20_ext__get_update_id__impl
#define get_updated_deciseconds_ago__impl ds18b20_ext__get_updated_deciseconds_ago__impl
    return ds18b20_ext__temperatureX16;
#undef get_crc_errors__impl
#undef get_disconnects__impl
#undef get_temperatureX16__impl
#undef get_update_id__impl
#undef get_updated_deciseconds_ago__impl
}
#define get_crc_errors__impl ds18b20_ext__get_crc_errors__impl
#define get_disconnects__impl ds18b20_ext__get_disconnects__impl
#define get_temperatureX16__impl ds18b20_ext__get_temperatureX16__impl
#define get_update_id__impl ds18b20_ext__get_update_id__impl
#define get_updated_deciseconds_ago__impl ds18b20_ext__get_updated_deciseconds_ago__impl

ds18b20_ext_t const ds18b20_ext = {.get_updated_deciseconds_ago = &get_updated_deciseconds_ago__impl
                                   ,
                                   .get_update_id = &get_update_id__impl
                                           ,
                                   .get_disconnects = &get_disconnects__impl
                                           ,
                                   .get_crc_errors = &get_crc_errors__impl
                                           ,
                                   .get_temperatureX16 = &get_temperatureX16__impl
                                  };


#undef get_crc_errors__impl
#undef get_disconnects__impl
#undef get_temperatureX16__impl
#undef get_update_id__impl
#undef get_updated_deciseconds_ago__impl
#define get_crc_errors__impl ds18b20_ext__get_crc_errors__impl
#define get_disconnects__impl ds18b20_ext__get_disconnects__impl
#define get_temperatureX16__impl ds18b20_ext__get_temperatureX16__impl
#define get_update_id__impl ds18b20_ext__get_update_id__impl
#define get_updated_deciseconds_ago__impl ds18b20_ext__get_updated_deciseconds_ago__impl


;

#define get_crc_errors__impl ds18b20_ext__get_crc_errors__impl
#define get_disconnects__impl ds18b20_ext__get_disconnects__impl
#define get_temperatureX16__impl ds18b20_ext__get_temperatureX16__impl
#define get_update_id__impl ds18b20_ext__get_update_id__impl
#define get_updated_deciseconds_ago__impl ds18b20_ext__get_updated_deciseconds_ago__impl








#undef get_crc_errors__impl
#undef get_disconnects__impl
#undef get_temperatureX16__impl
#undef get_update_id__impl
#undef get_updated_deciseconds_ago__impl
;



;

// Let pins
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} board_led__port_t;

extern board_led__port_t const board_led__port;

static AKAT_FORCE_INLINE void board_led__port__set__impl(u8 state) {
#define set__impl board_led__port__set__impl

    if (state) {
        PORTB |= 1 << 5;  //Set PORTB of B5 to 1
    } else {
        PORTB &= ~(1 << 5);  //Set PORTB of B5 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 board_led__port__is_set__impl() {
#define is_set__impl board_led__port__is_set__impl
#define set__impl board_led__port__set__impl
    return PORTB & (1 << 5);  //Get value of PORTB for B5
#undef is_set__impl
#undef set__impl
}
#define is_set__impl board_led__port__is_set__impl
#define set__impl board_led__port__set__impl

board_led__port_t const board_led__port = {.set = &set__impl
                                           ,
                                           .is_set = &is_set__impl
                                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl board_led__port__is_set__impl
#define set__impl board_led__port__set__impl


;

#define is_set__impl board_led__port__is_set__impl
#define set__impl board_led__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} board_led__ddr_t;

extern board_led__ddr_t const board_led__ddr;

static AKAT_FORCE_INLINE void board_led__ddr__set__impl(u8 state) {
#define set__impl board_led__ddr__set__impl

    if (state) {
        DDRB |= 1 << 5;  //Set DDRB of B5 to 1
    } else {
        DDRB &= ~(1 << 5);  //Set DDRB of B5 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 board_led__ddr__is_set__impl() {
#define is_set__impl board_led__ddr__is_set__impl
#define set__impl board_led__ddr__set__impl
    return DDRB & (1 << 5);  //Get value of DDRB for B5
#undef is_set__impl
#undef set__impl
}
#define is_set__impl board_led__ddr__is_set__impl
#define set__impl board_led__ddr__set__impl

board_led__ddr_t const board_led__ddr = {.set = &set__impl
                                         ,
                                         .is_set = &is_set__impl
                                        };


#undef is_set__impl
#undef set__impl
#define is_set__impl board_led__ddr__is_set__impl
#define set__impl board_led__ddr__set__impl


;

#define is_set__impl board_led__ddr__is_set__impl
#define set__impl board_led__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void board_led__init() {
    board_led__ddr.set(1); //Init B5 as output
}

;





typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} board_led_t;

extern board_led_t const board_led;

static AKAT_FORCE_INLINE void board_led__set__impl(u8 state) {
#define set__impl board_led__set__impl
    board_led__port.set(state);
#undef set__impl
}
static AKAT_FORCE_INLINE u8 board_led__is_set__impl() {
#define is_set__impl board_led__is_set__impl
#define set__impl board_led__set__impl
    return board_led__port.is_set();
#undef is_set__impl
#undef set__impl
}
#define is_set__impl board_led__is_set__impl
#define set__impl board_led__set__impl

board_led_t const board_led = {.set = &set__impl
                                      ,
                               .is_set = &is_set__impl
                              };


#undef is_set__impl
#undef set__impl
#define is_set__impl board_led__is_set__impl
#define set__impl board_led__set__impl


;

#define is_set__impl board_led__is_set__impl
#define set__impl board_led__set__impl





#undef is_set__impl
#undef set__impl
;



;
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} hdd_led__port_t;

extern hdd_led__port_t const hdd_led__port;

static AKAT_FORCE_INLINE void hdd_led__port__set__impl(u8 state) {
#define set__impl hdd_led__port__set__impl

    if (state) {
        PORTB |= 1 << 3;  //Set PORTB of B3 to 1
    } else {
        PORTB &= ~(1 << 3);  //Set PORTB of B3 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 hdd_led__port__is_set__impl() {
#define is_set__impl hdd_led__port__is_set__impl
#define set__impl hdd_led__port__set__impl
    return PORTB & (1 << 3);  //Get value of PORTB for B3
#undef is_set__impl
#undef set__impl
}
#define is_set__impl hdd_led__port__is_set__impl
#define set__impl hdd_led__port__set__impl

hdd_led__port_t const hdd_led__port = {.set = &set__impl
                                       ,
                                       .is_set = &is_set__impl
                                      };


#undef is_set__impl
#undef set__impl
#define is_set__impl hdd_led__port__is_set__impl
#define set__impl hdd_led__port__set__impl


;

#define is_set__impl hdd_led__port__is_set__impl
#define set__impl hdd_led__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} hdd_led__ddr_t;

extern hdd_led__ddr_t const hdd_led__ddr;

static AKAT_FORCE_INLINE void hdd_led__ddr__set__impl(u8 state) {
#define set__impl hdd_led__ddr__set__impl

    if (state) {
        DDRB |= 1 << 3;  //Set DDRB of B3 to 1
    } else {
        DDRB &= ~(1 << 3);  //Set DDRB of B3 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 hdd_led__ddr__is_set__impl() {
#define is_set__impl hdd_led__ddr__is_set__impl
#define set__impl hdd_led__ddr__set__impl
    return DDRB & (1 << 3);  //Get value of DDRB for B3
#undef is_set__impl
#undef set__impl
}
#define is_set__impl hdd_led__ddr__is_set__impl
#define set__impl hdd_led__ddr__set__impl

hdd_led__ddr_t const hdd_led__ddr = {.set = &set__impl
                                     ,
                                     .is_set = &is_set__impl
                                    };


#undef is_set__impl
#undef set__impl
#define is_set__impl hdd_led__ddr__is_set__impl
#define set__impl hdd_led__ddr__set__impl


;

#define is_set__impl hdd_led__ddr__is_set__impl
#define set__impl hdd_led__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void hdd_led__init() {
    hdd_led__ddr.set(1); //Init B3 as output
}

;





typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} hdd_led_t;

extern hdd_led_t const hdd_led;

static AKAT_FORCE_INLINE void hdd_led__set__impl(u8 state) {
#define set__impl hdd_led__set__impl
    hdd_led__port.set(state);
#undef set__impl
}
static AKAT_FORCE_INLINE u8 hdd_led__is_set__impl() {
#define is_set__impl hdd_led__is_set__impl
#define set__impl hdd_led__set__impl
    return hdd_led__port.is_set();
#undef is_set__impl
#undef set__impl
}
#define is_set__impl hdd_led__is_set__impl
#define set__impl hdd_led__set__impl

hdd_led_t const hdd_led = {.set = &set__impl
                                  ,
                           .is_set = &is_set__impl
                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl hdd_led__is_set__impl
#define set__impl hdd_led__set__impl


;

#define is_set__impl hdd_led__is_set__impl
#define set__impl hdd_led__set__impl





#undef is_set__impl
#undef set__impl
;



;
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} pwr_led__port_t;

extern pwr_led__port_t const pwr_led__port;

static AKAT_FORCE_INLINE void pwr_led__port__set__impl(u8 state) {
#define set__impl pwr_led__port__set__impl

    if (state) {
        PORTB |= 1 << 4;  //Set PORTB of B4 to 1
    } else {
        PORTB &= ~(1 << 4);  //Set PORTB of B4 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 pwr_led__port__is_set__impl() {
#define is_set__impl pwr_led__port__is_set__impl
#define set__impl pwr_led__port__set__impl
    return PORTB & (1 << 4);  //Get value of PORTB for B4
#undef is_set__impl
#undef set__impl
}
#define is_set__impl pwr_led__port__is_set__impl
#define set__impl pwr_led__port__set__impl

pwr_led__port_t const pwr_led__port = {.set = &set__impl
                                       ,
                                       .is_set = &is_set__impl
                                      };


#undef is_set__impl
#undef set__impl
#define is_set__impl pwr_led__port__is_set__impl
#define set__impl pwr_led__port__set__impl


;

#define is_set__impl pwr_led__port__is_set__impl
#define set__impl pwr_led__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} pwr_led__ddr_t;

extern pwr_led__ddr_t const pwr_led__ddr;

static AKAT_FORCE_INLINE void pwr_led__ddr__set__impl(u8 state) {
#define set__impl pwr_led__ddr__set__impl

    if (state) {
        DDRB |= 1 << 4;  //Set DDRB of B4 to 1
    } else {
        DDRB &= ~(1 << 4);  //Set DDRB of B4 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 pwr_led__ddr__is_set__impl() {
#define is_set__impl pwr_led__ddr__is_set__impl
#define set__impl pwr_led__ddr__set__impl
    return DDRB & (1 << 4);  //Get value of DDRB for B4
#undef is_set__impl
#undef set__impl
}
#define is_set__impl pwr_led__ddr__is_set__impl
#define set__impl pwr_led__ddr__set__impl

pwr_led__ddr_t const pwr_led__ddr = {.set = &set__impl
                                     ,
                                     .is_set = &is_set__impl
                                    };


#undef is_set__impl
#undef set__impl
#define is_set__impl pwr_led__ddr__is_set__impl
#define set__impl pwr_led__ddr__set__impl


;

#define is_set__impl pwr_led__ddr__is_set__impl
#define set__impl pwr_led__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void pwr_led__init() {
    pwr_led__ddr.set(1); //Init B4 as output
}

;





typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} pwr_led_t;

extern pwr_led_t const pwr_led;

static AKAT_FORCE_INLINE void pwr_led__set__impl(u8 state) {
#define set__impl pwr_led__set__impl
    pwr_led__port.set(state);
#undef set__impl
}
static AKAT_FORCE_INLINE u8 pwr_led__is_set__impl() {
#define is_set__impl pwr_led__is_set__impl
#define set__impl pwr_led__set__impl
    return pwr_led__port.is_set();
#undef is_set__impl
#undef set__impl
}
#define is_set__impl pwr_led__is_set__impl
#define set__impl pwr_led__set__impl

pwr_led_t const pwr_led = {.set = &set__impl
                                  ,
                           .is_set = &is_set__impl
                          };


#undef is_set__impl
#undef set__impl
#define is_set__impl pwr_led__is_set__impl
#define set__impl pwr_led__set__impl


;

#define is_set__impl pwr_led__is_set__impl
#define set__impl pwr_led__set__impl





#undef is_set__impl
#undef set__impl
;



;

// ATX PS ON Pin, the one that turns ATX on or OFF
// Settings this to output and ZERO will turn ATX ON
typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} ps_on__port_t;

extern ps_on__port_t const ps_on__port;

static AKAT_FORCE_INLINE void ps_on__port__set__impl(u8 state) {
#define set__impl ps_on__port__set__impl

    if (state) {
        PORTC |= 1 << 0;  //Set PORTC of C0 to 1
    } else {
        PORTC &= ~(1 << 0);  //Set PORTC of C0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 ps_on__port__is_set__impl() {
#define is_set__impl ps_on__port__is_set__impl
#define set__impl ps_on__port__set__impl
    return PORTC & (1 << 0);  //Get value of PORTC for C0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl ps_on__port__is_set__impl
#define set__impl ps_on__port__set__impl

ps_on__port_t const ps_on__port = {.set = &set__impl
                                   ,
                                   .is_set = &is_set__impl
                                  };


#undef is_set__impl
#undef set__impl
#define is_set__impl ps_on__port__is_set__impl
#define set__impl ps_on__port__set__impl


;

#define is_set__impl ps_on__port__is_set__impl
#define set__impl ps_on__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} ps_on__ddr_t;

extern ps_on__ddr_t const ps_on__ddr;

static AKAT_FORCE_INLINE void ps_on__ddr__set__impl(u8 state) {
#define set__impl ps_on__ddr__set__impl

    if (state) {
        DDRC |= 1 << 0;  //Set DDRC of C0 to 1
    } else {
        DDRC &= ~(1 << 0);  //Set DDRC of C0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 ps_on__ddr__is_set__impl() {
#define is_set__impl ps_on__ddr__is_set__impl
#define set__impl ps_on__ddr__set__impl
    return DDRC & (1 << 0);  //Get value of DDRC for C0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl ps_on__ddr__is_set__impl
#define set__impl ps_on__ddr__set__impl

ps_on__ddr_t const ps_on__ddr = {.set = &set__impl
                                        ,
                                 .is_set = &is_set__impl
                                };


#undef is_set__impl
#undef set__impl
#define is_set__impl ps_on__ddr__is_set__impl
#define set__impl ps_on__ddr__set__impl


;

#define is_set__impl ps_on__ddr__is_set__impl
#define set__impl ps_on__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} ps_on__pin_t;

extern ps_on__pin_t const ps_on__pin;

static AKAT_FORCE_INLINE void ps_on__pin__set__impl(u8 state) {
#define set__impl ps_on__pin__set__impl

    if (state) {
        PINC |= 1 << 0;  //Set PINC of C0 to 1
    } else {
        PINC &= ~(1 << 0);  //Set PINC of C0 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 ps_on__pin__is_set__impl() {
#define is_set__impl ps_on__pin__is_set__impl
#define set__impl ps_on__pin__set__impl
    return PINC & (1 << 0);  //Get value of PINC for C0
#undef is_set__impl
#undef set__impl
}
#define is_set__impl ps_on__pin__is_set__impl
#define set__impl ps_on__pin__set__impl

ps_on__pin_t const ps_on__pin = {.set = &set__impl
                                        ,
                                 .is_set = &is_set__impl
                                };


#undef is_set__impl
#undef set__impl
#define is_set__impl ps_on__pin__is_set__impl
#define set__impl ps_on__pin__set__impl


;

#define is_set__impl ps_on__pin__is_set__impl
#define set__impl ps_on__pin__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set_input_mode)();
    void (* const set_output_mode)();
    u8 (* const is_set)();
    void (* const set)(u8 state);
} ps_on_t;

extern ps_on_t const ps_on;

static AKAT_FORCE_INLINE void ps_on__set_input_mode__impl() {
#define set_input_mode__impl ps_on__set_input_mode__impl
    ps_on__ddr.set(0);
    ps_on__port.set(1);
#undef set_input_mode__impl
}
static AKAT_FORCE_INLINE void ps_on__set_output_mode__impl() {
#define set_input_mode__impl ps_on__set_input_mode__impl
#define set_output_mode__impl ps_on__set_output_mode__impl
    ps_on__ddr.set(1);
#undef set_input_mode__impl
#undef set_output_mode__impl
}
static AKAT_FORCE_INLINE u8 ps_on__is_set__impl() {
#define is_set__impl ps_on__is_set__impl
#define set_input_mode__impl ps_on__set_input_mode__impl
#define set_output_mode__impl ps_on__set_output_mode__impl
    return ps_on__pin.is_set();
#undef is_set__impl
#undef set_input_mode__impl
#undef set_output_mode__impl
}
static AKAT_FORCE_INLINE void ps_on__set__impl(u8 state) {
#define is_set__impl ps_on__is_set__impl
#define set__impl ps_on__set__impl
#define set_input_mode__impl ps_on__set_input_mode__impl
#define set_output_mode__impl ps_on__set_output_mode__impl
    ps_on__port.set(state);
#undef is_set__impl
#undef set__impl
#undef set_input_mode__impl
#undef set_output_mode__impl
}
#define is_set__impl ps_on__is_set__impl
#define set__impl ps_on__set__impl
#define set_input_mode__impl ps_on__set_input_mode__impl
#define set_output_mode__impl ps_on__set_output_mode__impl

ps_on_t const ps_on = {.set_input_mode = &set_input_mode__impl
                       ,
                       .set_output_mode = &set_output_mode__impl
                               ,
                       .is_set = &is_set__impl
                                 ,
                       .set = &set__impl
                      };


#undef is_set__impl
#undef set__impl
#undef set_input_mode__impl
#undef set_output_mode__impl
#define is_set__impl ps_on__is_set__impl
#define set__impl ps_on__set__impl
#define set_input_mode__impl ps_on__set_input_mode__impl
#define set_output_mode__impl ps_on__set_output_mode__impl


;

#define is_set__impl ps_on__is_set__impl
#define set__impl ps_on__set__impl
#define set_input_mode__impl ps_on__set_input_mode__impl
#define set_output_mode__impl ps_on__set_output_mode__impl







#undef is_set__impl
#undef set__impl
#undef set_input_mode__impl
#undef set_output_mode__impl
;



;

// Just maintain our state on every iteration of event loop.
// Might be unnecessary but safe and simple for sure.
static AKAT_FORCE_INLINE void ps_on_mainenance() {
    if (power_should_be_on) {//Drop pin to ground indicating that we want ATX to be in ON state.
        ps_on.set_output_mode();
        ps_on.set(0);
    } else {//Disconnect from pin. Pin will be pulled up to 5v by ATX itself
        //and ATX will turn it off it was ON.
        ps_on.set_input_mode();
    }
}

;



;

// ATX POWER BUTTON
typedef struct {
    void (* const on_release)();
    void (* const on_press)();
    void (* const on_long_press)();
} pwr_button_t;

extern pwr_button_t const pwr_button;

static void pwr_button__on_release__impl() {
#define on_release__impl pwr_button__on_release__impl
#undef on_release__impl
}
static void pwr_button__on_press__impl() {
#define on_press__impl pwr_button__on_press__impl
#define on_release__impl pwr_button__on_release__impl
//Can turn on with a single press
    //Can't turn off with a short press.
    power_should_be_on = AKAT_ONE;
#undef on_press__impl
#undef on_release__impl
}
static void pwr_button__on_long_press__impl() {
#define on_long_press__impl pwr_button__on_long_press__impl
#define on_press__impl pwr_button__on_press__impl
#define on_release__impl pwr_button__on_release__impl
//Can both turn on and turn off with a long press
    power_should_be_on = !power_should_be_on;
#undef on_long_press__impl
#undef on_press__impl
#undef on_release__impl
}
#define on_long_press__impl pwr_button__on_long_press__impl
#define on_press__impl pwr_button__on_press__impl
#define on_release__impl pwr_button__on_release__impl

pwr_button_t const pwr_button = {.on_release = &on_release__impl
                                 ,
                                 .on_press = &on_press__impl
                                         ,
                                 .on_long_press = &on_long_press__impl
                                };


#undef on_long_press__impl
#undef on_press__impl
#undef on_release__impl
#define on_long_press__impl pwr_button__on_long_press__impl
#define on_press__impl pwr_button__on_press__impl
#define on_release__impl pwr_button__on_release__impl


;

#define on_long_press__impl pwr_button__on_long_press__impl
#define on_press__impl pwr_button__on_press__impl
#define on_release__impl pwr_button__on_release__impl






#undef on_long_press__impl
#undef on_press__impl
#undef on_release__impl
;




static u8 pwr_button__delay = 0;

;
;


typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} pwr_button__button_full__input__port_t;

extern pwr_button__button_full__input__port_t const pwr_button__button_full__input__port;

static AKAT_FORCE_INLINE void pwr_button__button_full__input__port__set__impl(u8 state) {
#define set__impl pwr_button__button_full__input__port__set__impl

    if (state) {
        PORTB |= 1 << 1;  //Set PORTB of B1 to 1
    } else {
        PORTB &= ~(1 << 1);  //Set PORTB of B1 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 pwr_button__button_full__input__port__is_set__impl() {
#define is_set__impl pwr_button__button_full__input__port__is_set__impl
#define set__impl pwr_button__button_full__input__port__set__impl
    return PORTB & (1 << 1);  //Get value of PORTB for B1
#undef is_set__impl
#undef set__impl
}
#define is_set__impl pwr_button__button_full__input__port__is_set__impl
#define set__impl pwr_button__button_full__input__port__set__impl

pwr_button__button_full__input__port_t const pwr_button__button_full__input__port = {.set = &set__impl
                                                                                     ,
                                                                                     .is_set = &is_set__impl
                                                                                    };


#undef is_set__impl
#undef set__impl
#define is_set__impl pwr_button__button_full__input__port__is_set__impl
#define set__impl pwr_button__button_full__input__port__set__impl


;

#define is_set__impl pwr_button__button_full__input__port__is_set__impl
#define set__impl pwr_button__button_full__input__port__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} pwr_button__button_full__input__ddr_t;

extern pwr_button__button_full__input__ddr_t const pwr_button__button_full__input__ddr;

static AKAT_FORCE_INLINE void pwr_button__button_full__input__ddr__set__impl(u8 state) {
#define set__impl pwr_button__button_full__input__ddr__set__impl

    if (state) {
        DDRB |= 1 << 1;  //Set DDRB of B1 to 1
    } else {
        DDRB &= ~(1 << 1);  //Set DDRB of B1 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 pwr_button__button_full__input__ddr__is_set__impl() {
#define is_set__impl pwr_button__button_full__input__ddr__is_set__impl
#define set__impl pwr_button__button_full__input__ddr__set__impl
    return DDRB & (1 << 1);  //Get value of DDRB for B1
#undef is_set__impl
#undef set__impl
}
#define is_set__impl pwr_button__button_full__input__ddr__is_set__impl
#define set__impl pwr_button__button_full__input__ddr__set__impl

pwr_button__button_full__input__ddr_t const pwr_button__button_full__input__ddr = {.set = &set__impl
                                                                                   ,
                                                                                   .is_set = &is_set__impl
                                                                                  };


#undef is_set__impl
#undef set__impl
#define is_set__impl pwr_button__button_full__input__ddr__is_set__impl
#define set__impl pwr_button__button_full__input__ddr__set__impl


;

#define is_set__impl pwr_button__button_full__input__ddr__is_set__impl
#define set__impl pwr_button__button_full__input__ddr__set__impl





#undef is_set__impl
#undef set__impl
;



typedef struct {
    void (* const set)(u8 state);
    u8 (* const is_set)();
} pwr_button__button_full__input__pin_t;

extern pwr_button__button_full__input__pin_t const pwr_button__button_full__input__pin;

static AKAT_FORCE_INLINE void pwr_button__button_full__input__pin__set__impl(u8 state) {
#define set__impl pwr_button__button_full__input__pin__set__impl

    if (state) {
        PINB |= 1 << 1;  //Set PINB of B1 to 1
    } else {
        PINB &= ~(1 << 1);  //Set PINB of B1 to 0
    }

#undef set__impl
}
static AKAT_FORCE_INLINE u8 pwr_button__button_full__input__pin__is_set__impl() {
#define is_set__impl pwr_button__button_full__input__pin__is_set__impl
#define set__impl pwr_button__button_full__input__pin__set__impl
    return PINB & (1 << 1);  //Get value of PINB for B1
#undef is_set__impl
#undef set__impl
}
#define is_set__impl pwr_button__button_full__input__pin__is_set__impl
#define set__impl pwr_button__button_full__input__pin__set__impl

pwr_button__button_full__input__pin_t const pwr_button__button_full__input__pin = {.set = &set__impl
                                                                                   ,
                                                                                   .is_set = &is_set__impl
                                                                                  };


#undef is_set__impl
#undef set__impl
#define is_set__impl pwr_button__button_full__input__pin__is_set__impl
#define set__impl pwr_button__button_full__input__pin__set__impl


;

#define is_set__impl pwr_button__button_full__input__pin__is_set__impl
#define set__impl pwr_button__button_full__input__pin__set__impl





#undef is_set__impl
#undef set__impl
;



static AKAT_FORCE_INLINE void pwr_button__button_full__input__init() {
    pwr_button__button_full__input__ddr.set(0);
    pwr_button__button_full__input__port.set(1);
}

;





typedef struct {
    u8 (* const is_set)();
} pwr_button__button_full__input_t;

extern pwr_button__button_full__input_t const pwr_button__button_full__input;

static AKAT_FORCE_INLINE u8 pwr_button__button_full__input__is_set__impl() {
#define is_set__impl pwr_button__button_full__input__is_set__impl
    return pwr_button__button_full__input__pin.is_set();
#undef is_set__impl
}
#define is_set__impl pwr_button__button_full__input__is_set__impl

pwr_button__button_full__input_t const pwr_button__button_full__input = {.is_set = &is_set__impl
                                                                        };


#undef is_set__impl
#define is_set__impl pwr_button__button_full__input__is_set__impl


;

#define is_set__impl pwr_button__button_full__input__is_set__impl




#undef is_set__impl
;



static akat_x_button_state_t pwr_button__button_full__state;

static AKAT_FORCE_INLINE void pwr_button__button_full__init() {
    pwr_button__button_full__state.awaiting_key_press = AKAT_TRUE;
    pwr_button__button_full__state.checks_left = AKAT_X_BUTTON_CHECKS;
}

;





typedef struct {
    u8 (* const is_awaiting_key_press)();
    void (* const on_press)();
    void (* const on_release)();
} pwr_button__button_full_t;

extern pwr_button__button_full_t const pwr_button__button_full;

static AKAT_FORCE_INLINE u8 pwr_button__button_full__is_awaiting_key_press__impl() {
#define is_awaiting_key_press__impl pwr_button__button_full__is_awaiting_key_press__impl
    return pwr_button__button_full__state.awaiting_key_press;
#undef is_awaiting_key_press__impl
}
static void pwr_button__button_full__on_press__impl() {
#define is_awaiting_key_press__impl pwr_button__button_full__is_awaiting_key_press__impl
#define on_press__impl pwr_button__button_full__on_press__impl
    pwr_button__delay = 40;
#undef is_awaiting_key_press__impl
#undef on_press__impl
}
static void pwr_button__button_full__on_release__impl() {
#define is_awaiting_key_press__impl pwr_button__button_full__is_awaiting_key_press__impl
#define on_press__impl pwr_button__button_full__on_press__impl
#define on_release__impl pwr_button__button_full__on_release__impl

    if (pwr_button__delay) {
        pwr_button.on_press();
    }

    pwr_button.on_release();
#undef is_awaiting_key_press__impl
#undef on_press__impl
#undef on_release__impl
}
#define is_awaiting_key_press__impl pwr_button__button_full__is_awaiting_key_press__impl
#define on_press__impl pwr_button__button_full__on_press__impl
#define on_release__impl pwr_button__button_full__on_release__impl

pwr_button__button_full_t const pwr_button__button_full = {.is_awaiting_key_press = &is_awaiting_key_press__impl
                                                           ,
                                                           .on_press = &on_press__impl
                                                                   ,
                                                           .on_release = &on_release__impl
                                                          };


#undef is_awaiting_key_press__impl
#undef on_press__impl
#undef on_release__impl
#define is_awaiting_key_press__impl pwr_button__button_full__is_awaiting_key_press__impl
#define on_press__impl pwr_button__button_full__on_press__impl
#define on_release__impl pwr_button__button_full__on_release__impl


;

#define is_awaiting_key_press__impl pwr_button__button_full__is_awaiting_key_press__impl
#define on_press__impl pwr_button__button_full__on_press__impl
#define on_release__impl pwr_button__button_full__on_release__impl






#undef is_awaiting_key_press__impl
#undef on_press__impl
#undef on_release__impl
;




static AKAT_FORCE_INLINE void pwr_button__button_full__runnable() {
    akat_x_button_action_t const rc = akat_x_button_handle_pin_state(&pwr_button__button_full__state, pwr_button__button_full__input.is_set());

    if (rc == AKAT_X_BUTTON_ACTION_KEYPRESS) {
        pwr_button__button_full.on_press();
    } else if (rc == AKAT_X_BUTTON_ACTION_KEYRELEASE) {
        pwr_button__button_full.on_release();
    }
}

;







static AKAT_FORCE_INLINE void pwr_button__ticker() {
    if (!pwr_button__button_full.is_awaiting_key_press()) {
        if (pwr_button__delay) {
            pwr_button__delay--;

            if (pwr_button__delay == 0) {
                pwr_button.on_long_press();
            }
        }
    }
}

;




;

// Activity indication

static u8 activity_led__counter = 0;
static AKAT_FORCE_INLINE void activity_led() {
#define counter activity_led__counter
    ;
    board_led.set(counter % 2);

    if (power_should_be_on) {
        hdd_led.set(counter % 2);
        pwr_led.set(1);
    } else {
        hdd_led.set(0);
        pwr_led.set(counter < 10);
    }

    counter += 1;

    if (counter >= 30) {
        counter = 0;
    }

#undef counter
}

;





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
// USART0 - Serial interface over USB Connection

static AKAT_FORCE_INLINE void usart0_init() {
//Set baud rate
    const u16 ubrr = akat_cpu_freq_hz() / (AK_USART0_BAUD_RATE * 8L) - 1;
    UBRR0H = ubrr >> 8;
    UBRR0L = ubrr % 256;
    UCSR0A = H(U2X0);
    //Set frame format
    UCSR0C = AK_USART0_FRAME_FORMAT;
    //Enable transmitter, receiver and interrupt for receiver (interrupt for 'byte is received')
    UCSR0B = H(TXEN0) | H(RXEN0) | H(RXCIE0);
}

;





// ----------------------------------------------------------------
// USART0(USB): Interrupt handler for 'byte is received' event..

static volatile u8 usart0_rx_bytes_buf[AK_USART0_RX_BUF_SIZE] = {};
static volatile u8 usart0_rx_overflow_count = 0;
static volatile u8 usart0_rx_next_empty_idx = 0;
static volatile u8 usart0_rx_next_read_idx = 0;

;
;
;
;
;


ISR(USART_RX_vect) {
    u8 b = UDR0; // we must read here, no matter what, to clear interrupt flag
    u8 new_next_empty_idx = (usart0_rx_next_empty_idx + AKAT_ONE) & (AK_USART0_RX_BUF_SIZE - 1);

    if (new_next_empty_idx == usart0_rx_next_read_idx) {
        usart0_rx_overflow_count += AKAT_ONE;

        // Don't let it overflow!
        if (!usart0_rx_overflow_count) {
            usart0_rx_overflow_count -= AKAT_ONE;
        }
    } else {
        usart0_rx_bytes_buf[usart0_rx_next_empty_idx] = b;
        usart0_rx_next_empty_idx = new_next_empty_idx;
    }
}

// ----------------------------------------------------------------
// USART0(USB): This thread continuously writes current status into USART0

// NOTE: Just replace state_type to u16 if we ran out of state space..
register u8 usart0_writer__akat_coroutine_state asm ("r17");
static u8 usart0_writer__crc = 0;
register u8 usart0_writer__byte_to_send asm ("r16");
register u8 usart0_writer__u8_to_format_and_send asm ("r18");
static u16 usart0_writer__u16_to_format_and_send = 0;
static u32 usart0_writer__u32_to_format_and_send = 0;
register u8 usart0_writer__send_byte__akat_coroutine_state asm ("r4");
static u8 usart0_writer__send_byte() {
#define akat_coroutine_state usart0_writer__send_byte__akat_coroutine_state
#define byte_to_send usart0_writer__byte_to_send
#define crc usart0_writer__crc
#define send_byte usart0_writer__send_byte
#define u16_to_format_and_send usart0_writer__u16_to_format_and_send
#define u32_to_format_and_send usart0_writer__u32_to_format_and_send
#define u8_to_format_and_send usart0_writer__u8_to_format_and_send
    ;
    AKAT_HOT_CODE;

    switch (akat_coroutine_state) {
    case AKAT_COROUTINE_S_START:
        goto akat_coroutine_l_start;

    case AKAT_COROUTINE_S_END:
        goto akat_coroutine_l_end;

    case 2:
        goto akat_coroutine_l_2;
    }

akat_coroutine_l_start:
    AKAT_COLD_CODE;

    do {
        //Wait until USART0 is ready to transmit next byte
        //from 'byte_to_send';
        do {
            akat_coroutine_state = 2;
akat_coroutine_l_2:

            if (!(UCSR0A & H(UDRE0))) {
                AKAT_HOT_CODE;
                return akat_coroutine_state;
            }
        } while (0);

        ;
        UDR0 = byte_to_send;
        crc = akat_crc_add(crc, byte_to_send);
    } while (0);

    AKAT_COLD_CODE;
    akat_coroutine_state = AKAT_COROUTINE_S_START;
akat_coroutine_l_end:
    return akat_coroutine_state;
#undef akat_coroutine_state
#undef byte_to_send
#undef crc
#undef send_byte
#undef u16_to_format_and_send
#undef u32_to_format_and_send
#undef u8_to_format_and_send
}
static u8 usart0_writer__format_and_send_u8__akat_coroutine_state = 0;
static u8 usart0_writer__format_and_send_u8() {
#define akat_coroutine_state usart0_writer__format_and_send_u8__akat_coroutine_state
#define byte_to_send usart0_writer__byte_to_send
#define crc usart0_writer__crc
#define format_and_send_u8 usart0_writer__format_and_send_u8
#define send_byte usart0_writer__send_byte
#define u16_to_format_and_send usart0_writer__u16_to_format_and_send
#define u32_to_format_and_send usart0_writer__u32_to_format_and_send
#define u8_to_format_and_send usart0_writer__u8_to_format_and_send
    ;
    AKAT_HOT_CODE;

    switch (akat_coroutine_state) {
    case AKAT_COROUTINE_S_START:
        goto akat_coroutine_l_start;

    case AKAT_COROUTINE_S_END:
        goto akat_coroutine_l_end;

    case 2:
        goto akat_coroutine_l_2;

    case 3:
        goto akat_coroutine_l_3;
    }

akat_coroutine_l_start:
    AKAT_COLD_CODE;

    do {
        if (u8_to_format_and_send) {
            u8 h = u8_to_format_and_send / 16;

            if (h) {
                byte_to_send = HEX[h];

                do {
                    akat_coroutine_state = 2;
akat_coroutine_l_2:

                    if (send_byte() != AKAT_COROUTINE_S_START) {
                        return akat_coroutine_state;
                    }
                } while (0);

                ;
            }

            u8 i = u8_to_format_and_send & 15;
            byte_to_send = HEX[i];

            do {
                akat_coroutine_state = 3;
akat_coroutine_l_3:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return akat_coroutine_state;
                }
            } while (0);

            ;
        }
    } while (0);

    AKAT_COLD_CODE;
    akat_coroutine_state = AKAT_COROUTINE_S_START;
akat_coroutine_l_end:
    return akat_coroutine_state;
#undef akat_coroutine_state
#undef byte_to_send
#undef crc
#undef format_and_send_u8
#undef send_byte
#undef u16_to_format_and_send
#undef u32_to_format_and_send
#undef u8_to_format_and_send
}
static u8 usart0_writer__format_and_send_u16__akat_coroutine_state = 0;
static u8 usart0_writer__format_and_send_u16() {
#define akat_coroutine_state usart0_writer__format_and_send_u16__akat_coroutine_state
#define byte_to_send usart0_writer__byte_to_send
#define crc usart0_writer__crc
#define format_and_send_u16 usart0_writer__format_and_send_u16
#define format_and_send_u8 usart0_writer__format_and_send_u8
#define send_byte usart0_writer__send_byte
#define u16_to_format_and_send usart0_writer__u16_to_format_and_send
#define u32_to_format_and_send usart0_writer__u32_to_format_and_send
#define u8_to_format_and_send usart0_writer__u8_to_format_and_send
    ;
    AKAT_HOT_CODE;

    switch (akat_coroutine_state) {
    case AKAT_COROUTINE_S_START:
        goto akat_coroutine_l_start;

    case AKAT_COROUTINE_S_END:
        goto akat_coroutine_l_end;

    case 2:
        goto akat_coroutine_l_2;

    case 3:
        goto akat_coroutine_l_3;

    case 4:
        goto akat_coroutine_l_4;

    case 5:
        goto akat_coroutine_l_5;
    }

akat_coroutine_l_start:
    AKAT_COLD_CODE;

    do {
        u8_to_format_and_send = (u8)(u16_to_format_and_send / 256);

        if (u8_to_format_and_send) {
            do {
                akat_coroutine_state = 2;
akat_coroutine_l_2:

                if (format_and_send_u8() != AKAT_COROUTINE_S_START) {
                    return akat_coroutine_state;
                }
            } while (0);

            ;
            u8_to_format_and_send = (u8)u16_to_format_and_send;
            byte_to_send = HEX[u8_to_format_and_send / 16];

            do {
                akat_coroutine_state = 3;
akat_coroutine_l_3:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return akat_coroutine_state;
                }
            } while (0);

            ;
            byte_to_send = HEX[u8_to_format_and_send & 15];

            do {
                akat_coroutine_state = 4;
akat_coroutine_l_4:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return akat_coroutine_state;
                }
            } while (0);

            ;
        } else {
            u8_to_format_and_send = (u8)u16_to_format_and_send;

            do {
                akat_coroutine_state = 5;
akat_coroutine_l_5:

                if (format_and_send_u8() != AKAT_COROUTINE_S_START) {
                    return akat_coroutine_state;
                }
            } while (0);

            ;
        }
    } while (0);

    AKAT_COLD_CODE;
    akat_coroutine_state = AKAT_COROUTINE_S_START;
akat_coroutine_l_end:
    return akat_coroutine_state;
#undef akat_coroutine_state
#undef byte_to_send
#undef crc
#undef format_and_send_u16
#undef format_and_send_u8
#undef send_byte
#undef u16_to_format_and_send
#undef u32_to_format_and_send
#undef u8_to_format_and_send
}
static u8 usart0_writer__format_and_send_u32__akat_coroutine_state = 0;
static u8 usart0_writer__format_and_send_u32() {
#define akat_coroutine_state usart0_writer__format_and_send_u32__akat_coroutine_state
#define byte_to_send usart0_writer__byte_to_send
#define crc usart0_writer__crc
#define format_and_send_u16 usart0_writer__format_and_send_u16
#define format_and_send_u32 usart0_writer__format_and_send_u32
#define format_and_send_u8 usart0_writer__format_and_send_u8
#define send_byte usart0_writer__send_byte
#define u16_to_format_and_send usart0_writer__u16_to_format_and_send
#define u32_to_format_and_send usart0_writer__u32_to_format_and_send
#define u8_to_format_and_send usart0_writer__u8_to_format_and_send
    ;
    AKAT_HOT_CODE;

    switch (akat_coroutine_state) {
    case AKAT_COROUTINE_S_START:
        goto akat_coroutine_l_start;

    case AKAT_COROUTINE_S_END:
        goto akat_coroutine_l_end;

    case 2:
        goto akat_coroutine_l_2;

    case 3:
        goto akat_coroutine_l_3;

    case 4:
        goto akat_coroutine_l_4;

    case 5:
        goto akat_coroutine_l_5;

    case 6:
        goto akat_coroutine_l_6;

    case 7:
        goto akat_coroutine_l_7;
    }

akat_coroutine_l_start:
    AKAT_COLD_CODE;

    do {
        u16_to_format_and_send = (u16)(u32_to_format_and_send >> 16);

        if (u16_to_format_and_send) {
            do {
                akat_coroutine_state = 2;
akat_coroutine_l_2:

                if (format_and_send_u16() != AKAT_COROUTINE_S_START) {
                    return akat_coroutine_state;
                }
            } while (0);

            ;
            u16_to_format_and_send = (u16)u32_to_format_and_send;
            u8_to_format_and_send = (u8)(u16_to_format_and_send / 256);
            byte_to_send = HEX[u8_to_format_and_send / 16];

            do {
                akat_coroutine_state = 3;
akat_coroutine_l_3:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return akat_coroutine_state;
                }
            } while (0);

            ;
            byte_to_send = HEX[u8_to_format_and_send & 15];

            do {
                akat_coroutine_state = 4;
akat_coroutine_l_4:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return akat_coroutine_state;
                }
            } while (0);

            ;
            u8_to_format_and_send = (u8)u16_to_format_and_send;
            byte_to_send = HEX[u8_to_format_and_send / 16];

            do {
                akat_coroutine_state = 5;
akat_coroutine_l_5:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return akat_coroutine_state;
                }
            } while (0);

            ;
            byte_to_send = HEX[u8_to_format_and_send & 15];

            do {
                akat_coroutine_state = 6;
akat_coroutine_l_6:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return akat_coroutine_state;
                }
            } while (0);

            ;
        } else {
            u16_to_format_and_send = (u16)u32_to_format_and_send;

            do {
                akat_coroutine_state = 7;
akat_coroutine_l_7:

                if (format_and_send_u16() != AKAT_COROUTINE_S_START) {
                    return akat_coroutine_state;
                }
            } while (0);

            ;
        }
    } while (0);

    AKAT_COLD_CODE;
    akat_coroutine_state = AKAT_COROUTINE_S_START;
akat_coroutine_l_end:
    return akat_coroutine_state;
#undef akat_coroutine_state
#undef byte_to_send
#undef crc
#undef format_and_send_u16
#undef format_and_send_u32
#undef format_and_send_u8
#undef send_byte
#undef u16_to_format_and_send
#undef u32_to_format_and_send
#undef u8_to_format_and_send
}
static AKAT_FORCE_INLINE void usart0_writer() {
#define akat_coroutine_state usart0_writer__akat_coroutine_state
#define byte_to_send usart0_writer__byte_to_send
#define crc usart0_writer__crc
#define format_and_send_u16 usart0_writer__format_and_send_u16
#define format_and_send_u32 usart0_writer__format_and_send_u32
#define format_and_send_u8 usart0_writer__format_and_send_u8
#define send_byte usart0_writer__send_byte
#define u16_to_format_and_send usart0_writer__u16_to_format_and_send
#define u32_to_format_and_send usart0_writer__u32_to_format_and_send
#define u8_to_format_and_send usart0_writer__u8_to_format_and_send
    ;
    AKAT_HOT_CODE;

    switch (akat_coroutine_state) {
    case AKAT_COROUTINE_S_START:
        goto akat_coroutine_l_start;

    case AKAT_COROUTINE_S_END:
        goto akat_coroutine_l_end;

    case 2:
        goto akat_coroutine_l_2;

    case 3:
        goto akat_coroutine_l_3;

    case 4:
        goto akat_coroutine_l_4;

    case 5:
        goto akat_coroutine_l_5;

    case 6:
        goto akat_coroutine_l_6;

    case 7:
        goto akat_coroutine_l_7;

    case 8:
        goto akat_coroutine_l_8;

    case 9:
        goto akat_coroutine_l_9;

    case 10:
        goto akat_coroutine_l_10;

    case 11:
        goto akat_coroutine_l_11;

    case 12:
        goto akat_coroutine_l_12;

    case 13:
        goto akat_coroutine_l_13;

    case 14:
        goto akat_coroutine_l_14;

    case 15:
        goto akat_coroutine_l_15;

    case 16:
        goto akat_coroutine_l_16;

    case 17:
        goto akat_coroutine_l_17;

    case 18:
        goto akat_coroutine_l_18;

    case 19:
        goto akat_coroutine_l_19;

    case 20:
        goto akat_coroutine_l_20;

    case 21:
        goto akat_coroutine_l_21;

    case 22:
        goto akat_coroutine_l_22;

    case 23:
        goto akat_coroutine_l_23;

    case 24:
        goto akat_coroutine_l_24;

    case 25:
        goto akat_coroutine_l_25;

    case 26:
        goto akat_coroutine_l_26;

    case 27:
        goto akat_coroutine_l_27;

    case 28:
        goto akat_coroutine_l_28;

    case 29:
        goto akat_coroutine_l_29;

    case 30:
        goto akat_coroutine_l_30;

    case 31:
        goto akat_coroutine_l_31;

    case 32:
        goto akat_coroutine_l_32;

    case 33:
        goto akat_coroutine_l_33;

    case 34:
        goto akat_coroutine_l_34;

    case 35:
        goto akat_coroutine_l_35;

    case 36:
        goto akat_coroutine_l_36;
    }

akat_coroutine_l_start:
    AKAT_COLD_CODE;

    do {
        //---- All variable in the thread must be static (green threads requirement)
        ;
        ;
        ;
        ;
        ;

        //---- Subroutines can yield unlike functions

//---- Macro that writes the given status into UART
        //We also write some humand readable description of the protocol
        //also stuff to distinguish protocol versions and generate typescript parser code

        /* Defined new macro with name WRITE_STATUS  *///- - - - - - - - - - -

        //Main loop in thread (thread will yield on calls to YIELD$ or WAIT_UNTIL$)
        while (1) { //----  - - - - -- - - - - -
            crc = 0;
            //WRITE_STATUS(name for documentation, 1-character id for protocol, type1 val1, type2 val2, ...)
            byte_to_send = ' ';

            do {
                akat_coroutine_state = 2;
akat_coroutine_l_2:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            byte_to_send = 'A';

            do {
                akat_coroutine_state = 3;
akat_coroutine_l_3:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            /*
              COMMPROTO: A1: Misc: u32 uptime_deciseconds
              TS_PROTO_TYPE: "u32 uptime_deciseconds": number,
              TS_PROTO_ASSIGN: "u32 uptime_deciseconds": vals["A1"],
            */
            u32_to_format_and_send = uptime_deciseconds;

            do {
                akat_coroutine_state = 4;
akat_coroutine_l_4:

                if (format_and_send_u32() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            byte_to_send = ',';

            do {
                akat_coroutine_state = 5;
akat_coroutine_l_5:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            /*
              COMMPROTO: A2: Misc: u8 usart0_rx_overflow_count
              TS_PROTO_TYPE: "u8 usart0_rx_overflow_count": number,
              TS_PROTO_ASSIGN: "u8 usart0_rx_overflow_count": vals["A2"],
            */
            u8_to_format_and_send = usart0_rx_overflow_count;

            do {
                akat_coroutine_state = 6;
akat_coroutine_l_6:

                if (format_and_send_u8() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            byte_to_send = ',';

            do {
                akat_coroutine_state = 7;
akat_coroutine_l_7:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            /*
              COMMPROTO: A3: Misc: u32 main_loop_iterations_in_last_decisecond
              TS_PROTO_TYPE: "u32 main_loop_iterations_in_last_decisecond": number,
              TS_PROTO_ASSIGN: "u32 main_loop_iterations_in_last_decisecond": vals["A3"],
            */
            u32_to_format_and_send = main_loop_iterations_in_last_decisecond;

            do {
                akat_coroutine_state = 8;
akat_coroutine_l_8:

                if (format_and_send_u32() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            ;
            byte_to_send = ' ';

            do {
                akat_coroutine_state = 9;
akat_coroutine_l_9:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            byte_to_send = 'B';

            do {
                akat_coroutine_state = 10;
akat_coroutine_l_10:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            /*
              COMMPROTO: B1: Onboard temperature: u8 ds18b20_onboard.get_crc_errors()
              TS_PROTO_TYPE: "u8 ds18b20_onboard.get_crc_errors()": number,
              TS_PROTO_ASSIGN: "u8 ds18b20_onboard.get_crc_errors()": vals["B1"],
            */
            u8_to_format_and_send = ds18b20_onboard.get_crc_errors();

            do {
                akat_coroutine_state = 11;
akat_coroutine_l_11:

                if (format_and_send_u8() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            byte_to_send = ',';

            do {
                akat_coroutine_state = 12;
akat_coroutine_l_12:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            /*
              COMMPROTO: B2: Onboard temperature: u8 ds18b20_onboard.get_disconnects()
              TS_PROTO_TYPE: "u8 ds18b20_onboard.get_disconnects()": number,
              TS_PROTO_ASSIGN: "u8 ds18b20_onboard.get_disconnects()": vals["B2"],
            */
            u8_to_format_and_send = ds18b20_onboard.get_disconnects();

            do {
                akat_coroutine_state = 13;
akat_coroutine_l_13:

                if (format_and_send_u8() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            byte_to_send = ',';

            do {
                akat_coroutine_state = 14;
akat_coroutine_l_14:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            /*
              COMMPROTO: B3: Onboard temperature: u16 ds18b20_onboard.get_temperatureX16()
              TS_PROTO_TYPE: "u16 ds18b20_onboard.get_temperatureX16()": number,
              TS_PROTO_ASSIGN: "u16 ds18b20_onboard.get_temperatureX16()": vals["B3"],
            */
            u16_to_format_and_send = ds18b20_onboard.get_temperatureX16();

            do {
                akat_coroutine_state = 15;
akat_coroutine_l_15:

                if (format_and_send_u16() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            byte_to_send = ',';

            do {
                akat_coroutine_state = 16;
akat_coroutine_l_16:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            /*
              COMMPROTO: B4: Onboard temperature: u8 ds18b20_onboard.get_update_id()
              TS_PROTO_TYPE: "u8 ds18b20_onboard.get_update_id()": number,
              TS_PROTO_ASSIGN: "u8 ds18b20_onboard.get_update_id()": vals["B4"],
            */
            u8_to_format_and_send = ds18b20_onboard.get_update_id();

            do {
                akat_coroutine_state = 17;
akat_coroutine_l_17:

                if (format_and_send_u8() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            byte_to_send = ',';

            do {
                akat_coroutine_state = 18;
akat_coroutine_l_18:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            /*
              COMMPROTO: B5: Onboard temperature: u8 ds18b20_onboard.get_updated_deciseconds_ago()
              TS_PROTO_TYPE: "u8 ds18b20_onboard.get_updated_deciseconds_ago()": number,
              TS_PROTO_ASSIGN: "u8 ds18b20_onboard.get_updated_deciseconds_ago()": vals["B5"],
            */
            u8_to_format_and_send = ds18b20_onboard.get_updated_deciseconds_ago();

            do {
                akat_coroutine_state = 19;
akat_coroutine_l_19:

                if (format_and_send_u8() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            ;
            byte_to_send = ' ';

            do {
                akat_coroutine_state = 20;
akat_coroutine_l_20:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            byte_to_send = 'C';

            do {
                akat_coroutine_state = 21;
akat_coroutine_l_21:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            /*
              COMMPROTO: C1: Controller temperature: u8 ds18b20_ext.get_crc_errors()
              TS_PROTO_TYPE: "u8 ds18b20_ext.get_crc_errors()": number,
              TS_PROTO_ASSIGN: "u8 ds18b20_ext.get_crc_errors()": vals["C1"],
            */
            u8_to_format_and_send = ds18b20_ext.get_crc_errors();

            do {
                akat_coroutine_state = 22;
akat_coroutine_l_22:

                if (format_and_send_u8() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            byte_to_send = ',';

            do {
                akat_coroutine_state = 23;
akat_coroutine_l_23:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            /*
              COMMPROTO: C2: Controller temperature: u8 ds18b20_ext.get_disconnects()
              TS_PROTO_TYPE: "u8 ds18b20_ext.get_disconnects()": number,
              TS_PROTO_ASSIGN: "u8 ds18b20_ext.get_disconnects()": vals["C2"],
            */
            u8_to_format_and_send = ds18b20_ext.get_disconnects();

            do {
                akat_coroutine_state = 24;
akat_coroutine_l_24:

                if (format_and_send_u8() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            byte_to_send = ',';

            do {
                akat_coroutine_state = 25;
akat_coroutine_l_25:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            /*
              COMMPROTO: C3: Controller temperature: u16 ds18b20_ext.get_temperatureX16()
              TS_PROTO_TYPE: "u16 ds18b20_ext.get_temperatureX16()": number,
              TS_PROTO_ASSIGN: "u16 ds18b20_ext.get_temperatureX16()": vals["C3"],
            */
            u16_to_format_and_send = ds18b20_ext.get_temperatureX16();

            do {
                akat_coroutine_state = 26;
akat_coroutine_l_26:

                if (format_and_send_u16() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            byte_to_send = ',';

            do {
                akat_coroutine_state = 27;
akat_coroutine_l_27:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            /*
              COMMPROTO: C4: Controller temperature: u8 ds18b20_ext.get_update_id()
              TS_PROTO_TYPE: "u8 ds18b20_ext.get_update_id()": number,
              TS_PROTO_ASSIGN: "u8 ds18b20_ext.get_update_id()": vals["C4"],
            */
            u8_to_format_and_send = ds18b20_ext.get_update_id();

            do {
                akat_coroutine_state = 28;
akat_coroutine_l_28:

                if (format_and_send_u8() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            byte_to_send = ',';

            do {
                akat_coroutine_state = 29;
akat_coroutine_l_29:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            /*
              COMMPROTO: C5: Controller temperature: u8 ds18b20_ext.get_updated_deciseconds_ago()
              TS_PROTO_TYPE: "u8 ds18b20_ext.get_updated_deciseconds_ago()": number,
              TS_PROTO_ASSIGN: "u8 ds18b20_ext.get_updated_deciseconds_ago()": vals["C5"],
            */
            u8_to_format_and_send = ds18b20_ext.get_updated_deciseconds_ago();

            do {
                akat_coroutine_state = 30;
akat_coroutine_l_30:

                if (format_and_send_u8() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            ;
            //Protocol version
            byte_to_send = ' ';

            do {
                akat_coroutine_state = 31;
akat_coroutine_l_31:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            u8_to_format_and_send = 0x4b;

            do {
                akat_coroutine_state = 32;
akat_coroutine_l_32:

                if (format_and_send_u8() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            //Done writing status, send: CRC\r\n
            byte_to_send = ' ';

            do {
                akat_coroutine_state = 33;
akat_coroutine_l_33:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            u8_to_format_and_send = crc;

            do {
                akat_coroutine_state = 34;
akat_coroutine_l_34:

                if (format_and_send_u8() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            //Newline
            byte_to_send = '\r';

            do {
                akat_coroutine_state = 35;
akat_coroutine_l_35:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
            byte_to_send = '\n';

            do {
                akat_coroutine_state = 36;
akat_coroutine_l_36:

                if (send_byte() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;
        }
    } while (0);

    AKAT_COLD_CODE;
    akat_coroutine_state = AKAT_COROUTINE_S_END;
akat_coroutine_l_end:
    return;
#undef akat_coroutine_state
#undef byte_to_send
#undef crc
#undef format_and_send_u16
#undef format_and_send_u32
#undef format_and_send_u8
#undef send_byte
#undef u16_to_format_and_send
#undef u32_to_format_and_send
#undef u8_to_format_and_send
}

;






// ---------------------------------------------------------------------------------
// USART0(USB): This thread processes input from usart0_rx_bytes_buf that gets populated in ISR

static u8 usart0_reader__akat_coroutine_state = 0;
static u8 usart0_reader__command_code = 0;
static u8 usart0_reader__command_arg = 0;
static u8 usart0_reader__read_command__akat_coroutine_state = 0;
static u8 usart0_reader__read_command__dequeued_byte = 0;
static u8 usart0_reader__read_command__command_arg_copy = 0;
register u8 usart0_reader__read_command__dequeue_byte__akat_coroutine_state asm ("r3");
static u8 usart0_reader__read_command__dequeue_byte() {
#define akat_coroutine_state usart0_reader__read_command__dequeue_byte__akat_coroutine_state
#define command_arg usart0_reader__command_arg
#define command_arg_copy usart0_reader__read_command__command_arg_copy
#define command_code usart0_reader__command_code
#define dequeue_byte usart0_reader__read_command__dequeue_byte
#define dequeued_byte usart0_reader__read_command__dequeued_byte
#define read_command usart0_reader__read_command
    ;
    AKAT_HOT_CODE;

    switch (akat_coroutine_state) {
    case AKAT_COROUTINE_S_START:
        goto akat_coroutine_l_start;

    case AKAT_COROUTINE_S_END:
        goto akat_coroutine_l_end;

    case 2:
        goto akat_coroutine_l_2;
    }

akat_coroutine_l_start:
    AKAT_COLD_CODE;

    do {
        //Wait until there is something to read
        do {
            akat_coroutine_state = 2;
akat_coroutine_l_2:

            if (!(usart0_rx_next_empty_idx != usart0_rx_next_read_idx)) {
                AKAT_HOT_CODE;
                return akat_coroutine_state;
            }
        } while (0);

        ;
        //Read byte first, then increment idx!
        dequeued_byte = usart0_rx_bytes_buf[usart0_rx_next_read_idx];
        usart0_rx_next_read_idx = (usart0_rx_next_read_idx + 1) & (AK_USART0_RX_BUF_SIZE - 1);
    } while (0);

    AKAT_COLD_CODE;
    akat_coroutine_state = AKAT_COROUTINE_S_START;
akat_coroutine_l_end:
    return akat_coroutine_state;
#undef akat_coroutine_state
#undef command_arg
#undef command_arg_copy
#undef command_code
#undef dequeue_byte
#undef dequeued_byte
#undef read_command
}
static u8 usart0_reader__read_command__read_arg_and_dequeue__akat_coroutine_state = 0;
static u8 usart0_reader__read_command__read_arg_and_dequeue() {
#define akat_coroutine_state usart0_reader__read_command__read_arg_and_dequeue__akat_coroutine_state
#define command_arg usart0_reader__command_arg
#define command_arg_copy usart0_reader__read_command__command_arg_copy
#define command_code usart0_reader__command_code
#define dequeue_byte usart0_reader__read_command__dequeue_byte
#define dequeued_byte usart0_reader__read_command__dequeued_byte
#define read_arg_and_dequeue usart0_reader__read_command__read_arg_and_dequeue
#define read_command usart0_reader__read_command
    ;
    AKAT_HOT_CODE;

    switch (akat_coroutine_state) {
    case AKAT_COROUTINE_S_START:
        goto akat_coroutine_l_start;

    case AKAT_COROUTINE_S_END:
        goto akat_coroutine_l_end;

    case 2:
        goto akat_coroutine_l_2;

    case 3:
        goto akat_coroutine_l_3;

    case 4:
        goto akat_coroutine_l_4;

    case 5:
        goto akat_coroutine_l_5;
    }

akat_coroutine_l_start:
    AKAT_COLD_CODE;

    do {
        command_arg = 0;

        do {
            akat_coroutine_state = 2;
akat_coroutine_l_2:

            if (dequeue_byte() != AKAT_COROUTINE_S_START) {
                return akat_coroutine_state;
            }
        } while (0);

        ;

        if (dequeued_byte >= '0' && dequeued_byte <= '9') {
            command_arg = dequeued_byte - '0';

            do {
                akat_coroutine_state = 3;
akat_coroutine_l_3:

                if (dequeue_byte() != AKAT_COROUTINE_S_START) {
                    return akat_coroutine_state;
                }
            } while (0);

            ;

            if (dequeued_byte >= '0' && dequeued_byte <= '9') {
                command_arg = command_arg * 10 + (dequeued_byte - '0');

                do {
                    akat_coroutine_state = 4;
akat_coroutine_l_4:

                    if (dequeue_byte() != AKAT_COROUTINE_S_START) {
                        return akat_coroutine_state;
                    }
                } while (0);

                ;

                if (dequeued_byte >= '0' && dequeued_byte <= '9') {
                    if (command_arg < 25 || (command_arg == 25 && dequeued_byte <= '5')) {
                        command_arg = command_arg * 10 + (dequeued_byte - '0');

                        do {
                            akat_coroutine_state = 5;
akat_coroutine_l_5:

                            if (dequeue_byte() != AKAT_COROUTINE_S_START) {
                                return akat_coroutine_state;
                            }
                        } while (0);

                        ;
                    }
                }
            }
        }
    } while (0);

    AKAT_COLD_CODE;
    akat_coroutine_state = AKAT_COROUTINE_S_START;
akat_coroutine_l_end:
    return akat_coroutine_state;
#undef akat_coroutine_state
#undef command_arg
#undef command_arg_copy
#undef command_code
#undef dequeue_byte
#undef dequeued_byte
#undef read_arg_and_dequeue
#undef read_command
}
static u8 usart0_reader__read_command() {
#define akat_coroutine_state usart0_reader__read_command__akat_coroutine_state
#define command_arg usart0_reader__command_arg
#define command_arg_copy usart0_reader__read_command__command_arg_copy
#define command_code usart0_reader__command_code
#define dequeue_byte usart0_reader__read_command__dequeue_byte
#define dequeued_byte usart0_reader__read_command__dequeued_byte
#define read_arg_and_dequeue usart0_reader__read_command__read_arg_and_dequeue
#define read_command usart0_reader__read_command
    ;
    AKAT_HOT_CODE;

    switch (akat_coroutine_state) {
    case AKAT_COROUTINE_S_START:
        goto akat_coroutine_l_start;

    case AKAT_COROUTINE_S_END:
        goto akat_coroutine_l_end;

    case 2:
        goto akat_coroutine_l_2;

    case 3:
        goto akat_coroutine_l_3;

    case 4:
        goto akat_coroutine_l_4;

    case 5:
        goto akat_coroutine_l_5;
    }

akat_coroutine_l_start:
    AKAT_COLD_CODE;

    do {
        ;
        ;
        //Gets byte from usart0_rx_bytes_buf buffer.
//Read arg into command_arg, leaves byte after arg in the dequeued_byte variable
        //so the caller must process it as well upon return!
command_reading_start:

        //Read opening bracket
        do {
            akat_coroutine_state = 2;
akat_coroutine_l_2:

            if (dequeue_byte() != AKAT_COROUTINE_S_START) {
                return akat_coroutine_state;
            }
        } while (0);

        ;

        if (dequeued_byte != '<') {
            goto command_reading_start;
        }//Read command code

        //Verify that code is really a code letter
        do {
            akat_coroutine_state = 3;
akat_coroutine_l_3:

            if (dequeue_byte() != AKAT_COROUTINE_S_START) {
                return akat_coroutine_state;
            }
        } while (0);

        ;

        if (dequeued_byte < 'A' || dequeued_byte > 'Z') {
            goto command_reading_start;
        }

        command_code = dequeued_byte;

        //Read arg and save it as copy, note that read_arg aborts
        //when it either read fourth character in a row or a non digit character
        //so we have to process it (dequeued_byte) when the call to read_arg returns.
        //Verify that stuff that comes after the arg is a command code again!
        do {
            akat_coroutine_state = 4;
akat_coroutine_l_4:

            if (read_arg_and_dequeue() != AKAT_COROUTINE_S_START) {
                return akat_coroutine_state;
            }
        } while (0);

        ;

        if (dequeued_byte != command_code) {
            goto command_reading_start;
        }

        command_arg_copy = command_arg;

        //Read command arg once again (it comes after a copy of command code which we already verified)
        //We also verify that there is an > character right after the arg
        //And of course we verify that arg matches the copy we read before.
        do {
            akat_coroutine_state = 5;
akat_coroutine_l_5:

            if (read_arg_and_dequeue() != AKAT_COROUTINE_S_START) {
                return akat_coroutine_state;
            }
        } while (0);

        ;

        if (dequeued_byte != '>' || command_arg_copy != command_arg) {
            goto command_reading_start;
        }
    } while (0);

    AKAT_COLD_CODE;
    akat_coroutine_state = AKAT_COROUTINE_S_START;
akat_coroutine_l_end:
    return akat_coroutine_state;
#undef akat_coroutine_state
#undef command_arg
#undef command_arg_copy
#undef command_code
#undef dequeue_byte
#undef dequeued_byte
#undef read_arg_and_dequeue
#undef read_command
}
static AKAT_FORCE_INLINE void usart0_reader() {
#define akat_coroutine_state usart0_reader__akat_coroutine_state
#define command_arg usart0_reader__command_arg
#define command_code usart0_reader__command_code
#define read_command usart0_reader__read_command
    ;
    AKAT_HOT_CODE;

    switch (akat_coroutine_state) {
    case AKAT_COROUTINE_S_START:
        goto akat_coroutine_l_start;

    case AKAT_COROUTINE_S_END:
        goto akat_coroutine_l_end;

    case 2:
        goto akat_coroutine_l_2;
    }

akat_coroutine_l_start:
    AKAT_COLD_CODE;

    do {
        //---- all variable in the thread must be static (green threads requirement)
        ;
        ;

        //Subroutine that reads a command from the input
        //Command is expected to be in the format as one printed out by 'send_status'
        //Command end ups in 'command_code' variable and optional
        //arguments end ups in 'command_arg'. If commands comes without argument, then
        //we assume it is 0 by convention.

//- - - - - - - - - - -
        //Main loop in thread (thread will yield on calls to YIELD$ or WAIT_UNTIL$)
        while (1) { //Read command and put results into 'command_code' and 'command_arg'.
            do {
                akat_coroutine_state = 2;
akat_coroutine_l_2:

                if (read_command() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;

            switch (command_code) {
                /*
                case 'B':
                    received_clock1 = command_arg;
                    break;*/
            }
        }
    } while (0);

    AKAT_COLD_CODE;
    akat_coroutine_state = AKAT_COROUTINE_S_END;
akat_coroutine_l_end:
    return;
#undef akat_coroutine_state
#undef command_arg
#undef command_code
#undef read_command
}

;








////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Main


static AKAT_FORCE_INLINE void akat_on_every_decisecond() {
    performance_ticker();
    uptime_ticker();
    ds18b20_ticker();
    ds18b20_onboard__ticker();
    ds18b20_ext__ticker();
    pwr_button__ticker();
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

// Performs temperature measurements for sensors registered with X_DS18S20$ macro.
// We measure temperature for several sensors at the same time without blocking other threads as much as possible

static u8 ds18b20__tconv_countdown = 0;

;
;


static void ds18b20_ticker() {
//We are waiting for temperature conversion and decrement the counter every 0.1 second
    if (ds18b20__tconv_countdown) {
        ds18b20__tconv_countdown -= AKAT_ONE;
    }
}

;




register u8 ds18b20_thread__akat_coroutine_state asm ("r5");
static u8 ds18b20_thread__byte_to_send = 0;
static u8 ds18b20_thread__command_to_send = 0;
static u8 ds18b20_thread__receive_idx = 0;
static AKAT_FORCE_INLINE u8 ds18b20_thread__has_connected_sensors() {
#define akat_coroutine_state ds18b20_thread__akat_coroutine_state
#define byte_to_send ds18b20_thread__byte_to_send
#define command_to_send ds18b20_thread__command_to_send
#define has_connected_sensors ds18b20_thread__has_connected_sensors
#define receive_idx ds18b20_thread__receive_idx

    if (ds18b20_onboard__connected) {
        return 1;
    }

    if (ds18b20_ext__connected) {
        return 1;
    }

    return 0;
#undef akat_coroutine_state
#undef byte_to_send
#undef command_to_send
#undef has_connected_sensors
#undef receive_idx
}
static void ds18b20_thread__write_bit(const u8 bit) {
#define akat_coroutine_state ds18b20_thread__akat_coroutine_state
#define byte_to_send ds18b20_thread__byte_to_send
#define command_to_send ds18b20_thread__command_to_send
#define has_connected_sensors ds18b20_thread__has_connected_sensors
#define receive_idx ds18b20_thread__receive_idx
#define write_bit ds18b20_thread__write_bit

//'bit'can be either zero or non zero. Non zero bit value is treated as 1.

    if (ds18b20_onboard__connected) {
        ds18b20_onboard__pin.set_output_mode();
        ds18b20_onboard__pin.set(0);
    }

    if (ds18b20_ext__connected) {
        ds18b20_ext__pin.set_output_mode();
        ds18b20_ext__pin.set(0);
    }

    if (bit) {//Wait for edge to raise and slave to detect it
        akat_delay_us(4);

        //Set pin to 1 such that slave can sample the value we transmit
        if (ds18b20_onboard__connected) {
            ds18b20_onboard__pin.set_input_mode();
        }

        if (ds18b20_ext__connected) {
            ds18b20_ext__pin.set_input_mode();
        }
    }

    akat_delay_us(60);

    //Recovery, the pin will be pulled up by the external/internal pull-up resistor
    if (ds18b20_onboard__connected) {
        ds18b20_onboard__pin.set_input_mode();
    }

    if (ds18b20_ext__connected) {
        ds18b20_ext__pin.set_input_mode();
    }

    akat_delay_us(10);
#undef akat_coroutine_state
#undef byte_to_send
#undef command_to_send
#undef has_connected_sensors
#undef receive_idx
#undef write_bit
}
static void ds18b20_thread__read_bit(u8 mask) {
#define akat_coroutine_state ds18b20_thread__akat_coroutine_state
#define byte_to_send ds18b20_thread__byte_to_send
#define command_to_send ds18b20_thread__command_to_send
#define has_connected_sensors ds18b20_thread__has_connected_sensors
#define read_bit ds18b20_thread__read_bit
#define receive_idx ds18b20_thread__receive_idx
#define write_bit ds18b20_thread__write_bit

//Returns either 0 or non zero (doesn't mean '1'!)

    //Indicate that we want to read a bit
    if (ds18b20_onboard__connected) {
        ds18b20_onboard__pin.set_output_mode();
        ds18b20_onboard__pin.set(0);
    }

    if (ds18b20_ext__connected) {
        ds18b20_ext__pin.set_output_mode();
        ds18b20_ext__pin.set(0);
    }//Allow slave to detect the falling edge on the pin

    akat_delay_us(4);

    //Release the line and let slave set it to the value we will read after the delay
    if (ds18b20_onboard__connected) {
        ds18b20_onboard__pin.set_input_mode();
    }

    if (ds18b20_ext__connected) {
        ds18b20_ext__pin.set_input_mode();
    }

    akat_delay_us(9);

    if (ds18b20_onboard__connected) {
        if (ds18b20_onboard__pin.is_set()) {
            ds18b20_onboard__received_byte += mask;
        }
    }

    if (ds18b20_ext__connected) {
        if (ds18b20_ext__pin.is_set()) {
            ds18b20_ext__received_byte += mask;
        }
    }//Total duration of reading slot must be at least 60

    akat_delay_us(55);
#undef akat_coroutine_state
#undef byte_to_send
#undef command_to_send
#undef has_connected_sensors
#undef read_bit
#undef receive_idx
#undef write_bit
}
static u8 ds18b20_thread__send_byte__akat_coroutine_state = 0;
static u8 ds18b20_thread__send_byte() {
#define akat_coroutine_state ds18b20_thread__send_byte__akat_coroutine_state
#define byte_to_send ds18b20_thread__byte_to_send
#define command_to_send ds18b20_thread__command_to_send
#define has_connected_sensors ds18b20_thread__has_connected_sensors
#define read_bit ds18b20_thread__read_bit
#define receive_idx ds18b20_thread__receive_idx
#define send_byte ds18b20_thread__send_byte
#define write_bit ds18b20_thread__write_bit
    ;
    AKAT_HOT_CODE;

    switch (akat_coroutine_state) {
    case AKAT_COROUTINE_S_START:
        goto akat_coroutine_l_start;

    case AKAT_COROUTINE_S_END:
        goto akat_coroutine_l_end;

    case 2:
        goto akat_coroutine_l_2;

    case 3:
        goto akat_coroutine_l_3;

    case 4:
        goto akat_coroutine_l_4;

    case 5:
        goto akat_coroutine_l_5;

    case 6:
        goto akat_coroutine_l_6;

    case 7:
        goto akat_coroutine_l_7;

    case 8:
        goto akat_coroutine_l_8;
    }

akat_coroutine_l_start:
    AKAT_COLD_CODE;

    do {
        //LSB (Least significant bit) first order
        write_bit(byte_to_send & H(0));

        do {
            akat_coroutine_state = 2;
            return akat_coroutine_state;
akat_coroutine_l_2:
            ;
        } while (0);

        ;
        write_bit(byte_to_send & H(1));

        do {
            akat_coroutine_state = 3;
            return akat_coroutine_state;
akat_coroutine_l_3:
            ;
        } while (0);

        ;
        write_bit(byte_to_send & H(2));

        do {
            akat_coroutine_state = 4;
            return akat_coroutine_state;
akat_coroutine_l_4:
            ;
        } while (0);

        ;
        write_bit(byte_to_send & H(3));

        do {
            akat_coroutine_state = 5;
            return akat_coroutine_state;
akat_coroutine_l_5:
            ;
        } while (0);

        ;
        write_bit(byte_to_send & H(4));

        do {
            akat_coroutine_state = 6;
            return akat_coroutine_state;
akat_coroutine_l_6:
            ;
        } while (0);

        ;
        write_bit(byte_to_send & H(5));

        do {
            akat_coroutine_state = 7;
            return akat_coroutine_state;
akat_coroutine_l_7:
            ;
        } while (0);

        ;
        write_bit(byte_to_send & H(6));

        do {
            akat_coroutine_state = 8;
            return akat_coroutine_state;
akat_coroutine_l_8:
            ;
        } while (0);

        ;
        write_bit(byte_to_send & H(7));
    } while (0);

    AKAT_COLD_CODE;
    akat_coroutine_state = AKAT_COROUTINE_S_START;
akat_coroutine_l_end:
    return akat_coroutine_state;
#undef akat_coroutine_state
#undef byte_to_send
#undef command_to_send
#undef has_connected_sensors
#undef read_bit
#undef receive_idx
#undef send_byte
#undef write_bit
}
static u8 ds18b20_thread__receive_byte__akat_coroutine_state = 0;
static u8 ds18b20_thread__receive_byte() {
#define akat_coroutine_state ds18b20_thread__receive_byte__akat_coroutine_state
#define byte_to_send ds18b20_thread__byte_to_send
#define command_to_send ds18b20_thread__command_to_send
#define has_connected_sensors ds18b20_thread__has_connected_sensors
#define read_bit ds18b20_thread__read_bit
#define receive_byte ds18b20_thread__receive_byte
#define receive_idx ds18b20_thread__receive_idx
#define send_byte ds18b20_thread__send_byte
#define write_bit ds18b20_thread__write_bit
    ;
    AKAT_HOT_CODE;

    switch (akat_coroutine_state) {
    case AKAT_COROUTINE_S_START:
        goto akat_coroutine_l_start;

    case AKAT_COROUTINE_S_END:
        goto akat_coroutine_l_end;

    case 2:
        goto akat_coroutine_l_2;

    case 3:
        goto akat_coroutine_l_3;

    case 4:
        goto akat_coroutine_l_4;

    case 5:
        goto akat_coroutine_l_5;

    case 6:
        goto akat_coroutine_l_6;

    case 7:
        goto akat_coroutine_l_7;

    case 8:
        goto akat_coroutine_l_8;
    }

akat_coroutine_l_start:
    AKAT_COLD_CODE;

    do {
        //LSB (Least significant bit) first order
        ds18b20_onboard__received_byte = 0;
        ds18b20_ext__received_byte = 0;
        read_bit(H(0));

        do {
            akat_coroutine_state = 2;
            return akat_coroutine_state;
akat_coroutine_l_2:
            ;
        } while (0);

        ;
        read_bit(H(1));

        do {
            akat_coroutine_state = 3;
            return akat_coroutine_state;
akat_coroutine_l_3:
            ;
        } while (0);

        ;
        read_bit(H(2));

        do {
            akat_coroutine_state = 4;
            return akat_coroutine_state;
akat_coroutine_l_4:
            ;
        } while (0);

        ;
        read_bit(H(3));

        do {
            akat_coroutine_state = 5;
            return akat_coroutine_state;
akat_coroutine_l_5:
            ;
        } while (0);

        ;
        read_bit(H(4));

        do {
            akat_coroutine_state = 6;
            return akat_coroutine_state;
akat_coroutine_l_6:
            ;
        } while (0);

        ;
        read_bit(H(5));

        do {
            akat_coroutine_state = 7;
            return akat_coroutine_state;
akat_coroutine_l_7:
            ;
        } while (0);

        ;
        read_bit(H(6));

        do {
            akat_coroutine_state = 8;
            return akat_coroutine_state;
akat_coroutine_l_8:
            ;
        } while (0);

        ;
        read_bit(H(7));
        ds18b20_onboard__scratchpad[receive_idx] = ds18b20_onboard__received_byte;
        ds18b20_ext__scratchpad[receive_idx] = ds18b20_ext__received_byte;
    } while (0);

    AKAT_COLD_CODE;
    akat_coroutine_state = AKAT_COROUTINE_S_START;
akat_coroutine_l_end:
    return akat_coroutine_state;
#undef akat_coroutine_state
#undef byte_to_send
#undef command_to_send
#undef has_connected_sensors
#undef read_bit
#undef receive_byte
#undef receive_idx
#undef send_byte
#undef write_bit
}
static u8 ds18b20_thread__send_command__akat_coroutine_state = 0;
static u8 ds18b20_thread__send_command() {
#define akat_coroutine_state ds18b20_thread__send_command__akat_coroutine_state
#define byte_to_send ds18b20_thread__byte_to_send
#define command_to_send ds18b20_thread__command_to_send
#define has_connected_sensors ds18b20_thread__has_connected_sensors
#define read_bit ds18b20_thread__read_bit
#define receive_byte ds18b20_thread__receive_byte
#define receive_idx ds18b20_thread__receive_idx
#define send_byte ds18b20_thread__send_byte
#define send_command ds18b20_thread__send_command
#define write_bit ds18b20_thread__write_bit
    ;
    AKAT_HOT_CODE;

    switch (akat_coroutine_state) {
    case AKAT_COROUTINE_S_START:
        goto akat_coroutine_l_start;

    case AKAT_COROUTINE_S_END:
        goto akat_coroutine_l_end;

    case 2:
        goto akat_coroutine_l_2;

    case 3:
        goto akat_coroutine_l_3;

    case 4:
        goto akat_coroutine_l_4;

    case 5:
        goto akat_coroutine_l_5;

    case 6:
        goto akat_coroutine_l_6;
    }

akat_coroutine_l_start:
    AKAT_COLD_CODE;

    do {
        //Starts communication:
        //* send reset pulse
        //* wait for presence response
        if (has_connected_sensors()) {//Reset pulse: 480 ... 960 us in low state
            if (ds18b20_onboard__connected) {
                ds18b20_onboard__pin.set_output_mode();
                ds18b20_onboard__pin.set(0);
            }

            if (ds18b20_ext__connected) {
                ds18b20_ext__pin.set_output_mode();
                ds18b20_ext__pin.set(0);
            }

            akat_delay_us(600);

            //Slave awaits 15 ... 60 us
            //and then sinks pin to ground for 60 ... 240 us
            if (ds18b20_onboard__connected) {
                ds18b20_onboard__pin.set_input_mode();
            }

            if (ds18b20_ext__connected) {
                ds18b20_ext__pin.set_input_mode();
            }

            akat_delay_us(80);

            if (ds18b20_onboard__connected) {
                ds18b20_onboard__connected = !ds18b20_onboard__pin.is_set();

                if (!ds18b20_onboard__connected) {
                    ds18b20_onboard__disconnects += AKAT_ONE;

                    if (!ds18b20_onboard__disconnects) {//We can't go beyond 255
                        ds18b20_onboard__disconnects -= AKAT_ONE;
                    }
                }
            }

            if (ds18b20_ext__connected) {
                ds18b20_ext__connected = !ds18b20_ext__pin.is_set();

                if (!ds18b20_ext__connected) {
                    ds18b20_ext__disconnects += AKAT_ONE;

                    if (!ds18b20_ext__disconnects) {//We can't go beyond 255
                        ds18b20_ext__disconnects -= AKAT_ONE;
                    }
                }
            }

            do {
                akat_coroutine_state = 2;
                return akat_coroutine_state;
akat_coroutine_l_2:
                ;
            } while (0);

            ;
            //We must wait for present pulse for minimum of 480 us
            //We have already waited for 80 us in start_initialize + some time in 'yield'
            akat_delay_us(410);

            if (has_connected_sensors()) {
                do {
                    akat_coroutine_state = 3;
                    return akat_coroutine_state;
akat_coroutine_l_3:
                    ;
                } while (0);

                ;
                //Skip ROM
                byte_to_send = 0xCC;

                do {
                    akat_coroutine_state = 4;
akat_coroutine_l_4:

                    if (send_byte() != AKAT_COROUTINE_S_START) {
                        return akat_coroutine_state;
                    }
                } while (0);

                ;

                do {
                    akat_coroutine_state = 5;
                    return akat_coroutine_state;
akat_coroutine_l_5:
                    ;
                } while (0);

                ;
                //Send the command
                byte_to_send = command_to_send;

                do {
                    akat_coroutine_state = 6;
akat_coroutine_l_6:

                    if (send_byte() != AKAT_COROUTINE_S_START) {
                        return akat_coroutine_state;
                    }
                } while (0);

                ;
            }
        }
    } while (0);

    AKAT_COLD_CODE;
    akat_coroutine_state = AKAT_COROUTINE_S_START;
akat_coroutine_l_end:
    return akat_coroutine_state;
#undef akat_coroutine_state
#undef byte_to_send
#undef command_to_send
#undef has_connected_sensors
#undef read_bit
#undef receive_byte
#undef receive_idx
#undef send_byte
#undef send_command
#undef write_bit
}
static AKAT_FORCE_INLINE void ds18b20_thread() {
#define akat_coroutine_state ds18b20_thread__akat_coroutine_state
#define byte_to_send ds18b20_thread__byte_to_send
#define command_to_send ds18b20_thread__command_to_send
#define has_connected_sensors ds18b20_thread__has_connected_sensors
#define read_bit ds18b20_thread__read_bit
#define receive_byte ds18b20_thread__receive_byte
#define receive_idx ds18b20_thread__receive_idx
#define send_byte ds18b20_thread__send_byte
#define send_command ds18b20_thread__send_command
#define write_bit ds18b20_thread__write_bit
    ;
    AKAT_HOT_CODE;

    switch (akat_coroutine_state) {
    case AKAT_COROUTINE_S_START:
        goto akat_coroutine_l_start;

    case AKAT_COROUTINE_S_END:
        goto akat_coroutine_l_end;

    case 2:
        goto akat_coroutine_l_2;

    case 3:
        goto akat_coroutine_l_3;

    case 4:
        goto akat_coroutine_l_4;

    case 5:
        goto akat_coroutine_l_5;

    case 6:
        goto akat_coroutine_l_6;

    case 7:
        goto akat_coroutine_l_7;

    case 8:
        goto akat_coroutine_l_8;

    case 9:
        goto akat_coroutine_l_9;

    case 10:
        goto akat_coroutine_l_10;

    case 11:
        goto akat_coroutine_l_11;

    case 12:
        goto akat_coroutine_l_12;

    case 13:
        goto akat_coroutine_l_13;

    case 14:
        goto akat_coroutine_l_14;

    case 15:
        goto akat_coroutine_l_15;

    case 16:
        goto akat_coroutine_l_16;
    }

akat_coroutine_l_start:
    AKAT_COLD_CODE;

    do {
        //---- All variable in the thread must be static (green threads requirement)
        ;
        ;
        ;

        //---- Functions

//---- Subroutines can yield unlike functions

        //Sends byte from 'byte_to_send', initialization is supposed to be done

//Receive byte into 'received_byte'

//Sends command from 'command_to_send'
        //Does nothing if ds18b20_ext__connected is FALSE.
        //Sets ds18b20_ext__connected to FALSE if presence pulse is missing!

//- - - - - - - - - - -
        //Main loop in thread (thread will yield on calls to YIELD$ or WAIT_UNTIL$)
        while (1) { //Everything is connected until proven otherwise by presence pulse
            ds18b20_onboard__connected = 1;
            ds18b20_ext__connected = 1;
            //Start temperature conversion
            command_to_send = 0x44;

            do {
                akat_coroutine_state = 2;
akat_coroutine_l_2:

                if (send_command() != AKAT_COROUTINE_S_START) {
                    return ;
                }
            } while (0);

            ;

            if (has_connected_sensors()) {//Wait for conversion to end. It takes 750ms to convert, but we "wait"for approx. 900ms ... 1 second
                //tconv_countdown will be decremented every 1/10 second.
                ds18b20__tconv_countdown = 10;

                do {
                    akat_coroutine_state = 3;
akat_coroutine_l_3:

                    if (!(ds18b20__tconv_countdown == 0)) {
                        AKAT_HOT_CODE;
                        return ;
                    }
                } while (0);

                ; //This will YIELD
                //Read scratchpad (temperature)
                command_to_send = 0xBE;

                do {
                    akat_coroutine_state = 4;
akat_coroutine_l_4:

                    if (send_command() != AKAT_COROUTINE_S_START) {
                        return ;
                    }
                } while (0);

                ;

                if (has_connected_sensors()) {
                    receive_idx = 0;

                    do {
                        akat_coroutine_state = 5;
akat_coroutine_l_5:

                        if (receive_byte() != AKAT_COROUTINE_S_START) {
                            return ;
                        }
                    } while (0);

                    ;
                    receive_idx = 1;

                    do {
                        akat_coroutine_state = 6;
akat_coroutine_l_6:

                        if (receive_byte() != AKAT_COROUTINE_S_START) {
                            return ;
                        }
                    } while (0);

                    ;
                    receive_idx = 2;

                    do {
                        akat_coroutine_state = 7;
akat_coroutine_l_7:

                        if (receive_byte() != AKAT_COROUTINE_S_START) {
                            return ;
                        }
                    } while (0);

                    ;
                    receive_idx = 3;

                    do {
                        akat_coroutine_state = 8;
akat_coroutine_l_8:

                        if (receive_byte() != AKAT_COROUTINE_S_START) {
                            return ;
                        }
                    } while (0);

                    ;
                    receive_idx = 4;

                    do {
                        akat_coroutine_state = 9;
akat_coroutine_l_9:

                        if (receive_byte() != AKAT_COROUTINE_S_START) {
                            return ;
                        }
                    } while (0);

                    ;
                    receive_idx = 5;

                    do {
                        akat_coroutine_state = 10;
akat_coroutine_l_10:

                        if (receive_byte() != AKAT_COROUTINE_S_START) {
                            return ;
                        }
                    } while (0);

                    ;
                    receive_idx = 6;

                    do {
                        akat_coroutine_state = 11;
akat_coroutine_l_11:

                        if (receive_byte() != AKAT_COROUTINE_S_START) {
                            return ;
                        }
                    } while (0);

                    ;
                    receive_idx = 7;

                    do {
                        akat_coroutine_state = 12;
akat_coroutine_l_12:

                        if (receive_byte() != AKAT_COROUTINE_S_START) {
                            return ;
                        }
                    } while (0);

                    ;
                    receive_idx = 8;

                    do {
                        akat_coroutine_state = 13;
akat_coroutine_l_13:

                        if (receive_byte() != AKAT_COROUTINE_S_START) {
                            return ;
                        }
                    } while (0);

                    ;

                    //Check CRC
                    if (ds18b20_onboard__connected) {
                        do {
                            akat_coroutine_state = 14;
                            return ;
akat_coroutine_l_14:
                            ;
                        } while (0);

                        ;
                        //Check CRC
                        u8 crc = akat_crc_add_bytes(0, ds18b20_onboard__scratchpad, 8);

                        if (ds18b20_onboard__scratchpad[8] == crc) {//CRC is OK
                            ds18b20_onboard__updated_deciseconds_ago = 0;
                            ds18b20_onboard__update_id += 1;
                            ds18b20_onboard__temperatureX16 = ((u16)ds18b20_onboard__scratchpad[1]) * 256 + ds18b20_onboard__scratchpad[0];
                        } else {//CRC is incorrect
                            ds18b20_onboard__crc_errors += AKAT_ONE;

                            if (!ds18b20_onboard__crc_errors) {//We can't go beyond 255
                                ds18b20_onboard__crc_errors -= AKAT_ONE;
                            }
                        }
                    }

                    if (ds18b20_ext__connected) {
                        do {
                            akat_coroutine_state = 15;
                            return ;
akat_coroutine_l_15:
                            ;
                        } while (0);

                        ;
                        //Check CRC
                        u8 crc = akat_crc_add_bytes(0, ds18b20_ext__scratchpad, 8);

                        if (ds18b20_ext__scratchpad[8] == crc) {//CRC is OK
                            ds18b20_ext__updated_deciseconds_ago = 0;
                            ds18b20_ext__update_id += 1;
                            ds18b20_ext__temperatureX16 = ((u16)ds18b20_ext__scratchpad[1]) * 256 + ds18b20_ext__scratchpad[0];
                        } else {//CRC is incorrect
                            ds18b20_ext__crc_errors += AKAT_ONE;

                            if (!ds18b20_ext__crc_errors) {//We can't go beyond 255
                                ds18b20_ext__crc_errors -= AKAT_ONE;
                            }
                        }
                    }
                }
            }

            do {
                akat_coroutine_state = 16;
                return ;
akat_coroutine_l_16:
                ;
            } while (0);

            ;
        }
    } while (0);

    AKAT_COLD_CODE;
    akat_coroutine_state = AKAT_COROUTINE_S_END;
akat_coroutine_l_end:
    return;
#undef akat_coroutine_state
#undef byte_to_send
#undef command_to_send
#undef has_connected_sensors
#undef read_bit
#undef receive_byte
#undef receive_idx
#undef send_byte
#undef send_command
#undef write_bit
}

;






AKAT_NO_RETURN void main() {
    asm volatile ("EOR r2, r2\nINC r2": "=r"(__akat_one__));
    usart0_reader__read_command__dequeue_byte__akat_coroutine_state = 0;
    usart0_writer__send_byte__akat_coroutine_state = 0;
    ds18b20_thread__akat_coroutine_state = 0;
    usart0_writer__byte_to_send = 0;
    usart0_writer__akat_coroutine_state = 0;
    usart0_writer__u8_to_format_and_send = 0;
    akat_every_decisecond_run_required = 0;
    D3_unused__init();
    D4_unused__init();
    D5_unused__init();
    D6_unused__init();
    D7_unused__init();
    D2_unused__init();
    C5_unused__init();
    C4_unused__init();
    C3_unused__init();
    C2_unused__init();
    C1_unused__init();
    C7_unused__init();
    C6_unused__init();
    ds18b20_onboard__init();
    ds18b20_ext__init();
    board_led__init();
    hdd_led__init();
    pwr_led__init();
    pwr_button__button_full__input__init();
    pwr_button__button_full__init();
    watchdog_init();
    usart0_init();
    timer1();
    //Init
    //Enable interrupts
    sei();

    //Endless loop with threads, tasks and such
    while (1) {
        performance_runnable();
        akat_on_every_decisecond_runner();
        ps_on_mainenance();
        pwr_button__button_full__runnable();
        watchdog_reset();
        usart0_writer();
        usart0_reader();
        ds18b20_thread();
    }
}

;





