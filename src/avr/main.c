
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
X_CPU$(cpu_freq = 16000000);

static const char HEX[16] = "0123456789abcdef";

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// PINS

// For safety reasons we set unused pins to read with pull up

X_UNUSED_PIN$(D3); // 1 PD3
X_UNUSED_PIN$(D4); // 2 PD4
// .................. 3 GND
// .................. 4 VCC
// .................. 5 GND
// .................. 6 VCC
// .................. 7 XT1
// .................. 8 XT2
X_UNUSED_PIN$(D5); // 9 PD5
X_UNUSED_PIN$(D6); // 10 PD6
X_UNUSED_PIN$(D7); // 11 PD7
// B0 ext temp ...... 12 PB0
// B1 CASE PWR BTN .. 13 PB1
// B2 onboard temp .. 14 PB2
// B3 CASE HDD LED .. 15 PB3, MOSI
// B4 CASE PWD LED .. 16 PB4, MISO
X_UNUSED_PIN$(D2); // 32 PD2
// TX USART ......... 31 PD1, TxD
// RX USART ......... 30 PD0, RxD
// .................. 29 Reset
X_UNUSED_PIN$(C5); // 28 PC5, ADC5
X_UNUSED_PIN$(C4); // 27 PC4, ADC4
X_UNUSED_PIN$(C3); // 26 PC3, ADC3
X_UNUSED_PIN$(C2); // 25 PC2, ADC2
X_UNUSED_PIN$(C1); // 24 PC1, ADC1
// C0 ATX PS ON ..... 23 PC0, ADC0
X_UNUSED_PIN$(C7); // 22 PC7, ADC7
// .................. 21 AGND
// .................. 20 AREF
X_UNUSED_PIN$(C6); // 19 PC6, ADC6
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

GLOBAL$() {
    STATIC_VAR$(u32 __current_main_loop_iterations);
    STATIC_VAR$(u32 main_loop_iterations_in_last_decisecond);
}

RUNNABLE$(performance_runnable) {
    __current_main_loop_iterations += 1;
}

X_EVERY_DECISECOND$(performance_ticker) {
    main_loop_iterations_in_last_decisecond = __current_main_loop_iterations;
    __current_main_loop_iterations = 0;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Uptime

GLOBAL$() {
    STATIC_VAR$(u32 uptime_deciseconds);
}

X_EVERY_DECISECOND$(uptime_ticker) {
    uptime_deciseconds += 1;
}


////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Main logic

GLOBAL$() {
    // Main state that dictates whether power should be on now or off as requested by operator.
    STATIC_VAR$(u8 power_should_be_on, initial = 0);
};

// DS18B20
X_DS18B20$(ds18b20_onboard, B2);
X_DS18B20$(ds18b20_ext, B0);

// Let pins
X_GPIO_OUTPUT$(board_led, B5);
X_GPIO_OUTPUT$(hdd_led, B3);
X_GPIO_OUTPUT$(pwr_led, B4);

// ATX PS ON Pin, the one that turns ATX on or OFF
// Settings this to output and ZERO will turn ATX ON
X_GPIO_INPUT_OUTPUT$(ps_on, C0);

// Just maintain our state on every iteration of event loop.
// Might be unnecessary but safe and simple for sure.
RUNNABLE$(ps_on_mainenance) {
    if (power_should_be_on) {
        // Drop pin to ground indicating that we want ATX to be in ON state.
        ps_on.set_output_mode();
        ps_on.set(0);
    } else {
        // Disconnect from pin. Pin will be pulled up to 5v by ATX itself
        // and ATX will turn it off it was ON.
        ps_on.set_input_mode();
    }
};

// ATX POWER BUTTON
X_BUTTON_LONG$(pwr_button, B1, long_press_deciseconds = 40) {
    METHOD$(void on_release()) {}

    METHOD$(void on_press()) {
        // Can turn on with a single press
        // Can't turn off with a short press.
        power_should_be_on = AKAT_ONE;
    }

    METHOD$(void on_long_press()) {
        // Can both turn on and turn off with a long press
        power_should_be_on = !power_should_be_on;
    }
};

// Activity indication
X_EVERY_DECISECOND$(activity_led) {
    STATIC_VAR$(u8 counter);

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
}

// Watchdog
X_WATCHDOG$(8s);

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// USART0 - Serial interface over USB Connection

X_INIT$(usart0_init) {
    // Set baud rate
    const u16 ubrr = akat_cpu_freq_hz() / (AK_USART0_BAUD_RATE * 8L) - 1;
    UBRR0H = ubrr >> 8;
    UBRR0L = ubrr % 256;
    UCSR0A = H(U2X0);

    // Set frame format
    UCSR0C = AK_USART0_FRAME_FORMAT;

    // Enable transmitter, receiver and interrupt for receiver (interrupt for 'byte is received')
    UCSR0B = H(TXEN0) | H(RXEN0) | H(RXCIE0);
}

// ----------------------------------------------------------------
// USART0(USB): Interrupt handler for 'byte is received' event..

GLOBAL$() {
    STATIC_VAR$(volatile u8 usart0_rx_bytes_buf[AK_USART0_RX_BUF_SIZE], initial = {});
    STATIC_VAR$(volatile u8 usart0_rx_overflow_count);
    STATIC_VAR$(volatile u8 usart0_rx_next_empty_idx);
    STATIC_VAR$(volatile u8 usart0_rx_next_read_idx);
}

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
THREAD$(usart0_writer, state_type = u8) {
    // ---- All variable in the thread must be static (green threads requirement)
    STATIC_VAR$(u8 crc);
    STATIC_VAR$(u8 byte_to_send);
    STATIC_VAR$(u8 u8_to_format_and_send);
    STATIC_VAR$(u16 u16_to_format_and_send);
    STATIC_VAR$(u32 u32_to_format_and_send);

    // ---- Subroutines can yield unlike functions

    SUB$(send_byte) {
        // Wait until USART0 is ready to transmit next byte
        // from 'byte_to_send';
        WAIT_UNTIL$(UCSR0A & H(UDRE0), unlikely);
        UDR0 = byte_to_send;
        crc = akat_crc_add(crc, byte_to_send);
    }

    SUB$(format_and_send_u8) {
        if (u8_to_format_and_send) {
            u8 h = u8_to_format_and_send / 16;
            if (h) {
                byte_to_send = HEX[h]; CALL$(send_byte);
            }

            u8 i = u8_to_format_and_send & 15;
            byte_to_send = HEX[i]; CALL$(send_byte);
        }
    }

    SUB$(format_and_send_u16) {
        u8_to_format_and_send = (u8)(u16_to_format_and_send / 256);
        if (u8_to_format_and_send) {
            CALL$(format_and_send_u8);

            u8_to_format_and_send = (u8)u16_to_format_and_send;
            byte_to_send = HEX[u8_to_format_and_send / 16]; CALL$(send_byte);
            byte_to_send = HEX[u8_to_format_and_send & 15]; CALL$(send_byte);
        } else {
            u8_to_format_and_send = (u8)u16_to_format_and_send;
            CALL$(format_and_send_u8);
        }
    }

    SUB$(format_and_send_u32) {
        u16_to_format_and_send = (u16)(u32_to_format_and_send >> 16);
        if (u16_to_format_and_send) {
            CALL$(format_and_send_u16);

            u16_to_format_and_send = (u16)u32_to_format_and_send;
            u8_to_format_and_send = (u8)(u16_to_format_and_send / 256);
            byte_to_send = HEX[u8_to_format_and_send / 16]; CALL$(send_byte);
            byte_to_send = HEX[u8_to_format_and_send & 15]; CALL$(send_byte);
            u8_to_format_and_send = (u8)u16_to_format_and_send;
            byte_to_send = HEX[u8_to_format_and_send / 16]; CALL$(send_byte);
            byte_to_send = HEX[u8_to_format_and_send & 15]; CALL$(send_byte);
        } else {
            u16_to_format_and_send = (u16)u32_to_format_and_send;
            CALL$(format_and_send_u16);
        }
    }

    // ---- Macro that writes the given status into UART
    // We also write some humand readable description of the protocol
    // also stuff to distinguish protocol versions and generate typescript parser code

    DEFINE_MACRO$(WRITE_STATUS, required_args = ["name", "id"], keep_rest_as_is = True) {
        byte_to_send = ' '; CALL$(send_byte);
        byte_to_send = '${id}'; CALL$(send_byte);

        % for arg in rest:
            /*
              COMMPROTO: ${id}${loop.index+1}: ${name.replace('"', "")}: ${arg}
              TS_PROTO_TYPE: "${arg}": number,
              TS_PROTO_ASSIGN: "${arg}": vals["${id}${loop.index+1}"],
            */
            <% [argt, argn] = arg.split(" ", 1) %>
            ${argt}_to_format_and_send = ${argn}; CALL$(format_and_send_${argt});
            % if not loop.last:
                byte_to_send = ','; CALL$(send_byte);
            % endif
        % endfor
    }

    // - - - - - - - - - - -
    // Main loop in thread (thread will yield on calls to YIELD$ or WAIT_UNTIL$)
    while(1) {
        // ----  - - - - -- - - - - -

        crc = 0;

        // WRITE_STATUS(name for documentation, 1-character id for protocol, type1 val1, type2 val2, ...)

        WRITE_STATUS$(Misc,
                      A,
                      u32 uptime_deciseconds,
                      u8 usart0_rx_overflow_count,
                      u32 main_loop_iterations_in_last_decisecond);

        WRITE_STATUS$("Onboard temperature",
                      B,
                      u8 ds18b20_onboard.get_crc_errors(),
                      u8 ds18b20_onboard.get_disconnects(),
                      u16 ds18b20_onboard.get_temperatureX16(),
                      u8 ds18b20_onboard.get_update_id(),
                      u8 ds18b20_onboard.get_updated_deciseconds_ago());

        WRITE_STATUS$("Controller temperature",
                      C,
                      u8 ds18b20_ext.get_crc_errors(),
                      u8 ds18b20_ext.get_disconnects(),
                      u16 ds18b20_ext.get_temperatureX16(),
                      u8 ds18b20_ext.get_update_id(),
                      u8 ds18b20_ext.get_updated_deciseconds_ago());

        // Protocol version
        byte_to_send = ' '; CALL$(send_byte);
        u8_to_format_and_send = AK_PROTOCOL_VERSION; CALL$(format_and_send_u8);

        // Done writing status, send: CRC\r\n
        byte_to_send = ' '; CALL$(send_byte);
        u8_to_format_and_send = crc; CALL$(format_and_send_u8);

        // Newline
        byte_to_send = '\r'; CALL$(send_byte);
        byte_to_send = '\n'; CALL$(send_byte);
    }
}

// ---------------------------------------------------------------------------------
// USART0(USB): This thread processes input from usart0_rx_bytes_buf that gets populated in ISR

THREAD$(usart0_reader) {
    // ---- all variable in the thread must be static (green threads requirement)
    STATIC_VAR$(u8 command_code);
    STATIC_VAR$(u8 command_arg);

    // Subroutine that reads a command from the input
    // Command is expected to be in the format as one printed out by 'send_status'
    // Command end ups in 'command_code' variable and optional
    // arguments end ups in 'command_arg'. If commands comes without argument, then
    // we assume it is 0 by convention.
    SUB$(read_command) {
        STATIC_VAR$(u8 dequeued_byte);
        STATIC_VAR$(u8 command_arg_copy);

        // Gets byte from usart0_rx_bytes_buf buffer.
        SUB$(dequeue_byte) {
            // Wait until there is something to read
            WAIT_UNTIL$(usart0_rx_next_empty_idx != usart0_rx_next_read_idx, unlikely);

            // Read byte first, then increment idx!
            dequeued_byte = usart0_rx_bytes_buf[usart0_rx_next_read_idx];
            usart0_rx_next_read_idx = (usart0_rx_next_read_idx + 1) & (AK_USART0_RX_BUF_SIZE - 1);
        }

        // Read arg into command_arg, leaves byte after arg in the dequeued_byte variable
        // so the caller must process it as well upon return!
        SUB$(read_arg_and_dequeue) {
            command_arg = 0;
            CALL$(dequeue_byte);
            if (dequeued_byte >= '0' && dequeued_byte <= '9') {
                command_arg = dequeued_byte - '0';
                CALL$(dequeue_byte);
                if (dequeued_byte >= '0' && dequeued_byte <= '9') {
                    command_arg = command_arg * 10 + (dequeued_byte - '0');
                    CALL$(dequeue_byte);
                    if (dequeued_byte >= '0' && dequeued_byte <= '9') {
                        if (command_arg < 25 || (command_arg == 25 && dequeued_byte <= '5')) {
                            command_arg = command_arg * 10 + (dequeued_byte - '0');
                            CALL$(dequeue_byte);
                        }
                    }
                }
            }
        }

    command_reading_start:
        // Read opening bracket
        CALL$(dequeue_byte);
        if (dequeued_byte != '<') {
            goto command_reading_start;
        }

        // Read command code
        // Verify that code is really a code letter
        CALL$(dequeue_byte);
        if (dequeued_byte < 'A' || dequeued_byte > 'Z') {
            goto command_reading_start;
        }
        command_code = dequeued_byte;

        // Read arg and save it as copy, note that read_arg aborts
        // when it either read fourth character in a row or a non digit character
        // so we have to process it (dequeued_byte) when the call to read_arg returns.
        // Verify that stuff that comes after the arg is a command code again!
        CALL$(read_arg_and_dequeue);
        if (dequeued_byte != command_code) {
            goto command_reading_start;
        }
        command_arg_copy = command_arg;

        // Read command arg once again (it comes after a copy of command code which we already verified)
        // We also verify that there is an > character right after the arg
        // And of course we verify that arg matches the copy we read before.
        CALL$(read_arg_and_dequeue);
        if (dequeued_byte != '>' || command_arg_copy != command_arg) {
            goto command_reading_start;
        }
    }

    // - - - - - - - - - - -
    // Main loop in thread (thread will yield on calls to YIELD$ or WAIT_UNTIL$)
    while(1) {
        // Read command and put results into 'command_code' and 'command_arg'.
        CALL$(read_command);

        switch(command_code) {
            /*
        case 'B':
            received_clock1 = command_arg;
            break;*/
        }
    }
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Main

X_MAIN$() {
    // Enable interrupts
    sei();
}
