const std = @import("std");

const micro = @import("microzig");
const clk = micro.clock;
const interfaces = micro.core.experimental;

const peripherals = micro.chip.peripherals;
const USART0 = peripherals.USART0;
const I2C = peripherals.TWI;
const CPU = peripherals.CPU;

pub const cpu = micro.cpu;
const Port = enum(u8) {
    B = 1,
    C = 2,
    D = 3,
};

pub const clock = struct {
    pub const Domain = enum {
        cpu,
    };
};

pub fn parse_pin(comptime spec: []const u8) type {
    const invalid_format_msg = "The given pin '" ++ spec ++ "' has an invalid format. Pins must follow the format \"P{Port}{Pin}\" scheme.";

    if (spec.len != 3)
        @compileError(invalid_format_msg);
    if (spec[0] != 'P')
        @compileError(invalid_format_msg);

    return struct {
        pub const port: Port = std.meta.stringToEnum(Port, spec[1..2]) orelse @compileError(invalid_format_msg);
        pub const pin: u3 = std.fmt.parseInt(u3, spec[2..3], 10) catch @compileError(invalid_format_msg);
    };
}

pub const gpio = struct {
    fn regs(comptime desc: type) type {
        return struct {
            // io address
            const pin_addr: u5 = 3 * @enumToInt(desc.port) + 0x00;
            const dir_addr: u5 = 3 * @enumToInt(desc.port) + 0x01;
            const port_addr: u5 = 3 * @enumToInt(desc.port) + 0x02;

            // ram mapping
            const pin = @intToPtr(*volatile u8, 0x20 + @as(usize, pin_addr));
            const dir = @intToPtr(*volatile u8, 0x20 + @as(usize, dir_addr));
            const port = @intToPtr(*volatile u8, 0x20 + @as(usize, port_addr));
        };
    }

    pub fn setOutput(comptime pin: type) void {
        cpu.sbi(regs(pin).dir_addr, pin.pin);
    }

    pub fn setInput(comptime pin: type) void {
        cpu.cbi(regs(pin).dir_addr, pin.pin);
    }

    pub fn read(comptime pin: type) micro.gpio.State {
        return if ((regs(pin).pin.* & (1 << pin.pin)) != 0)
            .high
        else
            .low;
    }

    pub fn write(comptime pin: type, state: micro.gpio.State) void {
        if (state == .high) {
            cpu.sbi(regs(pin).port_addr, pin.pin);
        } else {
            cpu.cbi(regs(pin).port_addr, pin.pin);
        }
    }

    pub fn toggle(comptime pin: type) void {
        cpu.sbi(regs(pin).pin_addr, pin.pin);
    }
};

pub const uart = struct {
    pub const DataBits = enum {
        five,
        six,
        seven,
        eight,
        nine,
    };

    pub const StopBits = enum {
        one,
        two,
    };

    pub const Parity = enum {
        odd,
        even,
    };
};

pub fn Uart(comptime index: usize, comptime pins: interfaces.uart.Pins) type {
    if (index != 0) @compileError("Atmega328p only has a single uart!");
    if (pins.tx != null or pins.rx != null)
        @compileError("Atmega328p has fixed pins for uart!");

    return struct {
        const Self = @This();

        fn computeDivider(baud_rate: u32) !u12 {
            const pclk = interfaces.clock.get().cpu;
            const divider = ((pclk + (8 * baud_rate)) / (16 * baud_rate)) - 1;

            return std.math.cast(u12, divider) orelse return error.UnsupportedBaudRate;
        }

        fn computeBaudRate(divider: u12) u32 {
            return micro.clock.get().cpu / (16 * @as(u32, divider) + 1);
        }

        pub fn init(config: interfaces.uart.Config) !Self {
            const ucsz: u3 = switch (config.data_bits) {
                .five => 0b000,
                .six => 0b001,
                .seven => 0b010,
                .eight => 0b011,
                .nine => return error.UnsupportedWordSize, // 0b111
            };

            const upm: u2 = if (config.parity) |parity| switch (parity) {
                .even => @as(u2, 0b10), // even
                .odd => @as(u2, 0b11), // odd
            } else 0b00; // parity disabled

            const usbs: u1 = switch (config.stop_bits) {
                .one => 0b0,
                .two => 0b1,
            };

            const umsel: u2 = 0b00; // Asynchronous USART

            // baud is computed like this:
            //             f(osc)
            // BAUD = ----------------
            //        16 * (UBRRn + 1)

            const ubrr_val = try computeDivider(config.baud_rate);

            USART0.UCSR0A.modify(.{
                .MPCM0 = 0,
                .U2X0 = 0,
            });
            USART0.UCSR0B.write(.{
                .TXB80 = 0, // we don't care about these btw
                .RXB80 = 0, // we don't care about these btw
                .UCSZ02 = @truncate(u1, (ucsz & 0x04) >> 2),
                .TXEN0 = 1,
                .RXEN0 = 1,
                .UDRIE0 = 0, // no interrupts
                .TXCIE0 = 0, // no interrupts
                .RXCIE0 = 0, // no interrupts
            });
            USART0.UCSR0C.write(.{
                .UCPOL0 = 0, // async mode
                .UCSZ0 = @truncate(u2, (ucsz & 0x03) >> 0),
                .USBS0 = .{ .raw = usbs },
                .UPM0 = .{ .raw = upm },
                .UMSEL0 = .{ .raw = umsel },
            });

            USART0.UBRR0 = ubrr_val;

            return Self{};
        }

        pub fn canWrite(self: Self) bool {
            _ = self;
            return (USART0.UCSR0A.read().UDRE0 == 1);
        }

        pub fn tx(self: Self, ch: u8) void {
            while (!self.canWrite()) {} // Wait for Previous transmission
            USART0.UDR0 = ch; // Load the data to be transmitted
        }

        pub fn canRead(self: Self) bool {
            _ = self;
            return (USART0.UCSR0A.read().RXC0 == 1);
        }

        pub fn rx(self: Self) u8 {
            while (!self.canRead()) {} // Wait till the data is received
            return USART0.UDR0.*; // Read received data
        }
    };
}

// Terminology: I2C instead of TWI (TwoWireInterface) and controller/device, instead of master/slave
pub fn I2CController(comptime index: usize, comptime pins: interfaces.i2c.Pins) type {
    // TODO Add config options for pullup and power
    // TODO Check/Add status codes

    if (index != 0) @compileError("Index is not supported");
    if (pins.scl != null or pins.sda != null) @compileError("Custom pins are not supported");

    return struct {
        const Self = @This();

        pub fn init(config: interfaces.i2c.Config) !Self {
            // "Up to 400kHz data transfer speed" [1, p. 173]
            if (config.target_speed > 400_000) return interfaces.i2c.InitError.InvalidBusFrequency;
            // "[Device] operation does not depend on bit rate or prescaler settings,
            // but the CPU clock frequency in the [device] must be at least 16 times higher than the SCL frequency." [1, p. 180]
            if (!(interfaces.clock.get().cpu > config.target_speed * 16)) return interfaces.i2c.InitError.InvalidBusFrequency;

            // "[I]nternal pull-ups in the AVR pads can be enabled by setting the PORT bits corresponding to the SCL and SDA pins" [1, p. 179]
            const enable_internal_pullups = true;
            if (enable_internal_pullups) {
                peripherals.PORTC.PORTC |= 0b0011_0000; // SDA = PC4, SCL = PC5
            }

            // "The PRTWI bit ... must be written to zero to enable the 2-wire serial interface." [1, p. 174]
            const enable_i2c_power = true;
            if (enable_i2c_power) {
                CPU.PRR.modify(.{ .PRTWI = 0 });
            }

            // SCL_frequency = CPU_Clock_frequency/(16+2(TWBR)*(PrescalerValue)) [1, p. 180]
            const @"bitrate*prescaler" = (( interfaces.clock.get().cpu / config.target_speed) - 16) / 2;
            const bitrate_and_prescaler = for ([_]u8{ 1, 4, 16, 64 }, [_]u2{0b00, 0b01, 0b10, 0b11}) |prescaler, raw| {
                const bitrate = @"bitrate*prescaler" / prescaler;
                // Bitrate should be at least 10 [2, Sec. Note 5]
                if (10 <= bitrate and bitrate <= std.math.maxInt(@TypeOf(I2C.TWBR))) break .{ .bitrate = @truncate(u8, bitrate), .prescaler = raw };
            } else return interfaces.i2c.InitError.InvalidBusFrequency;

            I2C.TWBR = bitrate_and_prescaler.bitrate;
            I2C.TWSR.modify(.{ .TWPS = .{ .raw = bitrate_and_prescaler.prescaler } });
            I2C.TWCR.modify(.{ .TWEN = 1 });

            return Self{};
        }

        pub const WriteState = struct {
            address: u7,

            pub fn start(address: u7) !WriteState {
                // Steps 1 - 3 [1, p. 182] and example code at [1, p. 183]
                I2C.TWCR.modify(.{ .TWINT = 1, .TWSTA = 1, .TWEN = 1 });
                //debug_print("Waiting for status\r\n", .{});
                wait_for_i2c_to_finish_operation();
                return WriteState{ .address = address };
            }

            pub fn writeAll(self: *WriteState, bytes: []const u8) !void {
                send(@as(u8, self.address) << 1); // Address is 7 Bit, last bit is read(1)/write(0) [1, p. 175]
                for (bytes) |b| {
                    send(b);
                }
            }

            pub fn stop(self: *WriteState) !void {
                _ = self;
                // example code at [1, p. 183]
                I2C.TWCR.modify(.{ .TWINT = 1, .TWEN = 1, .TWSTO = 1 });
            }

            pub fn restart_read(self: *WriteState) !ReadState {
                try self.send_buffer(0);
                return ReadState{ .address = self.address };
            }
            pub fn restart_write(self: *WriteState) !WriteState {
                try self.send_buffer(0);
                return WriteState{ .address = self.address };
            }

            inline fn send(data: u8) void {
                I2C.TWDR = data;
                I2C.TWCR.modify(.{ .TWINT = 1, .TWEN = 1, .TWSTA = 0 });
                wait_for_i2c_to_finish_operation();
            }

            inline fn wait_for_i2c_to_finish_operation() void {
                // "When the [I2C] has finished an operation and expects application response, the TWINT flag is set." [1, p.182]
                while (I2C.TWCR.read().TWINT == 0) {}
            }
        };

        pub const ReadState = struct {
            address: u7,
            read_allowed: bool = true,

            pub fn start(address: u7) !ReadState {
                return ReadState{ .address = address };
            }

            /// Fails with ReadError if incorrect number of bytes is received.
            pub fn read_no_eof(self: *ReadState, buffer: []u8) !void {
                _ = self;
                _ = buffer;
            }

            pub fn stop(self: *ReadState) !void {
                _ = self;
            }

            pub fn restart_read(self: *ReadState) !ReadState {
                return ReadState{ .address = self.address };
            }
            pub fn restart_write(self: *ReadState) !WriteState {
                return WriteState{ .address = self.address };
            }
        };
    };
}

// References:
// [1] https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf
// [2] https://www.nongnu.org/avr-libc/user-manual/group__twi__demo.html
