const std = @import("std");

fn path(comptime suffix: []const u8) std.Build.LazyPath {
    return .{
        .cwd_relative = comptime ((std.fs.path.dirname(@src().file) orelse ".") ++ suffix),
    };
}

const hal = .{
    .source_file = path("/src/hals/ATmega328P.zig"),
};

pub const chips = struct {
    pub const atmega328p = .{
        .preferred_format = .hex,
        .chip = .{
            .name = "ATmega328P",
            .url = "https://www.microchip.com/en-us/product/atmega328p",
            .cpu = .avr5,
            .register_definition = .{
                .json = path("/src/chips/ATmega328P.json"),
            },
            .memory_regions = &.{
                .{ .offset = 0x000000, .length = 32 * 1024, .kind = .flash },
                .{ .offset = 0x800100, .length = 2048, .kind = .ram },
            },
        },
        .hal = hal,
        .binary_post_process = compileWithAvrGcc,
    };
};

pub const boards = struct {
    pub const arduino = struct {
        pub const nano = .{
            .preferred_format = .hex,
            .chip = chips.atmega328p.chip,
            .hal = hal,
            .board = .{
                .name = "Arduino Nano",
                .url = "https://docs.arduino.cc/hardware/nano",
                .source_file = path("/src/boards/arduino_nano.zig"),
            },
            .binary_post_process = compileWithAvrGcc,
        };

        pub const uno_rev3 = .{
            .preferred_format = .hex,
            .chip = chips.atmega328p.chip,
            .hal = hal,
            .board = .{
                .name = "Arduino Uno",
                .url = "https://docs.arduino.cc/hardware/uno-rev3",
                .source_file = path("/src/boards/arduino_uno.zig"),
            },
            .binary_post_process = compileWithAvrGcc,
        };
    };
};

fn compileWithAvrGcc(b: *std.Build, c_file: std.Build.LazyPath) std.Build.LazyPath {
    const compile_step = @fieldParentPtr(std.Build.CompileStep, "step", c_file.generated.step);
    if (compile_step.target.ofmt != .c) {
        // sanity check: if not building a C file, skip.
        return c_file;
    }

    const prog = b.findProgram(&.{"avr-gcc"}, &.{}) catch @panic("Please install avr-gcc!");

    const lib_dir = if (b.zig_lib_dir) |lib_dir|
        lib_dir.getPath(b)
    else
        b.pathJoin(&.{ std.fs.path.dirname(std.fs.path.dirname(b.zig_exe).?).?, "lib" });

    const linker_script = compile_step.linker_script.?;

    const compiler = b.addSystemCommand(&.{
        prog,
        "-g", // debug options
        "-mmcu=avr5", // compile for avr5
        "-Wno-builtin-declaration-mismatch", // hide weird zig warnings
        "-I",
        lib_dir,
        "-nostartfiles", // do not link _start from avrlibc
        "-ffreestanding", // do not link libc
    });

    compiler.addArg("-T");
    compiler.addFileArg(linker_script);

    compiler.addArg("-o");
    const elf_file = compiler.addOutputFileArg("firmware.elf");

    compiler.addFileArg(c_file);

    return elf_file;
}

pub fn build(b: *std.Build) void {
    _ = b;
    // const optimize = b.standardOptimizeOption(.{});
    // inline for (@typeInfo(boards).Struct.decls) |decl| {
    //     const exe = microzig.addEmbeddedExecutable(b, .{
    //         .name = @field(boards, decl.name).name ++ ".minimal",
    //         .source_file = .{
    //             .path = "test/programs/minimal.zig",
    //         },
    //         .backing = .{ .board = @field(boards, decl.name) },
    //         .optimize = optimize,
    //     });
    //     exe.installArtifact(b);
    // }

    // inline for (@typeInfo(chips).Struct.decls) |decl| {
    //     const exe = microzig.addEmbeddedExecutable(b, .{
    //         .name = @field(chips, decl.name).name ++ ".minimal",
    //         .source_file = .{
    //             .path = "test/programs/minimal.zig",
    //         },
    //         .backing = .{ .chip = @field(chips, decl.name) },
    //         .optimize = optimize,
    //     });
    //     exe.installArtifact(b);
    // }
}
