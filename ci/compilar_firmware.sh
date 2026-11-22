#!/bin/bash
# Compila robot_control.ino para el Arduino Nano (ATmega328P) con avr-gcc, sin el
# Arduino IDE. Uso: ci/compilar_firmware.sh <sketch.ino> <carpeta de la biblioteca PID>
set -euo pipefail
INO="$(realpath "$1")"
PID_DIR="$(realpath "$2")"
CORE=/usr/share/arduino/hardware/arduino/avr/cores/arduino
VARIANT=/usr/share/arduino/hardware/arduino/avr/variants/eightanaloginputs
BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT
cd "$BUILD"
DEFS="-mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=10806 -DARDUINO_AVR_NANO -DARDUINO_ARCH_AVR -I$CORE -I$VARIANT"
CXXFLAGS="-c -g -Os -std=gnu++11 -fpermissive -fno-exceptions -ffunction-sections -fdata-sections -fno-threadsafe-statics $DEFS"
for f in "$CORE"/*.c; do avr-gcc -c -g -Os -w -std=gnu11 -ffunction-sections -fdata-sections $DEFS "$f" -o "$(basename "$f").o"; done
for f in "$CORE"/*.cpp; do avr-g++ -w $CXXFLAGS "$f" -o "$(basename "$f").o"; done
avr-gcc -c -g -x assembler-with-cpp $DEFS "$CORE/wiring_pulse.S" -o wiring_pulse.S.o
avr-gcc-ar rcs core.a ./*.o
avr-g++ -w $CXXFLAGS -I"$PID_DIR" "$PID_DIR/PID_v1.cpp" -o pid.o
{ echo '#include <Arduino.h>'; cat "$INO"; } > sketch.cpp
avr-g++ -Wall -Wextra -Werror $CXXFLAGS -I"$PID_DIR" sketch.cpp -o sketch.o
avr-gcc -Os -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=atmega328p sketch.o pid.o core.a -lm -o sketch.elf
avr-size -C --mcu=atmega328p sketch.elf | grep -E "Program|Data"
echo "OK: $(basename "$INO") compila para el Arduino Nano"
