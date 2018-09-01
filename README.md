
# fixed frequency sine oscillator

generates two 50 or 67.5 hz sine waves one of them being shifted
by 90 degrees. as this runs on avr attiny45, the wave is generated
as a pwm signal, thus it needs to be filtered. given a 16MHz external
resonator the pwm frequency will be 62.5KHz.

check out src/ffso.c for an IO map.

 * build: ```make clean && make all```
 * flash: ```make program```
 * fuses: ```-U lfuse:w:0xef:m -U hfuse:w:0xdf:m -U efuse:w:0xff:m```
