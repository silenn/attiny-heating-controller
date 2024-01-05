# ATTINY13-Nozzles-Heater
Temperature controller based on Attiny13A/Atmega8A and DS18B20 digital thermometer.


## About
Initially developed for heating car washer-nozzles (yeah, I know about PTC thermistors existing), thats the reason why variables, macro, e.t.c are named in terms of heating, but of course this controller can be used in any other suitable applications, where some reaction is required when a certain temperature is reached.


## TL;DR
Scroll down to a deep bottom for the last chapter 'Short Instruction'.


## Features
- 3 output operating modes (`MODE_ROLLING`, `MODE_LINEAR`, `MODE_PULSE`);
- 2 Celsius degrees temperature thresholds (`THRESH_REG`, `THRESH_PER`);
- 2 logical outputs for load (`LOAD1`, `LOAD2`);
- 2 comparison options: if temperature **below** or **above** thresholds;
- no reset pin used.


## Pins and Fuses
| Function      | Attiny13A | Atmega8A |
| ------------- | --------- | -------- |
| DS12B20_DQ    | PB2       | PB0      |
| LOAD1         | PB3       | PB1      |
| LOAD2         | PB4       | PB2      |
| LED_Anode     | PB1       | PB5      |

| Fuse Bits     | Attiny13A | Atmega8A |
| ------------- | --------- | -------- |
| High Byte     | `0xFF`    | `0xD1`   |
| Low Byte      | `0x3A`    | `0xFF`   |

|               | Attiny13A | Atmega8A |
| ------------- | --------- | -------- |
| Sys Freq      | 9.6 MHz   | 8.0 MHz  |


## Short Functionality Brief
- Startup and reading 3 configuration bytes from an internal EEPROM: `THRESH_REG`, `THRESH_PER`, `RUNMODE`;
loop:
- A temperature measurement;
- Comparing the temperature with 2 thresholds: `THRESH_REG`, `THRESH_PER`;
- Changing a `system state` depending on the comparison result;
- Switching the load ouptuts depending on the system state and `RUNMODE`;
- Repeating the cycle every '16+1' seconds (defined by `OPERATING_CYCLE`).

The system can be in 4 different states: `H_OFF`, `H_PER`, `H_REG`, `H_ERR`.
H_PER and H_REG switches the load outputs in depending on `RUNMODE`.
H_OFF drives both outputs to low.
H_ERR drives both outputs to low, in 1 second returns to the loop beginning by skipping the full operating cycle time.


## EEPROM Configuration Bytes
Config bytes are read after the system startup. All three are interpreted as `int8_t`.

### EEPROM Data
| Address | Param Name | Description                                                                  |
| ------- | -----------| ---------------------------------------------------------------------------- |
| 0x00    | THRESH_REG | Regular temperature threshold for switching to H_REG state (\*C)             |
| 0x01    | THRESH_PER | Periodic temperature threshold for switching to H_PER state (\*C)            |
| 0x02    | RUNMODE    | Operating mode, defines dependency between the system state and output pins  |

### RUNMODE and Corresponding Value
| RUNMODE       | Value     |
| ------------- | --------- |
| MODE_ROLLING  | 0         |
| MODE_LINEAR   | 1         |
| MODE_PULSE    | any other |

**For example:**
If the first 3 bytes of EEPROM are: `0xEC 0xF6 0x01` - it will be the LINEAR (0x01) mode with -10 (0xF6) THRESH_PER and -20 (0xEC) THRESH_REG.
Erased EEPROM `0xFF 0xFF 0xFF` will be interpreted as PULSE mode and -1 for both thresholds.

### Thresholds and Comparison Process
THRESH_REG's priority is higher than THRESH_PER's. It means if the THRESH_PER value is lower than the THRESH_REG value, the system will never enters in H_PER state.
A simplefied relation of the measured temperature to the system state is next:

```C
if (t < THRESH_REG) state = H_REG;
else if (t < THRESH_PER) state = H_PER;
else state = H_OFF;
```

Comparison logic can be inverted by using a macro 'ACT_IF_T_ABOVE_THRESH' (THRESH_REG's priority is still the highest):

```C
if (t > THRESH_REG) state = H_REG;
else if (t > THRESH_PER) state = H_PER;
else state = H_OFF;
```

### Hysteresis
A hysteresis is done only for H_REG->H_PER transition due to lack of the flash memory, the value is defined by `HYSTERESIS` macro.


## System States
| RUNMODE       | State         | LOAD1 State    | LOAD2 State    |
| ------------- | ------------- | -------------- | -------------- |
| any mode      | H_OFF         | Low            | Low            |
| any mode      | H_ERR         | Low            | Low            |
| MODE_ROLLING  | H_PER         | **High**       | Low            |
| -//-          |   H_REG       | Low            | **High**       |
| MODE_LINEAR   | H_PER         | **High**       | Low            |
| -//-          |   H_REG       | **High**       | **High**       |
| MODE_PULSE    | H_PER         | `Periodic`     | `Periodic`     |
| -//-          |   H_REG       | **High**       | **High**       |


### Periodic State in Pulse Mode
Both outputs are driven high for some pulses number. There are could be up to 4 pulses (default value defined as `PULSES_NUM`) during OPERATING_CYCLE. 5 pulses = regular heating. The temperature range, designated as THRESH_REG and THRESH_PER, is devided by PULSES_NUM - it forms additional thresholds. Reaching the each additional threshold adds the additional pulse to operating cycle. If there is a division remainder, the gap to THRESH_REG will be filled with the maximum number of pulses.

**For example:**

`
REG threshold -15 | PER threshold +2\
Temperature Range: 17 degrees\
Temperature step for adding more pulses: 4 degrees\
0  t: 7    | state:  OFF, heating pulses: 0/5
1  t: 6    | state:  OFF, heating pulses: 0/5
2  t: 5    | state:  OFF, heating pulses: 0/5
3  t: 4    | state:  OFF, heating pulses: 0/5
4  t: 3    | state:  OFF, heating pulses: 0/5
5  t: 2    | state:  PER, heating pulses: 1/5 <-- periodic threshold defined as THR_PER, 1 pulse
6  t: 1    | state:  PER, heating pulses: 1/5
7  t: 0    | state:  PER, heating pulses: 1/5
8  t: -1   | state:  PER, heating pulses: 1/5
9  t: -2   | state:  PER, heating pulses: 2/5 <-- THR_PER minus (Step x1)
10 t: -3   | state:  PER, heating pulses: 2/5
11 t: -4   | state:  PER, heating pulses: 2/5
12 t: -5   | state:  PER, heating pulses: 2/5
13 t: -6   | state:  PER, heating pulses: 3/5 <-- THR_PER minus (Step x2)
14 t: -7   | state:  PER, heating pulses: 3/5
15 t: -8   | state:  PER, heating pulses: 3/5
16 t: -9   | state:  PER, heating pulses: 3/5
17 t: -10  | state:  PER, heating pulses: 4/5 <-- THR_PER minus (Step x3)
18 t: -11  | state:  PER, heating pulses: 4/5
19 t: -12  | state:  PER, heating pulses: 4/5
20 t: -13  | state:  PER, heating pulses: 4/5
21 t: -14  | state:  PER, heating pulses: 4/5 <-- extra 4 pulses, bcs (THR_REG - THR_PER) % PULSES_NUM > 0
22 t: -15  | state:  REG, heating pulses: 5/5 <-- THR_REG
23 t: -16  | state:  REG, heating pulses: 5/5
24 t: -17  | state:  REG, heating pulses: 5/5
25 t: -18  | state:  REG, heating pulses: 5/5
26 t: -19  | state:  REG, heating pulses: 5/5
27 t: -20  | state:  REG, heating pulses: 5/5
28 t: -19  | state:  REG, heating pulses: 5/5
29 t: -18  | state:  REG, heating pulses: 5/5
30 t: -17  | state:  REG, heating pulses: 5/5
...
`

A duration of each pulse is defined by `PULSE_LENGTH`:
```C
#define PULSES_NUM        4
#define PULSE_LENGTH      4
#define OPERATING_CYCLE   ((PULSES_NUM * PULSE_LENGTH) + PULSE_LENGTH)
```


## Indication
| Heating State      | LED                                             |
| ------------------ | ----------------------------------------------- |
| Sys Fault*         | Turned off                                      |
| H_OFF              | One short blink per second                      |
| H_PER              | One-second-toggling                             |
| H_REG              | Constantly lights                               |
| H_ERR**            | Lights with one short turning off every second  |

Sys Fault* - no power supply, MCU is not programmed, poor physical contact of conductors.
H_ERR** - see the next chapter 'Error Catching'.


## Error Catching
Program detects a DS18B20 presence and checks all received data using CRC. If something wrong, system falls to H_ERR state and tries to repeat operating cycle after 1.25 sec.
But it allows to correctly detect the absence of the sensor only if pull-up resistor is connected to a DQ-pin. For cases of missed pull-up, there is added a checking of 5th and 7th byte of a scratchpad - it's reserved 0xFF and 0x10 (DS18B20 datasheet, 19-7487; Rev 6; 7/19 - 'Figure 9. DS18B20 Memory Map'). If skipping of this check is needed, just uncomment the definition `SENS_SKIP_RESERVED_BYTES_CHECK`.


### Compile
Program compiled with AVR-Toolchain v3.4.2.1573, -Os -std=gnu99
`----------------
Device: attiny13
\
Program:    1022 bytes (99.8% Full)
(.text + .data + .bootloader)
\
Data:          2 bytes (3.1% Full)
(.data + .bss + .noinit)`


## Short Instruction
- Take an Attiny13A;
- Programm flash memory with 'heating_controller.hex';
- Programm Fuses: High Byte - `0xFF`, Low Byte - `0x3A`;
- Programm EEPROM with 'eeprom.bin';
- Connect DS18B20's DQ pin to PB2;
- Optional connect LED to PB1;
- Now your Attiny13A will drive PB3 and PB4 high in the pulse mode if temperature is below -4 Celsius degrees and in the regular mode if temperature below -16;
- Enjoy.


## License
CC-BY-SA


