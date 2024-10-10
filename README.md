## Планую розробити власний контролер автопілота та польотний контролер.


Вибір процесору:


STM32F030CCT6TR - LQFP-48 (0.5мм)
https://kosmodrom.ua/mikrokontroler/stm32f030cct6.html
47,5грн


STM32G070CBT6 - LQFP-48 (0.5мм)
https://kosmodrom.ua/mikrokontroler/stm32g070cbt6.html
47,5грн


Якшо треба більш компактну:
STM32G070KBT6 - LQFP-32 (0.8мм) - це буде простіше припаяти.
https://kosmodrom.ua/mikrokontroler/stm32g070kbt6.html



### Різні посилання

- https://habr.com/ru/articles/731514/ - Про керування VTX
- https://www.team-blacksheep.com/media/files/tbs_smartaudio_rev09.pdf
- https://betaflight.com/docs/wiki/guides/current/smartaudio
- https://github.com/EonClaw/STM32F411-Blackpill-INAV-FixedWing
- https://github.com/iNavFlight/OpenTX-Telemetry-Widget
- https://github.com/SolidGeek/VescUart

# army-simple-fc

Тут буде і плата і програма.

## Встановлення інструментів

https://github.com/riscv-software-src/homebrew-riscv

```bash
brew tap riscv-software-src/riscv
brew install riscv-tools
brew install riscv-openocd
brew test riscv-tools
/opt/homebrew/opt/riscv-gnu-toolchain/bin/riscv64-unknown-elf-gcc --version
```

/opt/homebrew/opt/riscv-openocd/bin/openocd



Во, подивиться це:

https://github.com/wagiminator/CH32V003-GameConsole/tree/main

Ще це:
https://github.com/wagiminator/MCU-Templates

Ще тут купа посилань:
https://github.com/wagiminator/Development-Boards/tree/main/CH32V003A4M6_DevBoard


https://github.com/AlessandroAU/ESP32-CRSF-Packet-Gen/blob/master/CRSF_testcode/CRSF.h
https://px4.github.io/Firmware-Doxygen/d9/dd2/crsf_8cpp_source.html

https://github.com/vedderb/bldc_uart_comm_stm32f4_discovery/blob/c19a6283e187040f25aa52e213673938db054d06/bldc_interface.c#L567

https://vedder.se/2015/10/communicating-with-the-vesc-using-uart/
