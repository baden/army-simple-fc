## Планую розробити власний контролер автопілота та польотний контролер.




## Розподілення каналів.

Канал | Де на пульті | Призначення
:-----:|:--------:|:------
1 | Ail правий стік вліво-вправо  | Повертання та разворот на місці 
2 | Ele правий стік донизу-догори | Рух вперед-назад
3 | Thr лівий стік донизу-догори  | Керування газом двигуна
4 | Rud лівий стік вліво-вправо   | Не задіян
5 | SA Арм/Дісарм (лівий двохпозиційний стік)  | Дозвіл на керування. При вимиканні зупиняє рух та вимикає двигун
6 | лівий трьохпозиційний стік | Не задіян
7 | правий трьохпозиційний стік | Не задіян
8 | правий двохпозиційний стік | Не задіян
9 | Назначається на нижню кнопку зліва, без фіксації | Глушення мотору, або декомпресор для старту
10 | Назначається на середню кнопку зліва, без фіксації | Стартер
11 | Назначається на верхню кнопку зліва, з фіксацією | Дозвіл на запуск двигуна. Не під'єднан.
12 | Назначається на верхню та нижню кнопки з правої сторони. | Керування лебідкою.
13 |
14 |
15 |
16 |



## Звукові сигнали

При подачі живлення видається короткий звуковий сигнал.
При підключенні до пульта видається другий звуковий сигнал.
При активації ARM видається третій звуковий сигнал.
При диактивації ARM видається два звукових сигнала.
При втачанні звʼязку з пультом видається три звукових сигнала.
При втрачанні звʼязку більше ніж на хвилину, видається три довгих звукових сигнали.






# Нахрін ZephyrOS. Все шо далі не читайте. Може колись повернусь.


# По ПО.

## Налаштування робочого оточення

Я використовую (коли згадую) оце: [uv](https://github.com/astral-sh/uv)
Але зі звичайним python/pip це працює точнісенько так само.

```
uv venv --python 3.12
source .venv/bin/activate
uv pip install west

cd zephyrproject
west init -l army-simple-fc
west update
uv pip install -r zephyr/scripts/requirements.txt
#west config --global zephyr.base-prefer configfile
west zephyr-export
west build -s army-simple-fc -p -b army_simple_fc_v1_0

```

Збірка

```
west build -s army-simple-fc -p -b army_simple_fc_v1_0
```


Це вже не актуально:

```
uv venv --python 3.12
source .venv/bin/activate
uv pip install west
west init -m https://github.com/baden/fxa-custom-zephyr --mr main zephyrproject
cd zephyrproject
west update
west zephyr-export
uv pip install -r zephyr/scripts/requirements.txt
```

Напевно ще треба зробити 

```
cd zephyr
west sdk install
```

але я поки не хочу, бо це довго і багато буде качати

Пробуємо шось зібрати

```
west build -p always -b nucleo_g070rb zephyr/samples/basic/blinky -d build-blinky
```

Сука, воно знов зависає на 

Found GnuLd: /Users/baden/.local/opt/zephyr-sdk-0.16.4/arm-zephyr-eabi/bin/../lib/gcc/arm-zephyr-eabi/12.2.0/../../../../arm-zephyr-eabi/bin/ld.bfd (found version "2.38")

## Подальша інформація не актуальна

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
