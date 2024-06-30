[![english](https://img.shields.io/badge/language-english-blue?style=for-the-badge)](https://github.com/devmlb/PHY622x-custom-firmware/blob/master/README.md)
[![russian](https://img.shields.io/badge/language-russian-red?style=for-the-badge)](https://github.com/devmlb/PHY622x-custom-firmware/blob/master/README-ru.md)

# BTHome THB1, THB2, THB3, BTH01, TH05(HW: v1.3..1.6), TH05F

Custom firmware for Tuya devices based on the PHY622x2 chipset
| [THB1](https://pvvx.github.io/THB1) | [THB2](https://pvvx.github.io/THB2) | [THB3](https://pvvx.github.io/THB3) | [BTH01](https://pvvx.github.io/BTH01/) | [TH05_V1.3](https://pvvx.github.io/TH05-v1.3) | [TH05_V1.4](https://pvvx.github.io/TH-05) | [TH05F](https://pvvx.github.io/TH05F) |
|:---:|:---:|:---:|:---:|:---:|:---:|:---:|
| ![THB1](https://pvvx.github.io/THB1/img/THB1.jpg) | ![THB2](https://pvvx.github.io/THB2/img/THB2.jpg) | ![THB3](https://pvvx.github.io/THB3/img/THB3.jpg) | ![BTH01](https://pvvx.github.io/BTH01/img/BTH01.jpg) | ![TH05V1.3](https://pvvx.github.io/TH05-v1.3/img/TH05-V1.3.jpg) | ![TH05V1.4](https://pvvx.github.io/TH-05/img/TH05V14.jpg) | ![TH05F](https://pvvx.github.io/TH05F/img/TH05F.jpg)

All firmware supports any of the sensors: CHT8215 (CHT8310), CHT8305, AHT20..30.

* Software for setting up and BLE OTA [PHY62x2BTHome.html](https://pvvx.github.io/THB2/web/PHY62x2BTHome.html).
* Uploading OTA files to [PHY62x2BTHome.html](https://pvvx.github.io/THB2/web/PHY62x2BTHome.html) is automatic. You don't need to download files from this repository for OTA.
* To run [PHY62x2BTHome.html](https://pvvx.github.io/THB2/web/PHY62x2BTHome.html) offline, just copy the [html](https://github.com/pvvx/THB2/blob/master/bthome_phy6222/web/PHY62x2BTHome.html) file to a local folder.

## Boot and App firmware

* The [Boot](https://github.com/pvvx/THB2?tab=readme-ov-file#fw-boot-%D0%B8-ota) firmware has minimal functions. Boot is used only for OTA - for downloading the full-featured version of APP (Application - *.bin files).
* The device type can be distinguished externally by the smiley face symbol on the screen.

| Device | Boot file | OTA file | Printed circuit board labelling |
|:---:|:---:|:---:|:---:|
| [THB1](https://pvvx.github.io/THB1) | BOOT_THB1_v17.hex | THB1_v17.bin | no |
| [THB2](https://pvvx.github.io/THB2) | BOOT_THB2_v17.hex | THB2_v17.bin | no |
| [THB3](https://pvvx.github.io/THB3) | BOOT_THB2_v17.hex | THB2_v17.bin | no |
| [BTH01](https://pvvx.github.io/BTH01) | BOOT_BTH01_v17.hex | BTH01_v17.bin | no |
| [TH05_V1.4](https://pvvx.github.io/TH-05) | BOOT_TH05_v17.hex | TH05_v17.bin | TH05_V1.4, TH05_V1.5, TH05_V1.6 (chip: BL55028) |
| [TH05_V1.3](https://pvvx.github.io/TH05-v1.3) | BOOT_TH05D_v17.hex | TH05D_v17.bin | RSH-TH05-V1.3 (chip: BL55072) |
| [TH05F](https://pvvx.github.io/TH05F) | BOOT_TH05F_v17.hex | TH05F_v17.bin | TH05Y_V1.1, TH05Y_V1.2 (chip: QD01 2332 NT) |

The main firmware files, BOOT_XXX_vXX.hex for programming via USB-COM adapter and XXX_vXX.bin for OTA, are located in the [bin](https://github.com/pvvx/THB2/tree/master/bin) directory.

Files for updating boot via OTA are located in the [update_boot](https://github.com/pvvx/THB2/tree/master/update_boot) directory. **The process of updating boot via OTA is not safe. Please check the battery level before doing this. If boot is working fine, there is no need to update to the new version. The need to replace boot with a new version will be announced later.**

The current boot version is **v1.7** for devices with CHT8305 sensor. For other variants, boot is not required from version **v1.4**.

## Основные характеристики

! _При настройках по умолчанию_ !

* Интервал BLE рекламы в формате [BTHome v2](https://bthome.io) равен 5 секундам.
* Опрос датчика влажности и температуры производится каждый второй интервал BLE рекламы - период 10 секунд.
* Измерение напряжения батареи происходит каждую минуту.
* Кнопка используется для быстрого подключения к старым BT-адаптерам. Нажатие кнопки переключает интервал BLE рекламы на более короткий период (1562.5 мс). Действие продолжится 60 секунд, затем интервал восстановится на установленный в настройках.
* Измеренное среднее потребление от источника в 3.3В при сканировании термометров THB2 и BTH01 в пассивном режиме составляет до 8 мкА. Для TH05_V1.4 среднее потребление около 23 мкА - [таков ток установленных компонентов](https://github.com/pvvx/THB2/issues/8#issuecomment-1908982171). Остальные версии с экраном: при включенном LCD - от 12 до 14 мкА, c отключенным LCD - от 7 до 12 мкА.
* Запись итории каждые 30 минут
* Интервал соединения с учетом Connect Latency - 900 мс
* Поддерживаемые сенсоры температуры и влажности: AHT30, CHT8305, CHT8215, CHT8310
* Обработка входного контакта со счетчиком для передаваемых событий [Open/Close](https://github.com/pvvx/THB2/issues/10#issuecomment-1935169274)
* Обработка выходного контакта переключаемого по устанавливаемой температуре и/или влажности с гистерезисом
* Поддержка недорогого [повторителя рекламы BLE в Zigbee](https://github.com/pvvx/TLB2Z)

## История версий

| N | Описание |
|---|--- |
| 1.0 | Первая релизная версия |
| 1.1 | Добавлен триггер - вывод TX2 срабатывающий по установленным значениям температуры и/или влажности с гистерезисами. Передача состояния вывода RX2 при connect. Для термометров с экраном добавлен показ смайлика с "комфортом". Дополнены: изменение имени и MAC устройства. |
| 1.2 | Обработка и передача событий open/close со счетчиком с вывода маркированного "RX2" (для THB2 - "RX1"). |
| 1.3 | Добавлен THB1 и TH05V1.3. Следующий этап уменьшения потребления для версий с LCD дисплеем и опция отключения дисплея. |
| 1.4 | Стабилизация соединения для всех вариантов устройств. Добавлен [TH05F](https://pvvx.github.io/TH05F). Коррекция хода RTC. Изменено BLE имя для TH05_V1.3 на "TH05D". Добавлены файлы для обновления Boot по OTA. |
| 1.5 | Добавлен вариант шифрования BLE рекламы с помощью BindKey. |
| 1.6 | Добавлено усреднение расчета напряжения батареи, дублирование состояния контакта open/close и счетчика через 12..20 сек (кратно интервалу рекламы) после срабатывания и далее каждые 30 минут. |
| 1.7 | Исправление ошибки (> 42 C) для сенсора CHT8305 |

## Прошивка

Прошить устройство програмой Boot возможно через USB-COM адаптер с выходами на 3.3В:

1. Соединить GND, TX, RX, RTS–RESET, VCC (+3.3B).

| Адаптер | Устройство |
|---|---|
| GND | -Vbat |
| +3.3В | +Vbat |
| TX | RX1 |
| RX | TX1 |
| RTS | RESET |

Если на адаптере отсутствует RST пин, тогда следует замкнуть вывод RESET с GND(-Vbat) и быстро разомкнуть при старте скрипта (вомзожно потребуется несколько попыток)

Название контактов на устройстве смотреть в описании по ссылкам: [THB1](https://pvvx.github.io/THB1), [THB2](https://pvvx.github.io/THB2), [THB3](https://pvvx.github.io/THB3), [BTH01](https://pvvx.github.io/BTH01/), [TH05_V1.3](https://pvvx.github.io/TH05-v1.3), [TH05_V1.4](https://pvvx.github.io/TH-05)

2. Установить python3 и необходимые библиотеки

```txt
pip3 install -r requirements.txt
```

3. Скачать необходимый для конкретного устройства файл BOOT_xxx_vxx.hex из директории bin.

4. Запустить:

```txt
python3 rdwr_phy62x2.py -p COM11 -e -r wh BOOT_xxx_vxx.hex
```

5. Прошивка Boot завершена. Устройство работает. Адаптер можно отсоединить.

6. Загружаем полную версию прошивки по OTA. Для этого:
   1. Подаем питание на датчик (устройство).
   2. Переходим на [PHY62x2BTHome.html](https://pvvx.github.io/THB2/web/PHY62x2BTHome.html).
   3. Жмем кнопку `Соединение`, ищем устройство, подключаемся.
   4. После подключение переходим на вкладку `OTA`, выбираем необходимую прошивку и жмем `Старт`.

7. Устройство должно перезагрузиться и быть готово для использования.

Дополнительно:

* Для предварительного стирания всей Flash используйте опцию `-a`.

* Для предварительного стирания рабочей области Flash используйте опцию `-e`.

* С помощью USB-COM адаптера возможно сразу дописать основную прошивку (APP)
```
python3 rdwr_phy62x2.py -p COM11 -r we 0x10000 xxx_vxx.bin
```

## Сохранение оригинальной прошивки

1. Соединить GND, TX, RX, RTS–RESET, VCC (+3.3B).
2. Запустить:

```
python3 rdwr_phy62x2.py -p COM11 -r rc 0x11000000 0x80000 ff_thb2.bin
```

3. Полученный файл ff_thb2.bin сохранить.

## Восстановление оригинальной прошивки

* Оригинальную Tuya прошивку на конкретный тип устройства можно взять по ссылкам в таблице, находящейся в самом начале README.

1. Взять сохраненный файл ff_thb2.bin оригинальной прошивки.
2. Соединить GND, TX, RX, RTS–RESET, VCC (+3.3B).
3. Запустить:

```
python3 rdwr_phy62x2.py -p COM11 -b 1000000 -r we 0 ff_thb2.bin
```

Не все адаптеры USB-COM поддерживают 1Mbit. Тогда удалите опцию `-b 1000000` или выберите другой Baud rate.

4. Прошивка зашита. Устройство работает.

## Распределение Flash 512 килобайт

| Адрес | Описание | Размер |
|---|---|---|
| 0x00000 | Используется ROM | 8 килобайт |
| 0x02000 | Boot Info для ROM | 4 килобайта |
| 0x03000 | FW Boot с функцией OTA | 52 килобайта |
| 0x10000 | FW APP | 128 килобайт |
| 0x30000 | Запись истории | 304 килобайт |
| 0x7C000 | Сохранение настроек (EEP) | 16 килобайт |

## FW Boot и OTA

* `FW Boot` имеет функцию OTA, но не имеет функции записи истории и прочих дополнений. Служит для обработки OTA при любых неудачных или неправильных обновлениях.

* `FW APP` не имеет функции OTA, для OTA перезагружается в `FW Boot`. Имеет дополнительные функции и расширения.

Поддерживаемые функции и сервисы описываются включенными битами в 32-х битном поле `dev_id.services`.

`FW Boot` запускается по старту, и если не нажата кнопка, проверяет есть или нет запись `FW APP`. Если есть – запускает `FW APP`. Если кнопка при старте нажата - запускается `FW Boot`.

При соединении указывается:

_Software:_ **V**x.x - значит работает `FW APP`
_Software:_ **B**x.x - значит работает `FW Boot`

На термометрах с экраном, если не включено отображение времени, при первом старте показывает:

* "Bot 12" - работает Boot версия 1.2
* "APP 12" - работает APP версия 1.2  

Принудительно перезагрузиться в `FW Boot` из `FW APP` возможно двумя способами:

1. Отключить питание и удерживая кнопку включить питание.
2. Подать команду `7233` в меню `Service` программы PHY62x2BTHome.html и отключить соединение.

Полная перезагрузка - Подать команду `7201` в меню `Service` программы PHY62x2BTHome.html и отключить соединение.

Через USB-UART адаптер App можно записать сразу после boot. Пример:

```
python3 rdwr_phy62x2.py -p COM11 -e wh ./bin/BOOT_TH05V13_v13.hex
python3 rdwr_phy62x2.py -p COM11 -r we 0x10000 ./bin/TH05V13_v13.bin
```

## Событие Open/Close и счет импульсов

С версии 1.2 поддерживается опрос вывода подключенного к геркону или контакту, замыкающемуся на GND.

Максимальная частота переключения - 100 раз в секунду.

Если контакт имеет дребезг, тогда желательно зашунтировать контакт конденсатором.

При замыкании или размыкании передается блок из 5 BLE реклам следующих друг за другом через период в 50 мс.

При каждом событии "Open" прибавляется счетчик.

Значение счетчика передается совместно с каждым событием "Open/Close".

Входной контакт на плате термометра:

* На [THB1](https://pvvx.github.io/THB1) - маркирован как `RX`
* На [THB2](https://pvvx.github.io/THB2) - маркирован как `RX`
* На [THB3](https://pvvx.github.io/THB3) - маркирован как `RX`
* На [BTH01](https://pvvx.github.io/BTH01/) - маркирован как `RX2`
* На [TH05_V1.3](https://pvvx.github.io/TH05-v1.3)  - маркирован как `RX0`
* На [TH05_V1.4](https://pvvx.github.io/TH-05) - маркирован как `RX2`

![image](https://github.com/pvvx/THB2/assets/12629515/09f6f810-f2e2-4b61-9c84-f7c3770bb76a)

![image](https://github.com/pvvx/THB2/assets/12629515/40de4978-2d97-4f79-af9d-565236d0ba2a)

## Вывод управления внешним устройством по температуре и/или влажности

Контакт на печатной плате с маркировкой "TX" или "TX2" управляется с помощью уставок с гистерезисами по температуре и влажности.
Имеется возможность переключения на инверсное управление выводом.

Настройка производится в PHY62x2BTHome.html.

Работа выхода назначается с помощью установки значения гистерезиса:

* Если значение гистерезиса равно нулю - переключений не будет.
* Если значение гистерезиса больше нуля - переключение (включение) произойдет при значении ниже уставка + гистерезис.
* Если значение гистерезиса меньше нуля - переключение (включение) произойдет при значении выше уставка + гистерезис.

## Сброс основных настроек

Для установки основных параметров в начальные значения берете смартфон с Android и программу "nRFConnect".
Соединяетесь с устройством и в сервис `0xFCD2` c характеристикой `0xFFF4` записываете `56`.

![image](https://github.com/pvvx/THB2/assets/12629515/85cfaf06-e430-492e-930a-536afb163b5b)

* Нажатие/отпускание кнопки временно меняет интервал соединения для подключения. Коротко нажмите кнопку и подключитесь за 60 сек. Если не успеваете - нажимайте кнопку чаще...
* В прошивке проверяются все установки на соответствие стандартам Bluetooth SIG.
Если интервал более 10 сек, то поставит 10 сек - это максимум в стандарте. Так и другие параметры.
Но не все BT адаптеры могут работать с интервалом 10 сек для соединения.
* **При работе с BLE в Linux обязательно требуется изменение опций Bluez и/или патчи kernel!**
    
---

## Сборка прошивки

Для сборки прошивки используется GNU Arm Embedded Toolchain.

Для работы в Eclipce используете импорт проекта и установите toolchain.path.

Дополнительная информация по чипам [PHY62xx](https://github.com/pvvx/PHY62x2).
