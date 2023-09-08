# clicky-turtle-c3

This repository contains a firmware implementation of a BLE-HID keyboard controller for the ESP32-C3 microcontroller.

## Usage

The firmware communicates over an SPI interface, with the other device behaving as the SPI master. The interface uses Motorola SPI mode 0, meaning that the clock polarity is active low, and the phase polarity is first/rising edge. The maximum supported clock speed of the interface is 1MHz.
All signals are digital and 3.3V. The pinout for the ESP32-C3-WROOM-02 module can be found on page 10/Section 3.1 of [the datasheet](https://www.espressif.com/sites/default/files/documentation/esp32-c3-wroom-02_datasheet_en.pdf),
and a description of how the pins are used in the clicky-turtle-c3 firmware is below:

| #  | Name   | Functionality                                                                                    |
|----|--------|--------------------------------------------------------------------------------------------------|
| 1  | 3V3    | Connect to 3V3 (500mA min) power supply with 100u/1206 and 1u/0603 ceramic decoupling capacitors |
| 2  | EN     | Drive high to enable device. Drive low and then high to reset.                                   |
| 3  | IO4    | SPI_CS                                                                                           |
| 4  | IO5    | SPI_SCK                                                                                          |
| 5  | IO6    | SPI_MISO                                                                                         |
| 6  | IO7    | SPI_MOSI                                                                                         |
| 7  | IO8    | Do not connect; leave floating                                                                   |
| 8  | IO9    | Do not connect; leave floating                                                                   |
| 9  | GND    | Connect to GND                                                                                   |
| 10 | IO10   | Unused; leave floating                                                                           |
| 11 | RXD    | Unused; leave floating                                                                           |
| 12 | TXD    | Debug console output, recommended to be exposed via pin header or test point for debugging.      |
| 13 | IO18   | Unused; leave floating                                                                           |
| 14 | IO19   | Unused; leave floating                                                                           |
| 15 | IO3    | SPI_IDLE                                                                                         |
| 16 | IO2    | Connect to 3V3                                                                                   |
| 17 | IO1    | CONN_STATE[0]                                                                                    |
| 18 | IO0    | CONN_STATE[1]                                                                                    |
| 19 | GNDPAD | Exposed ground pad on bottom of module. Either connect to GND or leave unconnected.              |

### SPI_IDLE pin

The `SPI_IDLE` pin indicates whether the SPI interface is able to accept new transactions. If the pin is low, the device 
is able to accept new transactions. If it is high, the device is busy and the result of executing a transaction is 
undefined.  

### CONN_STATE pins

The current BLE connection state can be read from the `CONN_STATE` pins:

| CONN_STATE[0] | CONN_STATE[1] | State description                |
|---------------|---------------|----------------------------------|
| 0             | 0             | Unused/module is off             |
| 0             | 1             | Disconnected                     |
| 1             | 0             | Controller is requesting passkey |
| 1             | 1             | Connected                        |


### Advertising

The clicky-turtle-c3 will advertise via BLE when it is not connected to any BLE controller. 

If you are using firmware v0.1.0, the name of the device will 
be `clicky-c3-X` where X is a hexadecimal number based on the hardware serial number of the radio.

If you are using firmware v0.1.1, the name will be `c-X` where X is a unique hexadecimal number.

### SPI transactions

These commands are possible to execute over the SPI interface:

1. Send the BLE passkey for pairing purposes (`PASSKEY`; command code is `0xAB`)
2. Send keyboard scan codes (`SCANCODE`; command code is `0xCD`)
3. Send a generic report (`SIDEBAND_UPLINK`; command code is `0xEF`)
4. Read the latest generic report sent to the device (`SIDEBAND_DOWNLINK`; command code is `0xA1`)
5. Erase all paring information and restart (`UNPAIR`; command code `0xB2`)
6. Empty command to ensure the SPI interface is set up correctly; this should be run once on startup but can be run at any time and any number of times (`SYNC`, command code `0xC3`)

All transactions begin with the command code from above as the first byte. Any transactions that do not have these bytes 
first will be ignored. You should ignore all data transmitted from the device (ie over the `MISO` pin) except in the 
case of the `SIDEBAND_DOWNLINK` command. 

#### Sending the passkey

When pairing with a keyboard, the BLE central device (most likely your PC) will display a 6 digit number that needs to be
sent to the clicky-turtle-c3 firmware. To send this, you need to perform a 7 byte transaction, where the first byte is the 
command code, and the remaining 6 bytes are the digits in the passkey (one digit per byte). 

Example transaction for passkey 153472: 

| Byte number | Value (hex) | Description            |
|-------------|-------------|------------------------|
| 0           | 0xAB        | `PASSKEY` command code |
| 1           | 0x01        | Passkey first digit    |
| 2           | 0x05        | Passkey second digit   |
| 3           | 0x03        | Passkey third digit    |
| 4           | 0x04        | Passkey fourth digit   |
| 5           | 0x07        | Passkey fifth digit    |
| 6           | 0x02        | Passkey sixth digit    |

Sending a passkey to the clicky-turtle-c3 should only occur when the `CONN_STATE` pins indicate that a passkey is being 
requested. If requested, you must then send the `PASSKEY` transaction at most once per second. If the `CONN_STATE` 
pins indicate that the passkey is still requested 1 second after the previous execution, you should send the `PASSKEY` 
transaction again. This process should be repeated until the `CONN_STATE` pins change.

#### Sending keyboard scan codes

The clicky-turtle-c3 is designed to receive 8 byte [standard HID reports](https://usb.org/sites/default/files/hut1_3_0.pdf) 
and forward them to the PC. The SPI transaction to send a report is exactly 9 bytes long; the first byte is the `SCANCODE` 
command byte, and the remaining bytes are the HID report bytes.

A HID report contains information about which modifier keys are pressed (Shift, Alt, etc) and which keys are pressed 
(also referred to as scan codes). A list of standard modifier keys and scan codes can be found [here](https://gist.github.com/MightyPork/6da26e382a7ad91b5496ee55fdc73db2).

The first byte of the report contains all of the pressed modifier keys bitwise OR-ed together. The second byte is always 
`0x00` and the remaining bytes are scan codes corresponding to keys that have been pressed. An example HID report is 
shown below; this report will indicate that `Left CTRL`, `Left Shift` and `B` are all being held down:

| Byte number | Value | Description                                   |
|-------------|-------|-----------------------------------------------|
| 0           | 0x03  | Modifier masks (`Left CTRL` and `Left Shift`) |
| 1           | 0x00  | Reserved, always `0x00`                       |
| 2           | 0x05  | Pressed key 1 (`B`)                           |
| 3           | 0x00  | Pressed key 2 (None)                          |
| 4           | 0x00  | Pressed key 3 (None)                          |
| 5           | 0x00  | Pressed key 4 (None)                          |
| 6           | 0x00  | Pressed key 5 (None)                          |
| 7           | 0x00  | Pressed key 6 (None)                          |

Note from this structure that only 6 keys can be sent at once; this is a limitation of the HID standard. All other keys 
not included in the report are assumed to be unpressed. To indicate that a key has been released, send a report that 
does not contain the scan code for that key. It is possible to send an empty report (ie `00 00 00 00 00 00 00 00`) to 
indicate that all keys have been released.

#### Using the sideband communication functionality

You can send and receive arbitrary data over the wireless link using the `SIDEBAND_UPLINK` and `SIDEBAND_DOWNLINK` 
commands. In this case, `SIDEBAND_UPLINK` command is for data that is sent from the device to the PC, and `SIDEBAND_DOWNLINK` 
is for data sent from the PC to the device. All packets of data to be transferred must be exactly 32 bytes.

To send data from the device to the PC, the SPI transaction must be exactly 33 bytes long. The first byte is the `SIDEBAND_UPLINK`
command code, and the remaining 32 bytes are any arbitrary data you wish to send to the PC. If the transaction is 
shorter or longer than 33 bytes, it will be ignored.

To read data that was sent from the PC, a 32 byte transaction is used. The `SIDEBAND_DOWNLINK` command code is sent as 
the first byte, and then 31 bytes of `0x00` (or any other data) is sent to make up the rest of the packet. The arbitrary
data from the PC will be sent simulatenously by the clicky-turtle-c3 firmware over the SPI interface in response. If the
transaction is longer than 32 bytes, the extra data returned is undefined.

For an example of sending uplinks and downlinks with python, please see the `uplink_downlink_example.py` file. 

**Important:** Due to some peculiarities in the way that the ESP32-C3 MCU implements SPI, you may not always 
receive the latest packet when performing the `SIDEBAND_DOWNLINK` transaction, and may instead receive the previous 
packet. It is therefore recommended that you perform the `SIDEBAND_DOWNLINK` transaction often and repeatedly to ensure 
that you receive the latest data available.

#### Erasing pairing information

In order to connect your device to a different PC, you may need to erase the pairing information stored in the 
clicky-turtle-c3 firmware. In order to do this, an SPI transaction of exactly 8 bytes is used. All bytes in this 
transaction must be the command code for `UNPAIR` (ie `0xB2`). If the request is received successfully, this may take a 
second or two to completely execute. 

### Building and flashing the firmware

(Note for UQ students: You do not need to flash the firmware yourself. The module comes pre-flashed with the firmware.)

This firmware can be built using ESP-IDF v5.0 (release tag). However, there is an issue in this release that prevents proper pairing 
with Mac operating systems. To correct this, update the function `create_hid_db` in `ble_hidd.c`
and update the HID over GATT (ie HOGP) configuration as per the `idf-5-fix.patch` file included in this repository.

Do not use ESP-IDF v5.1 as it is breaks HID enumeration on Windows. The firmware can be flashed using `esptool.py` with 
the firmware release in this repository. The binary is merged, so you just need to flash it to address 0.
