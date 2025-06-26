# M5Stack StampS3 with DRV2605L Haptic Driver

This project demonstrates using the DRV2605L haptic driver module with the M5Stack StampS3 microcontroller.

## Features

- **Deep Sleep Mode**: Device enters deep sleep after startup to conserve power
- **Button Wake-up**: Four buttons can wake the device from deep sleep
- **LED Patterns**: Different blink patterns based on which button is pressed
- **Haptic Feedback**: DRV2605L generates haptic patterns when BTN4 is pressed

## Pin Configuration

| Component | Pin | Description |
|-----------|-----|-------------|
| LED | G39 | Status LED |
| BTN1 | G10 | 1-second blink pattern |
| BTN2 | G9 | 2-second blink pattern |
| BTN3 | G8 | 3-second blink pattern |
| BTN4 | G7 | Haptic pattern trigger |
| DRV2605L EN | G41 | Enable pin for DRV2605L |
| DRV2605L SDA | G13 | I2C data line |
| DRV2605L SCL | G15 | I2C clock line |

## Operation

1. **Startup**: Device blinks LED once and enters deep sleep
2. **BTN1 (G10)**: Wakes device, blinks LED twice per second for 1 second, returns to sleep
3. **BTN2 (G9)**: Wakes device, blinks LED twice per second for 2 seconds, returns to sleep
4. **BTN3 (G8)**: Wakes device, blinks LED twice per second for 3 seconds, returns to sleep
5. **BTN4 (G7)**: Wakes device, enables DRV2605L, runs haptic patterns 50-123, returns to sleep

## Hardware Setup

### DRV2605L Connections
- VCC → 3.3V
- GND → GND
- SDA → G13
- SCL → G15
- EN → G41

### Button Connections
Connect buttons between the respective GPIO pins and GND with pull-up resistors (internal pull-ups are enabled in code).

## Dependencies

- Arduino ESP32 Core
- Wire library (included with Arduino)

## Usage

1. Upload the code to your M5Stack StampS3
2. Connect the DRV2605L module and buttons as specified
3. Power on the device - it will blink once and go to sleep
4. Press any button to wake and trigger the corresponding action

## Notes

- The device uses ESP32's deep sleep functionality for power conservation
- Haptic patterns 50-123 are from the DRV2605L's built-in effect library
- All buttons use internal pull-up resistors
- The DRV2605L is only powered when needed (BTN4 press) to save power

## License

This project is licensed under the GNU General Public License v3.0 - see the LICENSE file for details.