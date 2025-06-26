# M5Stack StampS3 with DRV2605L Haptic Driver

This project demonstrates using the DRV2605L haptic driver module with the M5Stack StampS3 microcontroller.

## Features

- **Deep Sleep Mode**: Device enters deep sleep after startup to conserve power
- **Button Wake-up**: Four buttons can wake the device from deep sleep
- **LED Patterns**: Different blink patterns based on which button is pressed
- **Haptic Feedback**: DRV2605L generates haptic patterns when BTN4 is pressed

## Pin Configuration (Corrected)

### DRV2605L Connections
| DRV2605L Pin | M5Stack StampS3 Pin | Description |
|--------------|---------------------|-------------|
| VCC | 5V | Power supply |
| GND | GND | Ground |
| SDA | G44 | I2C data line |
| SCL | G42 | I2C clock line |
| IN | G40 | Input trigger pin |
| EN | G41 | Enable pin |

### LED Connections
| Component | Pin | Description |
|-----------|-----|-------------|
| LED | G39 | Status LED |

### Button Connections
| Component | Pin | Description |
|-----------|-----|-------------|
| BTN1 | G10 | 1-second blink pattern |
| BTN2 | G9 | 2-second blink pattern |
| BTN3 | G8 | 3-second blink pattern |
| BTN4 | G7 | Haptic pattern trigger |

## Operation

1. **Startup**: Device blinks LED once and enters deep sleep
2. **BTN1 (G10)**: Wakes device, blinks LED twice per second for 1 second, returns to sleep
3. **BTN2 (G9)**: Wakes device, blinks LED twice per second for 2 seconds, returns to sleep
4. **BTN3 (G8)**: Wakes device, blinks LED twice per second for 3 seconds, returns to sleep
5. **BTN4 (G7)**: Wakes device, enables DRV2605L, runs haptic patterns 50-123, returns to sleep

## Hardware Setup

### DRV2605L Module
- Connect VCC to 5V (as specified in connection diagram)
- Connect GND to GND
- Connect SDA to G44
- Connect SCL to G42
- Connect IN to G40
- Connect EN to G41

### Button Connections
Connect buttons between the respective GPIO pins and GND. Internal pull-up resistors are enabled in the code.

### LED Connection
Connect LED with appropriate current limiting resistor between G39 and GND.

## Dependencies

- Arduino ESP32 Core
- Wire library (included with Arduino)

## Usage

1. Upload the code to your M5Stack StampS3
2. Connect the DRV2605L module and buttons as specified in the pin configuration
3. Power on the device - it will blink once and go to sleep
4. Press any button to wake and trigger the corresponding action

## Notes

- The device uses ESP32's deep sleep functionality for power conservation
- Haptic patterns 50-123 are from the DRV2605L's built-in effect library
- All buttons use internal pull-up resistors
- The DRV2605L is only powered when needed (BTN4 press) to save power
- Uses ESP32's ext1 wakeup to support multiple button wake-up sources
- DRV2605L is powered from 5V as specified in the connection diagram

## Troubleshooting

- If haptic effects don't work, check I2C connections on G44 (SDA) and G42 (SCL)
- Ensure DRV2605L is receiving 5V power supply
- Check serial monitor for I2C communication errors
- Verify button connections and ensure they're properly grounded when pressed

## License

This project is licensed under the GNU General Public License v3.0 - see the LICENSE file for details.