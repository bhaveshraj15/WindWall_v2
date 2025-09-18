# Wind-Wall (v2.0)
## ESP-NOW Communication and Control System

### Overview
This project implements a comprehensive ESP32-based communication and control system using ESP-NOW protocol. It enables wireless communication between multiple ESP32 devices with support for motor control, LED control, and file-based command execution.

### Features
* ESP-NOW Communication: Secure peer-to-peer wireless communication with encryption support
* Dynamic Peer Discovery: Automatic device discovery and pairing mechanism
* Motor Control: PWM control for up to 6 motors/ESCs with standard 50Hz frequency
* LED Control: RGB LED control with configurable colors and timing
* UART Command Interface: Interactive command system via serial interface
* File System Integration: SPIFFS support for command script files with loop and delay commands
* Security: PMK and LMK encryption for secure communication
* Multi-Device Control: Support for controlling multiple devices with single commands

### Pin Configuration
* Motor Pins: GPIO 5, 6, 7, 15, 16, 17
* LED Pin: GPIO 48 (on-board led)

### Setup
1. Clone the repository and open the project in your preferred ESP32 development environment (ESP-IDF)
2. Configure the hardware according to the pin configuration above
3. Update known MAC addresses in the known_macs array with your devices' MAC addresses
4. Build and flash the project to your ESP32 devices
5. Monitor the serial output at 115200 baud to see device status and execute commands

### Usage
#### Basic Commands
* `discover` - Initiate peer discovery
* `list` - List all known peers
* `send <MAC> <message>` - Send message to peer
* `remove <MAC>` - Remove a peer
* `me` - Display MAC address of current device
* `help` - Show available commands

#### LED Control
```
led <r> <g> <b> <timeout_ms> <blank_ms> [mac1] [mac2] [mac3] ...
```
Examples:
* `led 255 0 0 1000 500` - Set red LED for 1 second with 500ms blank period (local and all peers)
* `led 0 255 0 500 200 F0:9E:9E:1E:4A:E4` - Set green LED on specific device
* `led 0 0 255 300 100 F0:9E:9E:1E:4A:E4 F0:9E:9E:1E:4A:84` - Set blue LED on multiple devices

#### Motor Control
```
motor <m1> <m2> <m3> <m4> <m5> <m6> <delay time> [mac1] [mac2] [mac3] ...
```
* `motor 1500 1500 1500 1500 1500 1500 500` - Set all motors to 1500μs for 500ms (local)
* `motor 1500 1500 1500 1500 1500 1500 500 F0:9E:9E:1E:4A:E4` - Control motors on specific device
* `motor 1300 1300 1300 1300 1300 1300 500 F0:9E:9E:1E:4A:E4 F0:9E:9E:1E:4A:84` - Control motors on multiple devices

#### File-based Commands
Create command files in SPIFFS and execute them with:
```
execute <file_path>
```
Example file content:
```
# Motor sequence for multiple devices
motor 1500 1500 1500 1500 1500 1500 500 F0:9E:9E:1E:4A:E4 F0:9E:9E:1E:4A:84
hold 600
motor 1300 1300 1300 1300 1300 1300 500 F0:9E:9E:1E:4A:E4 F0:9E:9E:1E:4A:84
hold 600
```

##### Advanced Features
* Loop commands: Repeat a sequence of commands
* Hold commands: Pause execution for specified milliseconds
* MAC management: Add and remove devices dynamically
* Automatic reconnection: Devices automatically reconnect if connection is lost

### Security
The system uses two-layer encryption:
* Primary Master Key (PMK): `0123456789abcdef`
* Local Master Key (LMK): `abcdef0123456789`

### ⚠️Troubleshooting⚠️
1. Devices not connecting: Ensure all devices are powered on and in range
2. Motor not responding: Check ESC calibration and power supply
3. File not found: Ensure SPIFFS is properly initialized and file exists
4. Command not working: Check syntax and parameters using the `help` command
