# Smart Home Firmware

ESP32-WROOM firmware for smart home applications using PlatformIO with ESP-IDF framework.

## Prerequisites

- Python 3.6+ (required by PlatformIO)
- USB driver for ESP32 (CP210x or CH340)

## Setup

1. Clone this repository:
```bash
git clone https://github.com/kibshh/smart-home-firmware.git
cd smart_home_firmware
```

2. Set up Python virtual environment and install PlatformIO:
```bash
make setup
```

## Usage

All commands use the Makefile, which automatically handles the virtual environment:

### Building
```bash
make build
```

### Uploading
```bash
make upload
```

### Monitoring
```bash
make monitor
```

### Other Commands
- Clean build artifacts: `make clean`
- Activate venv shell: `make shell`
- Show all commands: `make help` or `make`

## Project Structure

```
smart_home_firmware/
├── platformio.ini      # PlatformIO configuration
├── Makefile            # Build commands and setup
├── src/
│   └── main.c         # Main application code
├── include/            # Header files
├── lib/                # Custom libraries
├── venv/               # Python virtual environment (created by make setup)
└── README.md          # This file
```

## Configuration

Edit `platformio.ini` to customize:
- Board type (currently `esp32dev`)
- Build flags
- Upload speed
- Monitor settings
- Library dependencies

## ESP-IDF Version

This project uses the ESP-IDF framework. PlatformIO will automatically download the appropriate ESP-IDF version based on the platform version specified.

## Troubleshooting

- **Upload fails**: Check USB connection and drivers. Try pressing BOOT button during upload.
- **Monitor shows garbage**: Adjust `monitor_speed` in `platformio.ini` (common values: 115200, 9600)
- **Build errors**: Run `make setup` first to create the virtual environment and install PlatformIO
- **Command not found**: Make sure you've run `make setup` before using other make commands

## License

This project is licensed under the MIT License – see the [LICENSE](https://github.com/kibshh/smart-home-firmware/blob/main/LICENSE) file for details.

