.PHONY: setup build upload monitor clean shell help

# Default target
help:
	@echo "Smart Home Firmware - Makefile Commands"
	@echo ""
	@echo "  make setup     - Set up Python venv and install PlatformIO"
	@echo "  make build     - Build the firmware"
	@echo "  make upload    - Build and upload to ESP32"
	@echo "  make monitor   - Monitor serial output"
	@echo "  make clean     - Clean build artifacts"
	@echo "  make shell     - Activate venv shell"
	@echo "  make help      - Show this help message"

setup:
	@if [ ! -d "venv" ]; then \
		echo "Creating Python virtual environment..."; \
		python3 -m venv venv; \
	fi
	@echo "Installing dependencies..."
	@./venv/bin/pip install --upgrade pip
	@./venv/bin/pip install platformio
	@./venv/bin/pio platform install espressif32
	@echo "Setup complete! Run 'source venv/bin/activate' to activate the venv."

build:
	@if [ ! -d "venv" ]; then \
		echo "Error: Virtual environment not found. Run 'make setup' first."; \
		exit 1; \
	fi
	@./venv/bin/pio run

upload:
	@if [ ! -d "venv" ]; then \
		echo "Error: Virtual environment not found. Run 'make setup' first."; \
		exit 1; \
	fi
	@./venv/bin/pio run --target upload

monitor:
	@if [ ! -d "venv" ]; then \
		echo "Error: Virtual environment not found. Run 'make setup' first."; \
		exit 1; \
	fi
	@./venv/bin/pio device monitor

clean:
	@if [ ! -d "venv" ]; then \
		echo "Error: Virtual environment not found. Run 'make setup' first."; \
		exit 1; \
	fi
	@./venv/bin/pio run --target clean

shell:
	@if [ ! -d "venv" ]; then \
		echo "Error: Virtual environment not found. Run 'make setup' first."; \
		exit 1; \
	fi
	@echo "Activating virtual environment..."
	@exec bash -c "source venv/bin/activate && exec bash"

