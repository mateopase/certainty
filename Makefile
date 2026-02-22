FQBN ?= rp2040:rp2040:seeed_xiao_rp2040
OUT_DIR ?= build
PORT ?=
BAUD ?= 115200

.PHONY: all build clean monitor

all: build

build:
	mkdir -p $(OUT_DIR)
	arduino-cli compile --fqbn "$(FQBN)" --output-dir "$(OUT_DIR)" .

clean:
	rm -rf "$(OUT_DIR)"

monitor:
	@if [ -z "$(PORT)" ]; then \
		echo "Usage: make monitor PORT=/dev/cu.usbmodemXXXX [BAUD=115200]"; \
		exit 1; \
	fi
	@echo "Opening serial monitor on $(PORT) @ $(BAUD)"
	@echo "Exit screen with: Ctrl-A then K then y"
	screen "$(PORT)" "$(BAUD)"
