# https://github.com/arduino/arduino-cli/releases

port := $(shell python3 board_detect.py)

default:
	arduino-cli compile --fqbn=esp8266:esp8266:d1_mini_clone code

upload:
	@# echo $(port)
	arduino-cli compile --fqbn=esp8266:esp8266:d1_mini_clone code
	arduino-cli -v upload -p "${port}" --fqbn=esp8266:esp8266:d1_mini_clone code

install_platform:
	arduino-cli config init
	arduino-cli core update-index
	arduino-cli core install esp8266:esp8266

deps:
	arduino-cli lib install "Etherkit JTEncode"
	arduino-cli lib install "Etherkit Si5351"
	arduino-cli lib install "RTClib"
	arduino-cli lib install "Time"
	arduino-cli lib install "NTPClient"
	arduino-cli lib install "uEEPROMLib"
	arduino-cli lib install "PU2CLR SI4735"
	# arduino-cli lib install "ESPAsyncWebServer"
	# arduino-cli lib install "ESPAsyncTCP"


install_arduino_cli:
	curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | BINDIR=~/.local/bin sh
