PROGRAMMER = "/dev/cu.wchusbserial1420"
CC=avr-gcc
CPPFLAGS=-IlibATmega328 -I../ros_workspace/src/hmmwv/src/ -Wall -Os -fpack-struct -fshort-enums -ffunction-sections -fdata-sections -funsigned-char -funsigned-bitfields -mmcu=atmega328p -DF_CPU=16000000UL -MMD -MP

default: clean build upload

.PHONY: build
build:
	$(CC) $(CPPFLAGS) -MF"main.d" -MT"main.o" -c -o "main.o" "main.cpp"
	$(CC) $(CPPFLAGS) -MF"i2c.d" -MT"i2c.o" -c -o "i2c.o" "libATmega328/i2c.cpp"
	$(CC) $(CPPFLAGS) -MF"ssd1306.d" -MT"ssd1306.o" -c -o "ssd1306.o" "libATmega328/ssd1306.cpp"
	$(CC) -Wl,-Map,enginecontrol.map -mmcu=atmega328p -o "enginecontrol.elf" ./main.o ./i2c.o ./ssd1306.o
	avr-objdump -h -S enginecontrol.elf  >"enginecontrol.lss"
	avr-objcopy -R .eeprom -R .fuse -R .lock -R .signature -O ihex enginecontrol.elf "enginecontrol.hex"
	avr-objcopy -j .eeprom --no-change-warnings --change-section-lma .eeprom=0 -O ihex enginecontrol.elf  "enginecontrol.eep"
	avr-size --format=avr --mcu=atmega328p enginecontrol.elf

.PHONY: upload
upload:
	avrdude -pm328p -carduino -P${PROGRAMMER} -b57600 -Uflash:w:enginecontrol.hex:a

.PHONY: clean
clean:
	rm ssd1306.* main.o main.d || true

