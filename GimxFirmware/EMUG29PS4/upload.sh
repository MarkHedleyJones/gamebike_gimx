#!/bin/bash
make
sudo dfu-programmer atmega32u4 erase
sudo dfu-programmer atmega32u4 flash emu.hex
sudo dfu-programmer atmega32u4 reset
