# Plainsong-Metronome
Open-source STM32-based digital stereo metronome
## Details
The Plainsong Metronome is a STM32F401-based audio platform, intended to be developed as a hi-fidelity digital stereo metronome for musicians.
The PCB contains two encoders intended to be used for UI navigation and volume control.
Four mechanical keyboard switches are included that are intended to control UI item selection and playback settings.
The PCB can be powered by a 12VDC 1A supply via, the DC barrel jack. 
Two 2.54mm pin header connectors are included for speaker outputs (L and R), as well as an extra 3-pin 2.54mm header (intended to be connected to a pane-mounted 3.4mm stereo TRS jack output for headphones).
The board houses an FFC connector to allow ribbon cable connection to a MIDAS OLED display.
A Micro B USB jack is included to allow firmware upgrade via. STM32 DFU, and potentially sample loading.
A mechanical enclosure design is available for the device (see 'Enclosure').
## Project Status
The project is currently in its B rev - board bring-up has been done in Rev A and Rev B has all changes implemented. 
I started this project to learn more about more complex 4-layer mixed-signal PCB design, so the full firmware was not my primary concern. I have therefore decided to make this project open-source, as I have learnt what I wanted to learn from it and if anyone else would like to dive deeper with the firmware, they can potentially also get something out of this project or take it further.
## Hardware Summary
### STM32F401CBU7 MCU
The MCU processes all IO and uses I2C, I2S and SPI to interface with the OLED, CODEC and Flash memory, respectively. The flash memory is included to expand audio sample storage capacity.
### MAX9867 Audio CODEC
The CODEC handles DAC duties for the system - samples are transmitted to it from the MCU via. I2S, then output to the audio amplifiers. Volume control is handled by the CODEC (with commands coming from the MCU).
### MAX9716 Audio Amplifiers
Two audio amps give differential audio outputs for the speakers (delivering 1.4W into 4 Ohms).
### MIDAS SSD1309ZC-Based OLED
The FFC connector on the board allows for connection of a 31-pin MIDAS OLED (MCOT128064B1V used during testing).
## Current Status
### Rev A
Board bring up was performed during Rev A. I have one Rev A board with all mods implemented that are required for the hardware to work - these changes have been implemented into the Rev B design. I have a number of Rev A boards (unmodded, unpopulated) - feel free to contact me if you would like one of these. I am happy to ship these to the UK if you would like to populate and mod them.
### Rev B
Rev B contains all of the mods from Rev A bring-up (see schematic for more details). The one untested part of the hardware that I have changed between rev's is the audio amplifier circuit. Rev B has a simplified circuit here, as I did not realise that the volume control could be done solely by the CODEC, so the digi-pots are no longer required.
## Untested Components
New Rev B audio amplifier circuit.
## Test Firmware Included
I have included the main test firmware file for STM32 Cube IDE. This includes simple drivers for some of the functions of each of the peripherals on the board that required testing during bring-up. The basic RTOS operation is also tested here. You will find the different test functions to be commented in/out.
## Firmware Build and Upload Process
The easiest way is using STM32 Cube IDE and USB DFU. Hold down the upload button and the SELECT button to access the DFU bootloader whilst power-cycling.
## Future Feature Wishlist
Feel free to help and contribute with these features:
- Audio amplifier circuit testing
- Full operational code
- Sample uploading option added
- External software for sample uploading
## Licence
This project is released under the CERN OHL V2 - Strongly Reciprocal Licence - see 'LICENCE.txt' for full terms.
