This guide walks you through setting up arduino-cli on a Linux System. This is tested and compiled on a Jetson Orin AGX, Ubuntu 22.04 LTS, ROS2 Humble, targeting Arduino CLI Setup for ESP32 or ESP32-S related boards. Inlcuding the installation of CLI, cores, using FQBNS, enabling DFU uploads, compiling and uploading to board. 

We first start with installing the Arduino CLI:
	sudo apt update
	cd ~/Downloads
	curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
	sudo mv bin/arduino-cli /usr/local/bin/
	arduino-cli version

Call the Arduino-CLI to run in tab (or paste in bash)
	export PATH=$PATH:/usr/bin

Initiliaze Configuration - this creates a config file where you can define where your sketches, libraries and downloads are stored. The yaml file can be found (~/.arduino15/arduino-cli.yaml):
	arduino-cli config init

Now we want to install the board support packages (cores)
Espressif ESP32:
	arduino-cli config set board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
	arduino-cli core update-index
	arduino-cli core install esp32:esp32

Official Arduino ESP32 or ESPs3 boards
	arduino-cli config add board_manager.additional_urls https://downloads.arduino.cc/cores/esp32/package_esp32_index.json
	arduino-cli core update-index
	arduino-cli core install arduino:esp32

Install libraries:
	arduino-cli lib install micro_ros_arduino 
	OR go to ~/Arduino/libraries/...
	git clone -b humble https://github.com/micro-ROS/micro_ros_arduino.git
	git clone https://github.com/madhephaestus/ESP32Servo.git

To check the available installed boards:
	arduino-cli board listall | grep -i nano

Understanding FQBN = Fully Qualified Board Name
<vendor>:<platform>:<board>
	espressif:esp32:nano_nora
	arduino:esp32:nano_nora

Create a Sketch
	arduino-cli sketch new DiffDriveBoat
	cd ~/Arduino/DiffDriveBoat/
	nano DiffDriveBoat.ino

Compile & Flash
	arduino-cli compile --fqbn arduino:esp32:nano_nora ~/Arduino/Blink
	arduino-cli upload -p /dev/ttyACM1 --fqbn arduino:esp32:nano_nora ~/Arduino/Blink

Monitor & verbose commands
	arduino-cli monitor -p /dev/ttyACM1
	arduino-cli board details --fqbn arduino:esp32:nano_nora

*** Errors & Solutions ***
Open New tab if this error comes up:
	bash: /home/aimm/bin/arduino-cli: No such file or directory 


When you compile it may say that the dfu device is not found, follow these directions:
	Check detalied info by device:
	ls -l /dev/serial/by-id/  
 	
 	Install dfu-util for serial support 
 	sudo apt update && sudo apt install dfu-util
	
	Check dfu-util version
	dfu-util --version
	
	Still can't see the dfu, create a udev rule
	sudo nano /etc/udev/rules.d/99-arduino-nano-esp32.rules
		
		Arduino Nano ESP32 DFU mode
		SUBSYSTEM=="usb", ATTR{idVendor}=="2341", ATTR{idProduct}=="0070", MODE="0666", GROUP="dialout"

	sudo udevadm control --reload-rules
	sudo udevadm trigger
	sudo usermod -a -G dialout $USER

When you compile for the arduino esp32 and says something about not being able to find the folder "Precompiled library in "/home/aimm/Arduino/libraries/micro_ros_arduino/src/esp32s3" not found", create a new folder and copy contents:
	mkdir -p ~/Arduino/libraries/micro_ros_arduino/src/esp32s3
	cp ~/Arduino/libraries/micro_ros_arduino/src/esp32/libmicroros.a ~/Arduino/libraries/micro_ros_arduino/src/esp32s3/


Sources:
Although I didn't understand a single word this guy was saying, he clearly showed each step on a x64 bit architecture:
https://www.youtube.com/watch?v=ggxNLKZqITU