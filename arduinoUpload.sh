#!/bin/bash

configPath="cfg/$1.h"
portPath="/dev/ttyACM$2"

echo "compiling..."
echo "using '$configPath'"
echo "to '$portPath'"

arduino-cli compile -b arduino:mbed_opta:opta main --build-property "build.extra_flags=\"-DCONFIG_PATH=\"$configPath\"\""

echo "Done\nuploading..."
arduino-cli upload main --fqbn arduino:mbed_opta:opta --port /dev/tty$2
