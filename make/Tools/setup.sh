#!/usr/bin/env bash

# If abort error, exit shell script.
set -e

# Set the gcc-arm tool chains
if command -v arm-none-eabi-gcc > /dev/null 2>&1; then
    echo "The gcc-arm tool chains has been installed!"
else
    echo "The gcc-arm tool chains has not been installed!"
    echo "Installing the gcc-arm tool chains..."
    sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
    sudo apt-get update
    sudo apt-get install gcc-arm-embedded
    echo "Installing the gcc-arm tool chains successfully!"
fi

# Set the openocd
if command -v openocd > /dev/null 2>&1; then
    echo "The openocd has been installed!"
else
    echo "The openocd has not been installed!"
    echo "Installing the openocd..."
    sudo apt-get install openocd
    echo "Installing the openocd successfully!"
fi
