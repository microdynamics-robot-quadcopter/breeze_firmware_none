#!/usr/bin/env bash

# If abort error, exit shell script.
set -e

# Install the gcc-arm tool chains
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

# Install the openocd
if command -v openocd > /dev/null 2>&1; then
    echo "The openocd has been installed!"
else
    echo "The openocd has not been installed!"
    echo "Installing the openocd..."
    sudo apt-get install openocd
    echo "Installing the openocd successfully!"
fi

# Install the minicom
if command -v minicom > /dev/null 2>&1; then
    echo "The minicom has been installed!"
else
    echo "The minicom has not been installed!"
    echo "Installing the minicom..."
    sudo apt-get install minicom
    echo "Installing the minicom successfully!"
fi
