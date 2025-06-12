#!/usr/bin/env bash
set -e

# Install system packages
apt-get update
apt-get install -y curl python3-pip

# Install Arduino CLI if not present
if ! command -v arduino-cli >/dev/null 2>&1; then
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
    mv -f bin/arduino-cli /usr/local/bin/
    rm -rf bin
fi

# Install Teensy board definitions
arduino-cli core update-index
arduino-cli core install teensy:avr

# Install Python requirements
pip3 install -r requirements.txt
