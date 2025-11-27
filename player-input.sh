#!/bin/bash

SERIAL_PORT="${1:-/dev/ttyACM0}"

# Check if serial port exists
if [ ! -e "$SERIAL_PORT" ]; then
    echo "Error: Serial port $SERIAL_PORT not found"
    exit 1
fi

# Configure serial port (115200 baud, raw mode)
stty -F "$SERIAL_PORT" 115200 raw -echo 2>/dev/null || {
    echo "Warning: Could not configure serial port settings"
    echo "Continuing anyway..."
}

# Main input loop
while true; do
    clear
    echo -e "\t\t╔══════════════════════════════════════════════════════╗"
    echo -e "\t\t║                 PITCH MATCHING GAME                  ║"
    echo -e "\t\t╚══════════════════════════════════════════════════════╝"
    echo -n -e "\t\t\t\t  🎤 Input: "
    read -r input

    # Check if input is not empty
    if [ -n "$input" ]; then
        # Send command to serial port with carriage return and newline
        echo "$input" >> "$SERIAL_PORT"
    fi
done
