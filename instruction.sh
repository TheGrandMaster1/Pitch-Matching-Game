#!/bin/bash

# Instruction display script for Pitch Matching Game
# Displays game instructions using cowsay in a loop

while true; do
    clear
    cat instruction.txt | cowsay -n
    sleep 1000000
done