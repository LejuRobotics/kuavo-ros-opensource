#!/bin/bash
pactl list cards short | awk '/usb-/ {print $2}' | while read -r card; do
    echo "Checking: $card"
    if pactl list cards | grep -A 50 "Name: $card" | grep -q "output:analog-stereo"; then
        echo "  → Detected as speaker, setting to output-only"
        pactl set-card-profile "$card" output:analog-stereo 2>/dev/null || \
        pactl set-card-profile "$card" off 2>/dev/null
    else
        echo "  → Not a speaker (likely mic), skipping"
    fi
done