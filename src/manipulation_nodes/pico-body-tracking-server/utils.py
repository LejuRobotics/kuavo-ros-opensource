#!/usr/bin/env python3
import os
import netifaces
import numpy as np

def get_wifi_ip():
    try:
        # Get all network interfaces
        interfaces = netifaces.interfaces()

        # Find the WiFi interface (usually starts with 'wl')
        wifi_interface = next(
            (iface for iface in interfaces if iface.startswith("wl")), None
        )

        if wifi_interface:
            # Get the IPv4 address of the WiFi interface
            addresses = netifaces.ifaddresses(wifi_interface)
            if netifaces.AF_INET in addresses:
                return addresses[netifaces.AF_INET][0]["addr"]

        return "WiFi not connected"
    except Exception as e:
        return f"Error: {e}"



import logging
import subprocess

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

def get_wifi():
    try:
        logger.info("Attempting to get WiFi SSID using nmcli")
        # Run the nmcli command to get the SSID
        result = subprocess.run(["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"], 
                                capture_output=True, text=True, check=True)
        # Parse the output to find the active connection
        for line in result.stdout.strip().split('\n'):
            active, ssid = line.split(':')
            if active == 'yes':
                logger.info(f"Connected to WiFi: {ssid}")
                return ssid
        
        logger.warning("No active WiFi connection found")
        return "Not connected to WiFi"
    except subprocess.CalledProcessError as e:
        logger.error(f"Error running nmcli command: {e}")
        return f"Error: {e}"
    except Exception as e:
        logger.error(f"Unexpected error getting WiFi SSID: {e}")
        return f"Error: {e}"

def get_mac_address():
    try:
        # Get all network interfaces
        interfaces = netifaces.interfaces()

        # Find the WiFi interface (usually starts with 'wl')
        wifi_interface = next(
            (iface for iface in interfaces if iface.startswith("wl")), None
        )

        if wifi_interface:
            # Get the MAC address of the WiFi interface
            addresses = netifaces.ifaddresses(wifi_interface)
            if netifaces.AF_LINK in addresses:
                return addresses[netifaces.AF_LINK][0]['addr']

        return "WiFi interface not found"
    except Exception as e:
        return f"Error: {e}"