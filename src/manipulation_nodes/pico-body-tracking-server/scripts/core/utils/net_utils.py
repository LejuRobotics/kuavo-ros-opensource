import netifaces
import subprocess

def get_wifi_ip():
    """Get WiFi IP address."""
    try:
        interfaces = netifaces.interfaces()
        wifi_interface = next(
            (iface for iface in interfaces if iface.startswith("wl")), None
        )

        if wifi_interface:
            addresses = netifaces.ifaddresses(wifi_interface)
            if netifaces.AF_INET in addresses:
                return addresses[netifaces.AF_INET][0]["addr"]

        return None
    except Exception as e:
        return f"Error: {e}"
    
def get_wifi():
    """Get WiFi SSID."""
    try:
        # SDKLogger.info("Attempting to get WiFi SSID using nmcli")
        result = subprocess.run(["nmcli", "-t", "-f", "active,ssid", "dev", "wifi"], 
                                capture_output=True, text=True, check=True)
        for line in result.stdout.strip().split('\n'):
            active, ssid = line.split(':')
            if active == 'yes':
                print(f"Connected to WiFi: {ssid}")
                return ssid
        
        print("\033[93mNo active WiFi connection found\033[0m")
        return "Not connected to WiFi"
    except Exception as e:
        print(f"\033[91mError getting WiFi SSID: {e}\033[0m")
        return f"Error: {e}"
    
def get_mac_address():
    """Get MAC address."""
    try:
        interfaces = netifaces.interfaces()
        wifi_interface = next(
            (iface for iface in interfaces if iface.startswith("wl")), None
        )

        if wifi_interface:
            addresses = netifaces.ifaddresses(wifi_interface)
            if netifaces.AF_LINK in addresses:
                return addresses[netifaces.AF_LINK][0]['addr']

        return "WiFi interface not found"
    except Exception as e:
        return f"Error: {e}"
    
def get_localip_and_broadcast_ips():
    """
    Gets a list of tuples containing local IPv4 addresses and their corresponding broadcast IP addresses 
    for both wired and wireless interfaces.
    Returns: List of tuples [(local_ip, broadcast_ip), ...]
    Requires the 'netifaces' library to be imported.
    """
    ip_pairs = []
    excluded_prefixes = ("docker", "br-", "veth", "lo")  # Exclude virtual and loopback interfaces
    wired_prefixes = ("eth", "en", "em")  # Common wired ethernet interface prefixes
    wireless_prefixes = ("wl", "wlan")  # Common wireless interface prefixes
    try:
        for iface_name in netifaces.interfaces():
            # Skip excluded interfaces
            if any(iface_name.startswith(prefix) for prefix in excluded_prefixes):
                continue
            # Include both wired and wireless interfaces
            if not any(iface_name.startswith(prefix) for prefix in wired_prefixes + wireless_prefixes):
                continue
            if_addresses = netifaces.ifaddresses(iface_name)
            if netifaces.AF_INET in if_addresses:
                for link_addr in if_addresses[netifaces.AF_INET]:
                    local_ip = link_addr.get('addr')
                    broadcast_ip = link_addr.get('broadcast')
                    
                    # Ensure both local IP and broadcast IP exist and are valid
                    if local_ip and broadcast_ip:
                        # Skip local/loopback addresses like 127.x.x.x
                        if not local_ip.startswith('127.') and not broadcast_ip.startswith('127.'):
                            ip_pairs.append((local_ip, broadcast_ip))
        
        # Return unique IP pairs, sorted for consistency
        return sorted(list(set(ip_pairs)))
    except Exception as e: # Catch any error during netifaces operations
        print(f"\033[91mError getting local and broadcast IPs using 'netifaces': {e}. Ensure 'netifaces' is installed and network interfaces are configured correctly.\033[0m")
        return []


