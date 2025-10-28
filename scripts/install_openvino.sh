#!/bin/bash
wget -O GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
sudo gpg --yes --output /etc/apt/trusted.gpg.d/intel.gpg --dearmor GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB


sudo apt-get install gnupg -y
echo "deb https://apt.repos.intel.com/openvino ubuntu20 main" | sudo tee /etc/apt/sources.list.d/intel-openvino.list

# Update package lists with error handling
echo "Updating package lists..."
if ! sudo apt update --fix-missing; then
    echo "ERROR: Failed to update package lists. Trying alternative approach..."
    sudo apt-get update --fix-missing || {
        echo "ERROR: Unable to update package lists. Please check your internet connection and try again."
        exit 1
    }
fi

# Install OpenVINO
echo "Installing OpenVINO..."
sudo apt install openvino -y

# Clean up downloaded key
rm -f GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
