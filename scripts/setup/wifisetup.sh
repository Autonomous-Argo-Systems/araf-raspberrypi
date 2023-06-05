#!/bin/bash

# Install hotspot dependencies
echo "Setting up enviroment..."
sudo apt install hostapd

echo "Cloning deps..."
git clone "https://github.com/oblique/create_ap"

# Installen create_ap
cd create_ap-master
sudo make install
cd ..

# Setting wifi settings
echo "Settings settings"
sudo echo "CHANNEL=default
GATEWAY=10.0.0.1
WPA_VERSION=2
ETC_HOSTS=0
DHCP_DNS=gateway
NO_DNS=0
NO_DNSMASQ=0
HIDDEN=0
MAC_FILTER=0
MAC_FILTER_ACCEPT=/etc/hostapd/hostapd.accept
ISOLATE_CLIENTS=0
SHARE_METHOD=nat
IEEE80211N=0
IEEE80211AC=0
HT_CAPAB=[HT40+]
VHT_CAPAB=
DRIVER=nl80211
NO_VIRT=0
COUNTRY=
FREQ_BAND=2.4
NEW_MACADDR=
DAEMONIZE=0
NO_HAVEGED=0
WIFI_IFACE=wlan1
INTERNET_IFACE=wlan0
SSID=ARAF_Wifi
PASSPHRASE=arafdegrootWIFI!
USE_PSK=0" > '/etc/create_ap.conf'
