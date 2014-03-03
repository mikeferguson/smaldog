#Intel NUC Specs

 * Intel NUC D34010WYK
 * Crucial CT120M500SSD3 120GB mSATA
 * Crucial Ballistix BLS8G3N169ES4 8GB DDR3 1600 9-9-9-24 @ 1.35V XMP Dual profile
 * Intel Centrino 6235N Wireless

#Setup
 * Updated BIOS (currently v0023, was initially 0018)
 * Installed 12.04 server (be sure to have EFI partition). Disabled fast boot in BIOS.
 * Had to setup wlan0 after install (failed to find networks during install, wpa-psk didn't seem to work...)
 * Update to 3.8 kernel: sudo apt-get update; sudo apt-get install linux-image-3.8.0.....
 * Ethernet now works
 * sudo apt-get upgrade to update remaining packages
 * sudo apt-get install git build-essential libeigen3-dev
 * sudo apt-get install ros-hydro-ros-base ros-hydro-common-msgs ros-hydro-hokuyo-node ros-hydro-tf
