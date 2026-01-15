# 6417 Fridocumentation

## Table of Contents
- [New Year, New Software Updates](#new-year-new-software-updates)
  - [FRC Game Tools](#frc-game-tools)
  - [Flash roboRIO with new software](#flash-roborio-with-new-software)
  - [Update radio](#update-radio)
  - [Update device firmware](#update-device-firmware)
    - [All REV devices](#all-rev-devices)

### New Year, New Software Updates

#### FRC Game Tools
- [ ] Visit https://www.ni.com/de/support/downloads/drivers/download.frc-game-tools.html#581857
- [ ] Download the tools:
  - [ ] A mentor helps you with the login
  - [ ] A mentor provides the installation file
- [ ] Run the installer

#### Flash roboRIO with new software
- [ ] Take the SD card from the roboRIO (helpful tool: pencil)
- [ ] Insert the SD card into your PC
- [ ] Open the roboRIO Imaging Tool
- [ ] Click the SD symbol
- [ ] Copy the ZIP file to another folder
- [ ] Download Balena Etcher: https://etcher.balena.io/#download-etcher
- [ ] Open Balena Etcher:
  - [ ] Select the ZIP file
  - [ ] Select the SD card
  - [ ] Click "Flash"
- [ ] Once finished, insert the SD card back into the roboRIO (helpful tool: pencil)

#### Update radio
- [ ] Visit https://frc-radio.vivid-hosting.net/overview/firmware-releases
- [ ] Download the newest version
- [ ] Copy the SHA-256 checksum to your clipboard
- [ ] Unpack the ZIP file
- [ ] Disconnect the radio from the roboRIO
- [ ] Create a connection to the roboRIO:
  - [ ] Wired:
    - [ ] Insert an Ethernet cable into the DS port
    - [ ] Connect the other end to your PC
  - [ ] Wireless (do these steps even if you want a wired connection):
    - [ ] Open the Wi-Fi selection
    - [ ] Connect to the network with a name starting with FRC-
    - [ ] In your browser, enter 10.64.17.1. If it does not work, try 10.64.17.4, 10.0.1.1, or 10.0.1.4
- [ ] You should see a mostly white page. In the top-left corner: VIVID HOSTING. In the top-right corner: VH-109
- [ ] Click all three checkboxes
- [ ] In the "Team number" field, enter 6417
- [ ] In the "SSID suffix" field, enter the name of your robot
- [ ] In the two "WPA" fields, choose a password for your network (e.g., Fridolin2026)
- [ ] Click "Configure"
- [ ] Scroll to the bottom of the page
- [ ] Click "Choose file" and select your previously downloaded firmware
- [ ] Paste the SHA-256 checksum from your clipboard
- [ ] Click "Upload"
- [ ] Wait until the SYS LED is slowly blinking
- [ ] Disconnect your PC from the radio
- [ ] Reconnect the roboRIO to the radio (RIO port)
- [ ] Restart the robot
- [ ] Without an Ethernet cable, you should see your robot in the networks list (FRC-6417-RoboName)
- [ ] Connect to this network and enter your previously set password
- [ ] Open the Driver Station
- [ ] The "communication" lamp should be green now

Alternative tutorial: https://frc-radio.vivid-hosting.net/overview/upgrading-firmware

Useful IP addresses while connected to your robot:
- 10.64.17.1
- 10.64.17.2
- 10.64.17.4
- 10.64.17.200

#### Update device firmware

##### All REV devices
- [ ] Connect your PC to the Power Distribution Board (USB-C port)
- [ ] Open "REV Hardware Client 2"
