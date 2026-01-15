
# 6417 Fridocumentation

### New Year New Software Updates

#### FRC tools
	Visit https://www.ni.com/de/support/downloads/drivers/download.frc-game-tools.html#581857
	Download the Tools:
		a) A Mentor helps you with the login
		b) A Mentor gives you the Installationfile
	Run The Installer

#### Flash RoboRio with new Software  
	Take the SD card from the Roborio. (Helpfull tool: Pencil)
	Insert the SD card into your PC.
	Open the RoboRio Imaging Tool
	Klick the SD-Symbol
	Copy the Zip file to another Folder
	Download Balena Etcher: https://etcher.balena.io/#download-etcher
	Open Balena Etcher:
	1: Select the Zip file
	2: Select the SD-Card
	3: klick "Flash"
	Once Finished, insert the SD Card back into the RoboRio (Helpfull tool: Pencil)  

#### Update Radio
	Visit https://frc-radio.vivid-hosting.net/overview/firmware-releases
	Download the newest version.
	Copy the SHA-256 Code to your Clipboard
	Unpack the zip file
	Disconnect the Radio from the RoboRio
	Create a Connection to the RoboRio:
	Wired: 
	Insert an ethernet cable into the DS-Port.
	Connect the other end to your PC.
	Wireless (do these Steps even if you want a Wired Connection):
	Open the WLAN selection.
	Connect to the Network with a Name starting with FRC-
	In your Browser, enter 10.64.17.1 If it doesnt work try theese aswell: 10.64.17.4 or 10.0.1.1 or 10.0.1.4
	Now you should see a mainly white Page. In the left top Corner: VIVID HOSTING In the right Top Corner: VH-109
	Click all three checkboxes.
	In the textfield "Teamnumber" enter 6417.
	In the textfield "SSID Suffix" you can enter the name of your robot.
	The two Fields about "WPA" you can choose a password for your network. (e.g. Fridolin2026)
	Click "Configure"
	Scroll to the bottom of the page
	Click "Upload file" and select your previously downloaded Firmware.
	Paste the SHA-256 Key from your Clipboard
	Click "Upload Firmware"
	Wait until the SYS LED is slowly blinking.
	Disconnect your PC from the Radio
	Reconnect the RoboRio to the Radio(RIO Port)
	Restart the Robot
	Without Ethernet Cable, you should see your Robot in the Networks list. (FRC-6417-RoboName)
	Connect to this network and enter your previously set Password.
	Open the Driver station.
	The "communication" lamp should be green now.
	 