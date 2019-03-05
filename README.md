# RF 433MHz Analyser

Instrument to monitor RF 433MHz signal strength and simplify data capture

Uses a RX6 433MHz receiver (modified)

Construction details at https://www.instructables.com/id/RF433MHzAnalyser

## Features
- Monitors signal strength of 433MHz transmissions
- Allows range of transmissions to be assessed
- Data capture triggered by RSSI level allowing clean data transitions to be captured without background noise
- Local display of RSSI
- Captures RSSI data to files
- Captures data transitions to files
- Web server to allow control from browser, viewing, downloading and deleting capture files
- Battery powered (rechargeable LIPO) with charger point
- Very low quiescent current (< 40uA) for long battery life
- Single button control to power on, trigger rssi / data captures, power off
- Auto turns off if quiescent for a period
- Status and configuration data also available from web interface​
- Software can be updated via web interface
- Initial AP to set wifi access details when first configured or network changes.

## Usage
- Press button to turn on, RSSI display comes on and updates continously
- Web Status screen shows current measurement and other details
- Short press of button to capture RSSI measurements to a file
	- Interval and duration can be configured
	- can also be terminated early with another short press
- Medium length press of button to capture data transitions to a file.
	- Triggered by RSSI going above a configured level (when a 433MHz signal received.
	- Transitions of 0 / 1 pulses measured in microseconds.
	- Terminated when configurable transition count reached. 
- Long press of button to turn instrument off
- Web Configuration screen displays current configuration and allows values to be changed
	
## Web interface
The firmware supports the following http calls
- /edit - access filing system of device; may be used to download measures Files
- /status - return a string containing status details
- /loadconfig -return a string containing config details
- /saveconfig - send and save a string to update config
- /loadcapture - return a string containing measures from a files
- /setmeasureindex - change the index to be used for next measure
- /getcapturefiles - get a string with list of available measure files
- /capture - trigger capture of RSSI or data
- /firmware - initiate update of firmware




