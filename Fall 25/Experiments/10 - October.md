
### October 16th, 2025

**Objective:** Set ESP32 as beacon

**Experiment:** Download [Arduino IDE](https://support.arduino.cc/hc/en-us/articles/360019833020-Download-and-install-Arduino-IDE)
Open Arduino IDE
- Preference (Setting)
	- Additional Boards Manager URLs add `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
- Tools -> Board -> Boards Manager
	- Install `esp32 by Espressif Systems`
	- Note: Please select version 2.0.14
- Script
```
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEBeacon.h>
#include <BLEAdvertising.h>

#define BEACON_UUID "12345678-1234-1234-1234-123456789abc"
#define BEACON_MAJOR 1
#define BEACON_MINOR 1
#define BEACON_TX_POWER -59 // RSSI @1m

void setBeacon() {
	BLEBeacon oBeacon;
	oBeacon.setManufacturerId(0x004C); // Apple company ID
	oBeacon.setProximityUUID(BLEUUID(BEACON_UUID));
	oBeacon.setMajor(BEACON_MAJOR);
	oBeacon.setMinor(BEACON_MINOR);
	oBeacon.setSignalPower(BEACON_TX_POWER);
	
	BLEAdvertisementData advertisementData;
	BLEAdvertisementData scanResponseData;
	
	advertisementData.setFlags(0x04); // BR_EDR_NOT_SUPPORTED
	advertisementData.setManufacturerData(oBeacon.getData());
	
	BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
	pAdvertising->setAdvertisementData(advertisementData);
	pAdvertising->setScanResponseData(scanResponseData);

	BLEDevice::startAdvertising();
	Serial.println("Beacon started...");
}

void setup() {
	Serial.begin(115200);
	Serial.println("Initializing BLE Beacon...");
	BLEDevice::init("TurtleBot_Station");
	BLEDevice::setPower(ESP_PWR_LVL_P9); // Max TX power
	setBeacon();
}

void loop() {
	delay(2000);
	Serial.println("Beacon advertising...");
}
```
- Select Board
	- Select other board and port...
	- Left (Board): Select your board
	- Right (Port): If you don't know unplug and replug the board in
- Verify
	- To check the code
- Upload
	- To upload your code to ESP32

**Result:** Working
In the Serial Monitor
- Beacon advertising...

### October 20th, 2025

**Objective:** Create python script to make it can detect the ESP32 (beacon)

**Experiment:**
- Script
```
import asyncio
from bleak import BleakScanner, BleakError
import math
import sys
  
TX_POWER = -59

def estimate_distance(rssi, tx_power=TX_POWER, n=2.0):
	"""
	
	Estimate distance (in meters) using the log-distance path loss model.
	n = environmental factor: 2=open space, 2.7–3.5=indoors
	"""
	if rssi == 0:
		return None
	ratio = (tx_power - rssi) / (10 * n)
	return round(pow(10, ratio), 2)

async def scan_loop():
	print("Starting BLE beacon")
	try:
		while True:
			devices = await BleakScanner.discover(timeout=2.5)
			found = False
			for d in devices:
				mdata = d.metadata.get("manufacturer_data", {})
				# iBeacon (Apple) Manufacturer ID = 0x004C
				if 0x004C in mdata:
					found = True
					dist = estimate_distance(d.rssi)
					print(f"iBeacon Detected | MAC: {d.address} | RSSI: {d.rssi:4d} dBm | Dist: {dist or '-'} m")
			if not found:
				print("(Beacon not detected)")
			await asyncio.sleep(1)
	except BleakError as e:
		print(f"BLE error: {e}")
		sys.exit(1)
	except KeyboardInterrupt:
		print("\nStopped by user.")
		sys.exit(0)
  
if __name__ == "__main__":
	asyncio.run(scan_loop())
```

**Result:** Detected (On pink computer)
### October 26th, 2025

**Objective:** Make Raspberry Pi can detect the ESP32 (beacon)

**Issue:**
- When run the python scrip
	- `BLE error: No Bluetooth adapters found.`
	- Because while the Bluetooth kernel modules are loaded (Bluetooth: Core ver 2.22), the firmware for the onboard Bluetooth chip isn’t loaded

**Solution:**
```
sudo apt update
sudo apt install pi-bluetooth bluez -y
sudo systemctl enable bluetooth
sudo systemctl enable hciuart
sudo reboot
```

**Result:** Detected

**Objective:** Make TurtleBot autonomous navigate to beacon

**Experiment:**
- Create python script `beacon_navigation.py`
```
#!/usr/bin/env python3
import rospy
import asyncio
import math
import tf
from geometry_msgs.msg import PoseStamped
from bleak import BleakScanner

TX_POWER = -59 # same as in your beacon code
ENV_FACTOR = 2.5 # indoor attenuation factor
  
def estimate_distance(rssi, tx_power=TX_POWER, n=ENV_FACTOR):
	if rssi == 0:
		return None
	ratio = (tx_power - rssi) / (10 * n)
	return round(pow(10, ratio), 2)

async def beacon_tracker():
	pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
	rospy.init_node("beacon_navigator", anonymous=True)
	rospy.loginfo("Beacon Navigator started")
	
	while not rospy.is_shutdown():
		devices = await BleakScanner.discover(timeout=2.5)
		for d in devices:
			mdata = d.metadata.get("manufacturer_data", {})
			if 0x004C in mdata: # Apple iBeacon format
				dist = estimate_distance(d.rssi)
				rospy.loginfo(f"Beacon found: {d.address} | RSSI={d.rssi} | Dist≈{dist}m")
				
				if dist is not None:
					# Create a navigation goal (simple version: forward direction)
					goal = PoseStamped()
					goal.header.frame_id = "base_link"
					goal.header.stamp = rospy.Time.now()
					goal.pose.position.x = dist # move forward toward beacon
					goal.pose.position.y = 0.0
					goal.pose.position.z = 0.0
					goal.pose.orientation.w = 1.0
					
					pub.publish(goal)
					rospy.loginfo("Goal sent to move_base")
				
		await asyncio.sleep(3)

if __name__ == "__main__":
	try:
		asyncio.run(beacon_tracker())
	except rospy.ROSInterruptException:
		pass
```

**Result:** TurtleBot can autonomous navigate to around the position of beacon, **BUT** need to in the robot's front direction only

**Objective:** Make TurtleBot can autonomous navigation to beacon at any direction

**Experiment:**
- Create python script `beacon_navigation_rotational.py`
```
#!/usr/bin/env python3
import rospy
import asyncio
from bleak import BleakScanner
from geometry_msgs.msg import Twist
import math
import time

TX_POWER = -59 # same as in ESP32
ROTATION_STEP = 30 # degrees per step
ROTATION_SPEED = 0.5 # rad/s
FORWARD_SPEED = 0.2
DIST_THRESHOLD = 1.0 # meters
SCAN_TIME = 2.0 # seconds per heading

def estimate_distance(rssi, tx_power=TX_POWER, n=2.2):
	if rssi == 0:
		return None
	ratio = (tx_power - rssi) / (10 * n)
	return round(pow(10, ratio), 2)

async def measure_rssi():
	devices = await BleakScanner.discover(timeout=SCAN_TIME)
	for d in devices:
		mdata = d.metadata.get("manufacturer_data", {})
		if 0x004C in mdata: # Apple iBeacon ID
			return d.rssi
	return None

async def find_beacon_direction(pub):
	rospy.loginfo("Scanning in 360° to find beacon direction...")
	readings = []
	for step in range(int(360 / ROTATION_STEP)):
		# rotate robot
		twist = Twist()
		twist.angular.z = ROTATION_SPEED
		pub.publish(twist)
		rospy.sleep(math.radians(ROTATION_STEP) / ROTATION_SPEED)
		twist.angular.z = 0
		pub.publish(twist)
		rssi = await measure_rssi()
		rospy.loginfo(f"Heading {step * ROTATION_STEP:3d}° -> RSSI {rssi}")
		readings.append((step * ROTATION_STEP, rssi if rssi else -100))
		await asyncio.sleep(0.5)
	# strongest RSSI
	best = max(readings, key=lambda x: x[1])
	rospy.loginfo(f"Strongest signal at {best[0]}° ({best[1]} dBm)")
	return best

async def navigate_to_beacon():
	rospy.init_node("beacon_seek")
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
	rate = rospy.Rate(1)
	
	while not rospy.is_shutdown():
		best = await find_beacon_direction(pub)
		dist = estimate_distance(best[1])
		if dist and dist < DIST_THRESHOLD:
			rospy.loginfo("Beacon reached (within threshold).")
			break
		# rotate toward beacon (short rotation)
		rospy.loginfo("Rotating toward beacon...")
		twist = Twist()
		twist.angular.z = ROTATION_SPEED
		turn_time = math.radians(best[0]) / ROTATION_SPEED
		pub.publish(twist)
		rospy.sleep(turn_time)
		twist.angular.z = 0
		pub.publish(twist)
		# move forward a bit
		rospy.loginfo(f"Moving forward ~{min(dist or 1, 2):.2f} m")
		twist.linear.x = FORWARD_SPEED
		move_time = min(dist or 1, 2) / FORWARD_SPEED
		pub.publish(twist)
		rospy.sleep(move_time)
		twist.linear.x = 0
		pub.publish(twist)
		await asyncio.sleep(1)

if __name__ == "__main__":
	try:
		asyncio.run(navigate_to_beacon())
	except KeyboardInterrupt:
		print("Stopped by user.")
```

**Result:** TurtleBot don't even move