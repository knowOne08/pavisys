# PAVI Flight Computer
## Simple User Manual

---

## What's in the Box
You have two devices:
- **TX Unit**: Goes on your rocket, collects flight data
- **RX Unit**: Stays with you on the ground, controls the TX unit

## What This System Does
- Records flight data (altitude, acceleration, weight)
- Lets you control pyrotechnic channels wirelessly
- Downloads data to your phone/computer after flight
- Works up to 5km distance

---

## Before Your First Flight

### Step 1: Power On Both Units
1. Connect TX unit to power (USB cable or battery)
2. Connect RX unit to power
3. Wait 30 seconds for startup
4. Check that both units show "ready" messages

### Step 2: Test Communication
1. On RX unit, send command: `PING`
2. TX should respond with: "PONG - TX alive"
3. If no response, check antennas and distance

### Step 3: Calibrate Load Cell (If Using Weight Sensor)
**Simple Method:**
1. On RX unit, send: `CALIB`
2. Follow the prompts on TX unit screen:
   - Remove all weight when asked
   - Place known weight when asked (like 1kg)
   - Wait for "Calibration Complete"

**That's it! Your system is ready.**

---

## How to Use the RX Unit (Ground Control)

### RX Unit Menu System
When you power on the RX unit, you'll see a menu:

```
=== PAVI RX CONTROL ===
1. Quick Start Flight
2. Manual Commands  
3. Load Cell Calibration
4. File Management
5. System Status
Choose option (1-5):
```

### Option 1: Quick Start Flight (Easiest Way)
This guides you through a complete flight setup:

1. **Enter Flight Details:**
   - Filename: `MyFlight`
   - Rocket weight: `2.5` (kg)
   - Wind speed: `5` (m/s) 
   - Launch height: `100` (meters)

2. **Start Recording:**
   - System automatically configures TX unit
   - Shows "Ready to Launch" when done
   - Press any key to start recording

3. **During Flight:**
   - Real-time altitude display
   - Signal strength indicator
   - Emergency stop available

4. **After Flight:**
   - Automatically stops recording
   - Shows total flight time
   - Guides you to download data

### Option 2: Manual Commands
For advanced users who want direct control:

**Basic Commands:**
- `STATUS` - Check if TX unit is working
- `PING` - Test connection
- `FLIGHT_START` - Start recording data
- `FLIGHT_STOP` - Stop recording

**Pyro Commands (BE CAREFUL!):**
- `PYRO1` - Fire pyro channel 1
- `PYRO2` - Fire pyro channel 2  
- `PYRO3` - Fire pyro channel 3
- `PYRO4` - Fire pyro channel 4

### Option 3: Load Cell Calibration
Easy calibration menu:
1. Choose "Start Calibration"
2. Remove all weight from load cell
3. Place known weight (0.5kg, 1kg, etc.)
4. System calculates automatically
5. Test accuracy

### Option 4: File Management  
- View files on TX unit
- Start WiFi for downloading
- Delete old files

### Option 5: System Status
Shows:
- Connection status
- Battery levels
- Signal strength
- Sensor status

---

## How to Configure the TX Unit

### Method 1: Using RX Unit (Recommended)
The RX unit can configure everything for you. Just use "Quick Start Flight" and enter your details.

### Method 2: Direct Commands
If you connect TX unit to computer via USB:

**Flight Configuration:**
```
Send: CONFIG:MyFlight,2.5,5,100
Response: Configuration applied successfully
```

**Start Recording:**
```
Send: FLIGHT_START  
Response: Flight logging started
```

**Stop Recording:**
```
Send: FLIGHT_STOP
Response: Flight logging stopped
```

---

## Load Cell Calibration (Detailed)

### When to Calibrate
- First time using the system
- If weight readings seem wrong
- After rough handling

### Easy Calibration Steps
1. **From RX unit:** Select "Load Cell Calibration" from menu
2. **From TX unit directly:** Send command `CALIB`

3. **Follow these steps:**
   - **Step 1:** Remove ALL weight from load cell
   - **Step 2:** Enter weight you'll use (like `1` for 1kg)
   - **Step 3:** Place that exact weight on load cell
   - **Step 4:** Wait for "Calibration Complete"

### Testing Calibration
- Remove weight: should read close to 0.00kg
- Place known weight: should read correct value
- If error is more than 5%, recalibrate

---

## Getting Your Flight Data

### Step 1: Enable WiFi on TX Unit
**From RX unit:** Select "File Management" → "Start WiFi"
**Direct command:** Send `WIFI_START` to TX unit

### Step 2: Connect to WiFi
- Network name: `PaviFlightData` or `PaviFlightData-2`
- Password: `pavi2024`

### Step 3: Download Files
1. Open web browser
2. Go to: `192.168.4.1`
3. Click "Download" next to your flight file
4. File opens in Excel or similar program

### Step 4: Turn Off WiFi
**From RX unit:** Select "Stop WiFi"
**Direct command:** Send `WIFI_STOP`

---

## Understanding Your Data

Your flight data is saved as a CSV file with these columns:

- **Time_ms**: Time since launch (milliseconds)
- **Event**: What happened (DATA = normal reading, PYRO1 = pyro fired)
- **Alt_m**: Height above launch point (meters)
- **LoadCell_kg**: Weight/thrust (kilograms)
- **AccelX/Y/Z_ms2**: Acceleration in 3 directions
- **GyroX/Y/Z_rads**: Rotation speed in 3 directions

**Easy Analysis:**
- Open CSV in Excel
- Make a graph of Alt_m vs Time_ms for altitude plot
- Make a graph of AccelZ_ms2 vs Time_ms for acceleration

---

## Common Problems & Solutions

### "No response from TX unit"
- Check both units are powered on
- Move closer together (under 100m for testing)
- Check antennas are connected
- Try sending `PING` command

### "Load cell reads wrong weight"
- Recalibrate using `CALIB` command
- Check nothing is touching the load cell
- Make sure load cell is stable

### "Can't connect to WiFi"
- Make sure you sent `WIFI_START` command first
- Check network name matches your unit number
- Password is `pavi2024` (all lowercase)

### "Data file won't download"
- Try refreshing the web page
- Check you're connected to right WiFi network
- Make sure TX unit has power

---

## Safety Important!

### Pyrotechnic Channels
- **ALWAYS** double-check before using PYRO commands
- Pyro channels turn off automatically after 1.5 seconds
- Keep safe distance when testing
- Use only certified pyrotechnic devices

### Radio Safety
- 433MHz is legal in most countries
- Don't use near airports or emergency services
- Maximum range is about 5km

---

## Quick Reference

### Essential RX Commands
- **Quick flight:** Use menu option 1
- **Test connection:** Send `PING`
- **Emergency stop:** Send `FLIGHT_STOP`
- **Download data:** Menu option 4

### Essential TX Commands (if using direct USB)
- **Calibrate:** Send `CALIB`
- **Start flight:** Send `FLIGHT_START`
- **Stop flight:** Send `FLIGHT_STOP`
- **Check status:** Send `STATUS`

**Need help?** Check the status messages on both units - they usually tell you what's wrong!

---

*This manual covers firmware version TX_Pavi_Optimized_v1*
*For technical support, keep this manual with your device*
