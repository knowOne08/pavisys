# PAVI Flight Computer
## Easy User Manual

---

## What You Have
- **TX Unit**: Goes on your rocket, records flight data
- **RX Unit**: Ground control with menu, controls everything

## What It Does
- Records flight data (altitude, speed, weight)
- Controls pyro channels wirelessly
- Downloads data to your phone/computer
- Works up to 5km away

---

## Quick Setup (First Time)

### 1. Power On Both Units
1. Plug in TX unit (USB cable or battery)
2. Plug in RX unit 
3. Wait 30 seconds - both should show "ready" messages
4. RX unit will show the main menu

### 2. Test Connection
1. On RX menu, choose option 5 (System Status)
2. Should show "TX Connection: ACTIVE"
3. If shows "NO CONNECTION", see troubleshooting below

### 3. Calibrate Weight Sensor (If You Have One)
1. On RX menu, choose option 3 (Load Cell Calibration)
2. Follow the simple steps on screen
3. Done! System is ready to fly

---

## Using the RX Unit Menu

When you turn on the RX unit, you see this main menu:

```
╔═══════════════════════════════════════╗
║     🚀 PAVI GROUND STATION RX 🚀      ║
║      Remote TX Control Interface      ║
╠═══════════════════════════════════════╣
║  1️⃣  - TX Configuration Menu (Local)  ║
║  2️⃣  - Flight Operations Menu         ║
║  3️⃣  - Data Management Menu           ║
║  4️⃣  - TX Calibration Menu            ║
║  5️⃣  - TX WiFi & File Menu            ║
║  6️⃣  - System Status                  ║
║  7️⃣  - ESP32-C3 Diagnostics           ║
║  0️⃣  - Help & About                   ║
╚═══════════════════════════════════════╝
```

### Option 1: TX Configuration Menu ⭐ (Setup Your Flight)

This is where you set up all your flight parameters before sending them to TX:

**Sub-menu options:**
```
╔═══════════════════════════════════════╗
║        ⚙️  TX CONFIGURATION ⚙️        ║
╠═══════════════════════════════════════╣
║  1️⃣  - Set All Parameters (Guided)   ║
║  2️⃣  - Set Data Filename              ║
║  3️⃣  - Set Total Weight (g)           ║
║  4️⃣  - Set Wind Speed (m/s)           ║
║  5️⃣  - Set Drop Height (m)            ║
║  6️⃣  - Toggle Sensor Filters          ║
║  7️⃣  - Show Current Config            ║
║  8️⃣  - Send Config to TX              ║
║  9️⃣  - Clear Configuration            ║
║  0️⃣  - Back to Main Menu              ║
╚═══════════════════════════════════════╝
```

**Option 1-1: Set All Parameters (Easiest)**
- Guided setup walks you through all settings
- Type values or "skip" to keep current
- Sets filename, weight, wind speed, height all at once

**Option 1-8: Send Config to TX**
- Sends all your settings to TX unit at once
- TX confirms it received the configuration
- Must do this before starting flight

### Option 2: Flight Operations Menu 🚀

Controls the actual flight recording and pyro channels:

```
╔═══════════════════════════════════════╗
║       🚀 FLIGHT OPERATIONS 🚀        ║
╠═══════════════════════════════════════╣
║  1️⃣  - Start Data Logging            ║
║  2️⃣  - Stop Data Logging             ║
║  3️⃣  - Fire Pyro 1                   ║
║  4️⃣  - Fire Pyro 2                   ║
║  5️⃣  - Fire Pyro 3                   ║
║  6️⃣  - Fire Pyro 4                   ║
║  7️⃣  - Reset Altitude Offset         ║
║  8️⃣  - TX System Status              ║
║  0️⃣  - Back to Main Menu              ║
╚═══════════════════════════════════════╝
```

**Most Used:**
- **Option 2-1:** Start recording flight data
- **Option 2-2:** Stop recording
- **Option 2-7:** Reset altitude to zero (before launch)
- **Option 2-8:** Check TX status

### Option 3: Data Management Menu 📊

Download your flight data via WiFi:

```
╔═══════════════════════════════════════╗
║       📊 DATA MANAGEMENT 📊          ║
╠═══════════════════════════════════════╣
║  1️⃣  - Start WiFi Hotspot            ║
║  2️⃣  - Stop WiFi Hotspot             ║
║  3️⃣  - WiFi Status                   ║
║  4️⃣  - Show Connection Info          ║
║  0️⃣  - Back to Main Menu              ║
╚═══════════════════════════════════════╝
```

**Download Process:**
1. Choose option 3-1 (Start WiFi Hotspot)
2. Connect phone/computer to "PaviFlightData" network
3. Password: `pavi2024`
4. Go to `192.168.4.1` in browser
5. Download your files
6. Choose option 3-2 (Stop WiFi) when done

### Option 4: TX Calibration Menu ⚖️

Calibrate the load cell (weight sensor):

```
╔═══════════════════════════════════════╗
║        🎯 LOAD CELL CALIBRATION 🎯   ║
╠═══════════════════════════════════════╣
║  1️⃣  - Tare Load Cell (Zero)         ║
║  2️⃣  - Calibrate with Known Weight    ║
║  3️⃣  - Save Calibration               ║
║  4️⃣  - Test Current Calibration      ║
║  0️⃣  - Back to Main Menu              ║
╚═══════════════════════════════════════╝
```

**Calibration Steps:**
1. Option 4-1: Remove all weight, set zero point
2. Option 4-2: Place known weight (like 1kg), calibrate
3. Option 4-3: Save calibration permanently
4. Option 4-4: Test accuracy

### Option 5: TX WiFi & File Menu 🌐

Same as Option 3 but with more WiFi controls:

```
╔═══════════════════════════════════════╗
║        🌐 TX WIFI & FILES 🌐         ║
╠═══════════════════════════════════════╣
║  1️⃣  - Start WiFi Hotspot            ║
║  2️⃣  - Stop WiFi Hotspot             ║
║  3️⃣  - Show WiFi Status              ║
║  4️⃣  - Get Download URL              ║
║  5️⃣  - WiFi File Browser             ║
║  6️⃣  - Restart WiFi Service          ║
║  0️⃣  - Back to Main Menu              ║
╚═══════════════════════════════════════╝
```

### Option 6: System Status 📊

Shows complete system information:
- TX connection status  
- Signal strength (RSSI)
- Free memory
- SD card status
- Requests full status from TX unit

### Option 7: ESP32-C3 Diagnostics 🔧

Technical information for troubleshooting:
- Memory usage
- CPU frequency  
- Reset information
- LoRa status

---

## Complete Flight Procedure (Step by Step)

### Step 1: Configure Your Flight (Main Menu → Option 1)
1. Choose **Option 1** from main menu (TX Configuration)
2. Choose **Option 1-1** (Set All Parameters - Guided)
3. Enter when prompted:
   - **Filename:** `MyFlight` (or whatever you want)
   - **Weight:** `2500` (weight in grams)  
   - **Wind Speed:** `5` (meters per second)
   - **Height:** `100` (launch height in meters)
4. Choose **Option 1-8** (Send Config to TX)
5. Wait for "✅ Configuration sent!" message

### Step 2: Start Recording (Main Menu → Option 2)
1. Go back to main menu (type `0`)
2. Choose **Option 2** (Flight Operations)
3. Choose **Option 2-7** (Reset Altitude Offset) - do this at launch site
4. Choose **Option 2-1** (Start Data Logging)
5. Wait for "✅ TX ACK: Flight logging started"

### Step 3: Launch Your Rocket
- RX unit stays with you
- TX unit records flight data automatically
- You can fire pyro channels if needed (Options 2-3 through 2-6)

### Step 4: Stop Recording (After Flight)
1. Choose **Option 2-2** (Stop Data Logging)
2. Wait for "✅ TX ACK: Flight logging stopped"

### Step 5: Download Data (Main Menu → Option 3)
1. Go to main menu, choose **Option 3** (Data Management)
2. Choose **Option 3-1** (Start WiFi Hotspot)  
3. On your phone/computer:
   - Connect to WiFi "PaviFlightData"
   - Password: `pavi2024`
   - Open browser: `192.168.4.1`
   - Download your flight file
4. Choose **Option 3-2** (Stop WiFi Hotspot)

---

## Downloading Your Flight Data

### Method 1: From RX Menu (Easiest)
1. Choose option 4 (File Management)
2. Select "Start WiFi"
3. On your phone/computer:
   - Connect to WiFi "PaviFlightData"
   - Password: `pavi2024`
   - Open browser, go to `192.168.4.1`
   - Download your flight files
4. Choose "Stop WiFi" to save battery

### Method 2: Direct Commands
1. In Manual Commands (option 2), send: `WIFI_START`
2. Follow same WiFi steps above
3. Send: `WIFI_STOP` when done

### Opening Your Data
- Files are CSV format
- Open in Excel, Google Sheets, or similar
- Contains: time, altitude, acceleration, weight data

---

## Load Cell Calibration (Weight Sensor)

### When to Calibrate
- First time using system
- Weight readings seem wrong
- After rough handling

### Simple Calibration (From RX Menu)
1. **Main Menu:** Choose **Option 4** (TX Calibration Menu)
2. **Step 1:** Choose **Option 4-1** (Tare Load Cell) - removes all weight
3. **Step 2:** Choose **Option 4-2** (Calibrate with Known Weight)
   - Enter weight in grams (like `1000` for 1kg)
   - Place that exact weight on TX load cell
4. **Step 3:** Choose **Option 4-3** (Save Calibration) - saves permanently  
5. **Step 4:** Choose **Option 4-4** (Test) - verify accuracy

### Testing Your Calibration
- Remove all weight: should read close to 0.00kg
- Place known weight: should read correct value
- If error more than 5%, repeat calibration

---

## Quick Troubleshooting

### Problem: "No TX Connection" or "Connection Lost"

**Quick Fixes:**
1. **Check Power:** Make sure TX unit is plugged in
2. **Move Closer:** Get within 50 meters for testing
3. **Check Antennas:** Make sure both antennas connected
4. **Restart Both:** Unplug and plug back in both units
5. **Test Command:** Try option 2, send `PING` command

### Problem: "Load Cell Not Working" or Wrong Weight

**Quick Fixes:**
1. **Recalibrate:** Use option 3 from menu
2. **Check Connections:** Make sure weight sensor plugged in
3. **Remove Everything:** Nothing should touch the sensor
4. **Stable Surface:** Put sensor on solid, level surface
5. **Wait:** Let sensor settle for 30 seconds

### Problem: "Can't Connect to WiFi" for Data Download

**Quick Fixes:**
1. **Start WiFi:** Make sure you chose "Start WiFi" first
2. **Forget Network:** Delete old "PaviFlightData" from phone
3. **Correct Password:** Use `pavi2024` (all lowercase)
4. **Turn Off Data:** Disable cellular data on phone
5. **Right Address:** Go to `192.168.4.1` (not .com)

### Problem: "Storage Full" or "Can't Save Data"

**Quick Fixes:**
1. **Download Files:** Use option 4 to get your data
2. **Delete Old:** Remove old flight files
3. **Check Space:** Status shows storage percentage
4. **Shorter Names:** Use shorter filenames

### Problem: "Sensors Not Ready" or Startup Errors

**Quick Fixes:**
1. **Wait Longer:** Give 60 seconds for full startup
2. **Power Cycle:** Unplug for 10 seconds, plug back in
3. **Check Connections:** Make sure all sensors connected
4. **USB Power:** Try different USB cable/power source

### Problem: Poor Signal or "Weak Signal"

**Quick Fixes:**
1. **Line of Sight:** Remove obstacles between units
2. **Higher Position:** Put RX unit higher up
3. **Check Antennas:** Make sure antennas vertical
4. **Move Closer:** Reduce distance for testing
5. **Avoid Interference:** Away from WiFi routers, phones

---

## Safety - READ THIS!

### Pyro Channels (Rocket Ignition)
- **ALWAYS double-check** before using PYRO commands
- Pyro channels automatically turn off after 1.5 seconds
- Keep safe distance when testing
- Only use proper pyrotechnic devices

### General Safety
- Test system completely before important flights
- Keep spare batteries
- Download data immediately after flights
- Check local laws for 433MHz radio use

---

## Data Analysis Made Simple

### Your Data File Contains:
- **Time**: Seconds since launch
- **Altitude**: Height above launch point (meters)
- **Acceleration**: G-forces in 3 directions
- **Weight**: Load cell reading (kg)

### Quick Analysis in Excel:
1. **Altitude Graph:** Plot Altitude vs Time
2. **Max Altitude:** Find highest altitude value
3. **Flight Time:** Time when altitude returns to zero
4. **Max Acceleration:** Find highest G-force

---

## Quick Reference Card

### Essential Menu Navigation:
- **Main Menu Options 1-7, 0:** Always available by typing number
- **Type `0`:** Go back to previous menu
- **Type `99`:** Jump to main menu from anywhere

### Most Common Flight Sequence:
1. **Main Menu → 1 → 1-1:** Set all flight parameters 
2. **→ 1-8:** Send config to TX
3. **Main Menu → 2 → 2-7:** Reset altitude at launch site
4. **→ 2-1:** Start recording
5. **→ 2-2:** Stop recording after flight
6. **Main Menu → 3 → 3-1:** Start WiFi for download

### Emergency Commands:
- **Main Menu → 2 → 2-2:** Emergency stop recording
- **Main Menu → 2 → 2-8:** Check TX status
- **Main Menu → 6:** Full system status

### WiFi Download Quick Steps:
1. **Menu 3 → 3-1:** Start WiFi
2. **Connect to:** `PaviFlightData`
3. **Password:** `pavi2024`  
4. **Browser:** `192.168.4.1`
5. **Menu 3 → 3-2:** Stop WiFi

---

**Questions? Problems?**
1. Try the troubleshooting section above
2. Check system status (option 5)
3. Power cycle both units
4. Keep this manual handy!

---

*User Manual for PAVI Flight Computer System v1.0*
