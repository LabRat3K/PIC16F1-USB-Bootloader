# YAMKE v1 (Yet Another MAME Keyboard Encoder)

A USB Composite dual-keyboard and virtual serial (CDC) arcade controller firmware for the **Microchip PIC16F1459**, compiled with **SDCC** and designed to run alongside Matt Sarnoff's 512-word USB bootloader.

## Features
* **Dual USB HID Boot Keyboards:** Presents two independent standard HID keyboard interfaces (`MI_00` and `MI_01`) for 2-player arcade controls with zero ghosting and 1 ms Timer2 bitwise debouncing.
* **USB CDC Virtual COM Port (`MI_02`):** Provides a built-in serial command-line interface for runtime configuration, LED control, and diagnostics without reflashing.
* **5 Switchable Keymap Profiles:** Supports independent profile assignment per player (`0`–`4`, including a dedicated Coin Door profile).
* **Non-Volatile Settings (HEF):** Commits player keymap profiles and a custom 9-character USB serial number to the PIC16F1459's High-Endurance Flash (`0x1F80`) so settings persist across power cycles.
* **Unique Multi-Board Serial Numbers:** Supports programming unique USB serial strings (`YAMKE0001`, `YAMKE0002`, etc.) over CDC so Windows and Linux assign consistent COM ports and device paths when multiple encoders are connected.

---

## Hardware Pinout (PIC16F1459 — 20-Pin DIP/SOIC/SSOP)

All 12 arcade switch inputs are **active-low** (`GND` = pressed) and require **external pull-up resistors tied to +5V (`VDD`)**.

| Pin | Port | Function | Description |
| :--- | :--- | :--- | :--- |
| **1** | `VDD` | Power | +5V USB VBUS supply |
| **2** | `RA5` | `J1 UP` | Player 1 Joystick Up |
| **3** | `RA4` | `J1 RIGHT` | Player 1 Joystick Right |
| **4** | `RA3` | `PROG_BUTTON` | Bootloader / Reset switch (external pull-up to +5V; ground to reset) |
| **5** | `RC5` | `J1 DOWN` | Player 1 Joystick Down |
| **6** | `RC4` | `J1 LEFT` | Player 1 Joystick Left |
| **7** | `RC3` | `J1 FIRE` | Player 1 Button 2 (Fire) |
| **8** | `RC6` | `J1 SELECT` | Player 1 Button 1 (Select / Coin) |
| **9** | `RC7` | `J2 UP` | Player 2 Joystick Up |
| **10** | `RB7` | `J2 RIGHT` | Player 2 Joystick Right |
| **11** | `RB6` | `J2 DOWN` | Player 2 Joystick Down |
| **12** | `RB5` | `J2 LEFT` | Player 2 Joystick Left |
| **13** | `RB4` | `J2 FIRE` | Player 2 Button 2 (Fire) |
| **14** | `RC2` | `J2 SELECT` | Player 2 Button 1 (Select / Coin) |
| **15** | `RC1` | `J2 LED` | Player 2 LED output (drives MM3906 transistor) |
| **16** | `RC0` | `J1 LED` | Player 1 LED output (drives MM3906 transistor) |
| **17** | `VUSB3V3` | USB LDO | Internal 3.3V USB transceiver regulator (0.47 µF cap to GND) |
| **18** | `RA1` | `USB D-` | USB Data- |
| **19** | `RA0` | `USB D+` | USB Data+ |
| **20** | `VSS` | Ground | Common GND |

---

## Default Keymap Profiles

Each player interface (`P1` or `P2`) can be assigned any of the 5 built-in profiles (`0`–`4`):

| Input | Profile 0 (P1 Default) | Profile 1 (P2 Default) | Profile 2 (P3) | Profile 3 (P4) | Profile 4 (Coin Door) |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Select / Btn 1** | `Left Ctrl` | `A` | `Right Ctrl` | `B` | `5` |
| **Joystick Up** | `Up Arrow` | `R` | `I` | `Y` | `6` |
| **Fire / Btn 2** | `Left Alt` | `S` | `Right Shift` | `E` | `7` |
| **Joystick Left** | `Left Arrow` | `D` | `J` | `V` | `8` |
| **Joystick Down** | `Down Arrow` | `F` | `K` | `N` | *None* |
| **Joystick Right** | `Right Arrow` | `G` | `L` | `U` | *None* |

---

## Building & Flashing

### Prerequisites
* **SDCC** (Small Device C Compiler, v4.0.0+ with `--use-non-free` PIC14 headers)
* **gputils** (`gpasm`, `gplink`, `gpdasm`)
* **usb16f1prog** (for flashing via Matt Sarnoff's 512-word USB CDC bootloader)

### Build Commands
```bash
# Compile and link yamke_v1.hex
make

# Enter bootloader mode (hold PROG_BUTTON / RA3 low during reset) and flash
make flash

# Clean build artifacts
make clean
```

---

## CDC Serial Command Reference

Connect to the encoder's virtual serial port (`COMx` on Windows, `/dev/ttyACM0` on Linux) at **9600 baud, 8N1**. Commands are case-insensitive and terminated with `\r` or `\n`.

| Command | Arguments | Description | Example |
| :--- | :--- | :--- | :--- |
| `STATUS` | *None* | Displays the active keymap profile index (`0`–`4`) for Player 1 and Player 2. | `STATUS` → `P1:0 P2:1` |
| `SET` | `P<1-2> <0-4>` | Sets Player 1 or Player 2 to keymap profile `0`–`4` in RAM. | `SET P1 2` → `OK` |
| `LED` | `<1-2> <0\|1>` | Turns LED 1 (`RC0`) or LED 2 (`RC1`) ON (`1`) or OFF (`0`). | `LED 1 1` → `OK` |
| `SN` | `<9-chars>` | Updates the 9-character USB serial number string in RAM. | `SN YAMKE0002` → `OK (RUN SAVE & REBOOT)` |
| `SAVE` | *None* | Commits current `P1`/`P2` profiles and USB serial number to High-Endurance Flash (`0x1F80`). | `SAVE` → `SAVED` |
| `?` / `HELP` | *None* | Prints a summary of supported serial commands. | `?` |

### Provisioning Multiple Boards
To assign a unique USB serial number to a newly flashed encoder:
1. Open the device's serial terminal and send:
   ```text
   SN YAMKE0002
   SAVE
   REBOOT
   ```
2. The device will restart and re-enumerate with the new serial number (`USB\VID_04D8&PID_E3D6\YAMKE0002`).

---

## Acknowledgments
* Original PIC16F1459 USB framework by **Szymon Roslowski** (based on work by Alexander Enzmann and Alberto Maccioni).
* 512-word USB CDC bootloader and linker script by **Matt Sarnoff**.
* Firmware debugging, USB composite/CDC integration, and High-Endurance Flash persistence developed with assistance from **Google Gemini**.

