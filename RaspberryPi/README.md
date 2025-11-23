# Blinkin' Universal Driver
A robust, hardware-agnostic Python driver for controlling daisy-chained 74HC595 Shift Registers (Outputs/LEDs) and reading 74HC165 Shift Registers (Inputs/Switches) via Raspberry Pi GPIO.
Designed originally for IMSAI 8080 Front Panel replicas and Apollo DSKY simulations, this driver handles the low-level bit-banging, timing signal stability, and logic inversion required for mixed-voltage legacy hardware.

## Features
Universal Pinout: Configurable Clock, Latch, and Data pins for both Input and Output chains.
Hardware Tuning: Tunable latch_delay and clock_delay to accommodate slow optocouplers or level shifters (fixes "flickering" or "missed bit" issues).
Logic Inversion: Built-in XOR masks allow you to handle Active-Low vs Active-High hardware entirely in software.
Context Manager: Supports Python with syntax for automatic resource cleanup.
Libgpiod: Uses modern Linux GPIO character device access (no deprecated sysfs).

## Hardware Requirements
Outputs: 74HC595 (or 74LS595) Shift Registers.
Inputs: 74HC165 (or 74LS165) Shift Registers.
Platform: Raspberry Pi (Zero/3/4/5) running Raspberry Pi OS.

## Installation
Ensure libgpiod is installed on your Raspberry Pi:
```
sudo apt update
sudo apt install gpiod python3-libgpiod
```

## Quick Start
1. Basic Usage (Defaults)
The driver defaults to the IMSAI 8080 / Blinkin' Board pinout.
```
from blinkin_driver import BlinkinBoard
import time

# Initialize with defaults (5 LED chips, 4 Switch chips)
with BlinkinBoard() as board:
    
    # --- OUTPUTS ---
    # Turn on every other LED (Alternating pattern)
    board.set_all_leds(0xAAAAAAAAAA) 
    board.update_leds()
    
    # --- INPUTS ---
    # Read the current state of the switches
    switches = board.read_switches()
    print(f"Switch State: {hex(switches)}")
```
2. Custom Pinout & Tuning
For custom hardware or noisy electrical environments.

```
board = BlinkinBoard(
    # Custom Pins (BCM Numbering)
    led_clk=13, led_latch=19, led_data=26,
    sw_clk=5,   sw_latch=6,   sw_data=12,
    
    # Hardware Configuration
    num_led_chips=2,       # 16 LEDs
    num_switch_chips=1,    # 8 Switches
    
    # Timing Tuning (Crucial for 74HC165 stability)
    latch_delay=0.000001,  # 1 microsecond
    clock_delay=0.000004   # 4 microseconds
)
```

## Configuration Reference
__init__ Parameters

| Parameter	| Type	| Default	| Description |
Output (LED) Config			
led_clk	int	21	Shift Clock (SH_CP) Pin
led_latch	int	20	Storage/Latch Clock (ST_CP) Pin
led_data	int	16	Serial Data (DS) Pin
num_led_chips	int	5	Number of daisy-chained 595 chips
led_inversion_mask	int	0x0	XOR mask. Set bits to 1 to invert LED logic.
Input (Switch) Config			
sw_clk	int	17	Shift Clock (CP) Pin
sw_latch	int	22	Parallel Load/Latch (PL) Pin
sw_data	int	27	Serial Data (Q7) Pin
num_switch_chips	int	4	Number of daisy-chained 165 chips
sw_inversion_mask	int	0x0	XOR mask. Set 0xFF.. if inputs are inverted.
Timing			
latch_delay	float	1e-6	Pause after Latch toggle (Seconds).
clock_delay	float	4e-6	Pause during Clock pulse (Seconds).

