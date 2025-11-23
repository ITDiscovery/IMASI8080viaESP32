"""
blinkin_driver.py
Universal Driver for 74HC595 (Output) and 74HC165 (Input).
Target Hardware: IMSAI 8080 Front Panel / Blinkin' Board.
"""

import gpiod
import time
from gpiod.line import Direction, Value
from typing import Optional

class BlinkinBoard:
    # --- DEFAULTS ---
    # Output (LEDs) - BCM Numbering
    DEFAULT_LED_CLK = 21
    DEFAULT_LED_LATCH = 20
    DEFAULT_LED_DATA = 16
    
    # Input (Switches) - Verified Hardware Mapping
    DEFAULT_SW_CLK = 17    
    DEFAULT_SW_LATCH = 22  
    DEFAULT_SW_DATA = 27   

    def __init__(self, 
                 # Output Config
                 led_clk: Optional[int] = None, 
                 led_latch: Optional[int] = None, 
                 led_data: Optional[int] = None,
                 num_led_chips: int = 5,
                 
                 # Input Config
                 sw_clk: Optional[int] = None,
                 sw_latch: Optional[int] = None,
                 sw_data: Optional[int] = None,
                 num_switch_chips: int = 4, # 32 bits (Matches AL/AH/CL/CH structure)
                 
                 # Timing Tuning (Matches C++ Defines)
                 # Ldelay = 1us = 0.000001
                 latch_delay: float = 0.000001, 
                 # Cdelay = 4us = 0.000004
                 clock_delay: float = 0.000004, 

                 chip_name: str = "gpiochip4",
                 led_inversion_mask: int = 0x0):
        
        # 1. Setup Config
        self.led_clk = led_clk if led_clk is not None else self.DEFAULT_LED_CLK
        self.led_latch = led_latch if led_latch is not None else self.DEFAULT_LED_LATCH
        self.led_data = led_data if led_data is not None else self.DEFAULT_LED_DATA
        
        self.sw_clk = sw_clk if sw_clk is not None else self.DEFAULT_SW_CLK
        self.sw_latch = sw_latch if sw_latch is not None else self.DEFAULT_SW_LATCH
        self.sw_data = sw_data if sw_data is not None else self.DEFAULT_SW_DATA

        self.num_led_bits = num_led_chips * 8
        self.num_sw_bits = num_switch_chips * 8
        self._chip_path = f"/dev/{chip_name}"
        self.led_inversion_mask = led_inversion_mask
        
        self.latch_delay = latch_delay
        self.clock_delay = clock_delay
        
        # Internal State
        self._led_buffer = 0
        self.request = None

        self._initialize_hardware()

    def _initialize_hardware(self):
        try:
            config = {}
            
            # --- OUTPUTS (74HC595) ---
            # Latch ACTIVE (High), Clock/Data INACTIVE (Low)
            config[self.led_latch] = gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.ACTIVE)
            config[self.led_clk]   = gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)
            config[self.led_data]  = gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)
            
            # --- INPUTS (74HC165) ---
            # Idle State: Latch HIGH, Clock LOW
            config[self.sw_latch] = gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.ACTIVE)
            config[self.sw_clk]   = gpiod.LineSettings(direction=Direction.OUTPUT, output_value=Value.INACTIVE)
            config[self.sw_data]  = gpiod.LineSettings(direction=Direction.INPUT) 

            self.request = gpiod.request_lines(
                self._chip_path,
                consumer="blinkin_driver",
                config=config
            )
        except Exception as e:
            raise RuntimeError(f"Hardware Init Failed on {self._chip_path}: {e}")

    def __enter__(self): return self
    def __exit__(self, exc_type, exc_value, traceback): self.close()

    def close(self):
        if self.request:
            self.clear_leds()
            self.update_leds()
            self.request.release()
            self.request = None

    # --- LOW LEVEL GPIO ---
    def _set(self, pin: int, state: bool):
        self.request.set_value(pin, Value.ACTIVE if state else Value.INACTIVE)

    def _get(self, pin: int) -> int:
        return 1 if self.request.get_value(pin) == Value.ACTIVE else 0

    # ==========================
    # OUTPUT SUB-SYSTEM (LEDs)
    # ==========================
    def update_leds(self):
        """Push the current buffer state to the physical LEDs."""
        if not self.request: return
        phys_val = self._led_buffer ^ self.led_inversion_mask

        # 1. Open Latch (Low)
        self._set(self.led_latch, False)
        
        # 2. Shift Data (MSB First: Bit 39 down to 0)
        for i in range(self.num_led_bits - 1, -1, -1):
            bit = (phys_val >> i) & 1
            self._set(self.led_data, bool(bit))
            self._set(self.led_clk, True)
            self._set(self.led_clk, False)
        
        # 3. Close Latch (High)
        self._set(self.led_latch, True)

    def set_led(self, index: int, state: bool):
        if 0 <= index < self.num_led_bits:
            if state: self._led_buffer |= (1 << index)
            else: self._led_buffer &= ~(1 << index)

    def clear_leds(self): self._led_buffer = 0
    def fill_leds(self): self._led_buffer = (1 << self.num_led_bits) - 1
    def set_all_leds(self, val: int): self._led_buffer = val

    # ==========================
    # INPUT SUB-SYSTEM (Switches)
    # ==========================
    def read_switches(self) -> int:
        """
        Reads 74HC165 input chain with verified timing logic.
        """
        if not self.request: return 0

        # 1. LATCH SEQUENCE (Pulse LOW)
        self._set(self.sw_latch, False)
        time.sleep(self.latch_delay)     # Critical Wait
        self._set(self.sw_latch, True)
        time.sleep(self.latch_delay)     # Critical Wait

        # 2. SHIFT SEQUENCE
        raw_val = 0
        
        # Read MSB First (Standard for daisy-chained 165s)
        for i in range(self.num_sw_bits - 1, -1, -1):
            # Read Data (Sample before clock rising edge)
            bit = self._get(self.sw_data)
            
            if bit:
                raw_val |= (1 << i)

            # Pulse Clock (Low -> High -> Low)
            self._set(self.sw_clk, True)
            time.sleep(self.clock_delay) # Hold High
            self._set(self.sw_clk, False)
            time.sleep(self.clock_delay) # Hold Low

        return raw_val