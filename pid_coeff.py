#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

def calculate_pid_scaled():
    print("=== Buck85 PID Calculator (General Purpose / Fixed-Point) ===\n")

    # --- 1. Power Stage Parameters ---
    Vin = 1.1      # Input Voltage (V)
    f_sw = 31250.0 # Switching frequency (Hz)
    prescaler = 4  # PID runs every 4th cycle (approx 7.8kHz)
    L = 47e-6      # 47uH
    C = 264e-6     # 220uF + 2x22uF total capacitance

    # --- 2. Hardware Scaling Parameters ---
    r_top = 35700.0   # 35.7k
    r_bottom = 10000.0 # 10k
    v_ref = 1.1       # ATtiny85 Internal Reference
    adc_res = 1024    # 10-bit ADC
    
    # Calculate ADC counts per 1 Volt
    divider_ratio = r_bottom / (r_top + r_bottom)
    counts_per_volt = (divider_ratio / v_ref) * adc_res
    
    # --- 3. Control Theory Calculations ---
    # We use a FIXED R_design (5.0 Ohms) so coefficients are Vout-independent.
    R_design = 5.0 
    Ts = (1.0 / f_sw) * prescaler
    
    omega_0 = 1.0 / math.sqrt(L * C)
    Q = R_design * math.sqrt(C / L)
    
    f_c = 0.10 * f_sw # 10% crossover
    omega_c = 2 * math.pi * f_c
    
    Ti = 1.0 / (omega_c / 10.0)
    Td = 1.0 / (omega_c / 10.0)
    
    v_ramp = 255.0 # 8-bit PWM scale
    ratio = omega_c / omega_0
    plant_mag = Vin / math.sqrt((1.0 - ratio**2)**2 + (ratio/Q)**2)
    Kp = 1.0 / ((1.0 / v_ramp) * plant_mag)

    # Discrete incremental coefficients
    a0_raw = Kp * (1.0 + Ts / (2.0 * Ti) + Td / Ts)
    a1_raw = Kp * (-1.0 + Ts / (2.0 * Ti) - 2.0 * Td / Ts)
    a2_raw = Kp * (Td / Ts)

    # Conversion to ATtiny85 Fixed-Point (Scale by 256)
    A0_fixed = round((a0_raw / counts_per_volt) * 256)
    A1_fixed = round((a1_raw / counts_per_volt) * 256)
    A2_fixed = round((a2_raw / counts_per_volt) * 256)

    print(f"--- ATtiny85 ISR Coefficients (Scaled by 256) ---")
    print(f"const int a0 = {A0_fixed};")
    print(f"const int a1 = {A1_fixed};")
    print(f"const int a2 = {A2_fixed};")

if __name__ == "__main__":
    calculate_pid_scaled()
