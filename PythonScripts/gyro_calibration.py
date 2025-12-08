import matplotlib.pyplot as plt
import sys
import os

# ================= USER CONFIGURATION =================

# 1. LOG FILE SELECTION
# Replace this string with the actual name of the log file you want to test
LOG_FILE_NAME = "gyro_biases_static_test_20251209_015905.txt"

# 2. CALIBRATION VARIABLES (MANUAL ENTRY)
# Enter the Biases you calculated (b) - Units: dps
b_x = 1.01405
b_y = -0.74114
b_z = 0.60313

# Enter the Scale Factor Errors you calculated (s) - Dimensionless
# Example: -0.015 means a -1.5% error
s_x = 0.37017
s_y = 0.04670
s_z = -0.17063

# Sensitivity (L3G4200D 250dps)
SENSITIVITY = 0.00875043752

# ======================================================

def parse_log(filename):
    """
    Parses the log file to extract continuous data.
    """
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found.")
        print("Please check the 'LOG_FILE_NAME' variable at the top of the script.")
        sys.exit(1)
        
    print(f"Reading file: {filename}...")
    
    # Storage for continuous data
    data = {'x': [], 'y': [], 'z': []}
    
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
            
        for line in lines:
            line = line.strip()
            
            # Skip headers, START/END tags, Cycle tags, and Summary Headers
            # Added "Axis" and "Temperature" to explicitly skip the new CSV summaries
            if (',' in line and 
                "START" not in line and 
                "END" not in line and 
                "CYCLE" not in line and 
                "Axis" not in line and 
                "Temperature" not in line):
                
                parts = line.split(',')
                if len(parts) >= 3:
                    try:
                        # Parse Raw ADC values
                        # Note: This int() conversion is the primary filter. 
                        # It naturally fails on float summary lines (e.g. "12.34"), skipping them.
                        rx = int(parts[0])
                        ry = int(parts[1])
                        rz = int(parts[2])
                        
                        # Convert to DPS immediately
                        data['x'].append(rx * SENSITIVITY)
                        data['y'].append(ry * SENSITIVITY)
                        data['z'].append(rz * SENSITIVITY)
                    except ValueError:
                        # This catches any lines that aren't pure integers (like the summary floats)
                        pass
    except Exception as e:
        print(f"Error reading file: {e}")
        sys.exit(1)
                    
    return data

def apply_calibration(val, bias, scale_error):
    # Calibration Formula: 
    # calibrated = (raw - bias) / (1 + scale_error)
    return (val - bias) / (1.0 + scale_error)

def generate_calibrated_data(raw_data):
    calib = {'x': [], 'y': [], 'z': []}
    
    print("\nApplying Calibration Parameters:")
    print(f"  Biases -> X: {b_x}, Y: {b_y}, Z: {b_z}")
    print(f"  ScaleErrs -> X: {s_x}, Y: {s_y}, Z: {s_z}")
    
    calib['x'] = [apply_calibration(v, b_x, s_x) for v in raw_data['x']]
    calib['y'] = [apply_calibration(v, b_y, s_y) for v in raw_data['y']]
    calib['z'] = [apply_calibration(v, b_z, s_z) for v in raw_data['z']]
    
    return calib

def plot_two_graphics(raw_data, calib_data):
    samples = range(len(raw_data['x']))
    
    # Create single figure with 3 overlapping subplots
    fig, (ax_x, ax_y, ax_z) = plt.subplots(3, 1, figsize=(10, 12))
    fig.canvas.manager.set_window_title('Gyroscope Calibration Comparison')
    fig.suptitle(f"Calibration Verification\nFile: {LOG_FILE_NAME}\nBias: [{b_x}, {b_y}, {b_z}] | ScaleErr: [{s_x}, {s_y}, {s_z}]", 
                 fontsize=14, fontweight='bold')
    
    # --- X Plot ---
    # Uncalibrated (Strong Blue)
    ax_x.plot(samples, raw_data['x'], color='mediumblue', label='Uncalibrated', linewidth=1.5, alpha=0.7)
    # Calibrated (Sky Blue)
    ax_x.plot(samples, calib_data['x'], color='deepskyblue', label='Calibrated', linewidth=1.5, alpha=0.9)
    ax_x.set_ylabel("Angular Velocity X [dps]", fontweight='bold')
    ax_x.legend(loc='upper right')
    ax_x.grid(True, alpha=0.3)
    
    # --- Y Plot ---
    ax_y.plot(samples, raw_data['y'], color='mediumblue', label='Uncalibrated', linewidth=1.5, alpha=0.7)
    ax_y.plot(samples, calib_data['y'], color='deepskyblue', label='Calibrated', linewidth=1.5, alpha=0.9)
    ax_y.set_ylabel("Angular Velocity Y [dps]", fontweight='bold')
    ax_y.legend(loc='upper right')
    ax_y.grid(True, alpha=0.3)
    
    # --- Z Plot ---
    ax_z.plot(samples, raw_data['z'], color='mediumblue', label='Uncalibrated', linewidth=1.5, alpha=0.7)
    ax_z.plot(samples, calib_data['z'], color='deepskyblue', label='Calibrated', linewidth=1.5, alpha=0.9)
    ax_z.set_ylabel("Angular Velocity Z [dps]", fontweight='bold')
    ax_z.set_xlabel("Sample Number")
    ax_z.legend(loc='upper right')
    ax_z.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    # 1. Parse File
    raw = parse_log(LOG_FILE_NAME)
    
    if not raw['x']:
        print("No data found. Please check the 'LOG_FILE_NAME' variable.")
        sys.exit()

    # 2. Process Data
    calib = generate_calibrated_data(raw)
    
    # 3. Plot Comparison
    print("Generating plots...")
    plot_two_graphics(raw, calib)