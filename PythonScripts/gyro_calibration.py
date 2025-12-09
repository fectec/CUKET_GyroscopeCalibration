import matplotlib.pyplot as plt
import sys
import os

# ================= USER CONFIGURATION =================

# REPLACE THIS with your actual file name
LOG_FILE_NAME = "LOG_FILE_NAME.txt"

# Calibration Variables
b_x = 0.0
b_y = 0.0
b_z = 0.0

s_x = 0.0
s_y = 0.0
s_z = 0.0

SENSITIVITY = 0.00875043752

# ======================================================

def parse_log_into_cycles(filename):
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found.")
        sys.exit(1)
        
    print(f"--- PARSING FILE: {filename} ---")
    
    cycles = []
    current_cycle = {'x': [], 'y': [], 'z': []}
    recording = False
    line_count = 0
    
    try:
        with open(filename, 'r') as f:
            lines = f.readlines()
            
        print(f"File contains {len(lines)} lines.")

        for line in lines:
            line_count += 1
            line = line.strip()
            
            # 1. Detect START
            if "START" in line:
                print(f"[Line {line_count}] Found START tag.")
                # If we were already recording, save the previous one first
                if recording and len(current_cycle['x']) > 0:
                    print(f"   -> Saving previous cycle (Count: {len(current_cycle['x'])} samples)")
                    cycles.append(current_cycle)
                    current_cycle = {'x': [], 'y': [], 'z': []}
                recording = True
                continue

            # 2. Detect CYCLE_ID (New Separator Logic)
            if "CYCLE" in line:
                print(f"[Line {line_count}] Found CYCLE tag: {line}")
                # If we have accumulated data, this implies a new cycle is starting
                if len(current_cycle['x']) > 0:
                     print(f"   -> Saving previous cycle (Count: {len(current_cycle['x'])} samples)")
                     cycles.append(current_cycle)
                     current_cycle = {'x': [], 'y': [], 'z': []}
                # Ensure recording is ON (in case START was missed)
                recording = True
                continue

            # 3. Detect END
            if "END" in line:
                print(f"[Line {line_count}] Found END tag.")
                if recording and len(current_cycle['x']) > 0:
                    print(f"   -> Saving cycle (Count: {len(current_cycle['x'])} samples)")
                    cycles.append(current_cycle)
                    current_cycle = {'x': [], 'y': [], 'z': []}
                recording = False
                continue

            # 4. Skip Headers/Metadata (Removed "CYCLE" from here)
            if "Axis" in line or "Temperature" in line:
                 print(f"[Line {line_count}] Skipping Metadata: {line}")
                 continue

            # 5. Parse Data
            if recording and ',' in line:
                parts = line.split(',')
                if len(parts) >= 3:
                    try:
                        # Parsing logic
                        rx = int(parts[0])
                        ry = int(parts[1])
                        rz = int(parts[2])
                        
                        current_cycle['x'].append(rx * SENSITIVITY)
                        current_cycle['y'].append(ry * SENSITIVITY)
                        current_cycle['z'].append(rz * SENSITIVITY)
                    except ValueError:
                        # Silently skip header lines or float lines (e.g., calibration summary)
                        pass
        
        # Catch trailing data (if file ends without END tag)
        if len(current_cycle['x']) > 0:
             print(f"[EOF] Saving final trailing cycle (Count: {len(current_cycle['x'])} samples)")
             cycles.append(current_cycle)

    except Exception as e:
        print(f"Error reading file: {e}")
        sys.exit(1)
                    
    print(f"--- FINISHED: Found {len(cycles)} valid cycles ---\n")
    return cycles

def apply_calibration(val, bias, scale_error):
    return (val - bias) / (1.0 + scale_error)

def process_cycles(cycles):
    calibrated_cycles = []
    for i, raw_data in enumerate(cycles):
        calib = {'x': [], 'y': [], 'z': []}
        calib['x'] = [apply_calibration(v, b_x, s_x) for v in raw_data['x']]
        calib['y'] = [apply_calibration(v, b_y, s_y) for v in raw_data['y']]
        calib['z'] = [apply_calibration(v, b_z, s_z) for v in raw_data['z']]
        calibrated_cycles.append(calib)
    return calibrated_cycles

def plot_cycles(raw_cycles, calib_cycles):
    if not raw_cycles:
        print("No valid data found to plot.")
        return

    for i in range(len(raw_cycles)):
        print(f"Preparing Plot for Cycle {i+1}...")
        raw_data = raw_cycles[i]
        calib_data = calib_cycles[i]
        samples = range(len(raw_data['x']))

        fig, (ax_x, ax_y, ax_z) = plt.subplots(3, 1, figsize=(10, 10))
        fig.canvas.manager.set_window_title(f'Cycle {i+1} - Calibration Verification')
        fig.suptitle(f"Cycle {i+1}: Calibration Verification\nFile: {LOG_FILE_NAME}", 
                     fontsize=14, fontweight='bold')

        # X Plot
        ax_x.plot(samples, raw_data['x'], color='mediumblue', label='Uncalibrated', alpha=0.6)
        ax_x.plot(samples, calib_data['x'], color='deepskyblue', label='Calibrated', alpha=0.9)
        ax_x.set_ylabel("X [dps]")
        ax_x.legend(loc='upper right')
        ax_x.set_title(f"X Axis (Bias: {b_x}, Scale: {s_x})", fontsize=10)

        # Y Plot
        ax_y.plot(samples, raw_data['y'], color='mediumblue', label='Uncalibrated', alpha=0.6)
        ax_y.plot(samples, calib_data['y'], color='deepskyblue', label='Calibrated', alpha=0.9)
        ax_y.set_ylabel("Y [dps]")
        ax_y.legend(loc='upper right')
        ax_y.set_title(f"Y Axis (Bias: {b_y}, Scale: {s_y})", fontsize=10)

        # Z Plot
        ax_z.plot(samples, raw_data['z'], color='mediumblue', label='Uncalibrated', alpha=0.6)
        ax_z.plot(samples, calib_data['z'], color='deepskyblue', label='Calibrated', alpha=0.9)
        ax_z.set_ylabel("Z [dps]")
        ax_z.set_xlabel("Sample Number")
        ax_z.legend(loc='upper right')
        ax_z.set_title(f"Z Axis (Bias: {b_z}, Scale: {s_z})", fontsize=10)

        plt.tight_layout()

    plt.show()

if __name__ == "__main__":
    raw_cycles = parse_log_into_cycles(LOG_FILE_NAME)
    calib_cycles = process_cycles(raw_cycles)
    plot_cycles(raw_cycles, calib_cycles)
