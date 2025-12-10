import matplotlib.pyplot as plt
import sys
import statistics
import os

# ================= USER CONFIGURATION =================

# REPLACE THIS with your actual log file name
LOG_FILE_NAME = "TEST1_THERMAL.txt"

# L3G4200D Sensitivity (250 dps range)
SENSITIVITY_250DPS = 0.00875043752

# ======================================================

def parse_and_plot_from_file(filename):
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found.")
        return

    print(f"Reading file: {filename}...")
    
    results = []
    raw_cycle_data = {}
    has_processed_header = False

    try:
        with open(filename, 'r') as f:
            lines = f.readlines()

        # 1. Check for existing Processed Header
        if len(lines) > 0 and "Temperature_C,Avg_X_dps" in lines[0]:
            print("Found processed data header. Reading directly...")
            has_processed_header = True
            
            for line in lines[1:]: # Skip header
                line = line.strip()
                if not line or "RAW LOG DATA" in line or "=" in line:
                    break
                
                parts = line.split(',')
                if len(parts) == 4:
                    try:
                        results.append({
                            'temp': float(parts[0]),
                            'avg_x': float(parts[1]),
                            'avg_y': float(parts[2]),
                            'avg_z': float(parts[3])
                        })
                    except ValueError:
                        continue
        
        # 2. If no header, parse Raw Data
        else:
            print("No processed header found. Parsing raw cycle data...")
            current_cycle = -1
            
            for line in lines:
                line = line.strip()
                if not line: continue

                # Parse Cycle ID
                if "CYCLE_ID" in line:
                    try:
                        parts = line.split(':')
                        if len(parts) > 1:
                            current_cycle = int(parts[1].strip())
                            if current_cycle not in raw_cycle_data:
                                raw_cycle_data[current_cycle] = {'x': [], 'y': [], 'z': []}
                    except:
                        pass
                    continue

                # Parse Data Lines
                if ',' in line and "Temp" not in line and "Axis" not in line:
                    try:
                        parts = line.split(',')
                        if len(parts) >= 3:
                            x_dps = int(parts[0]) * SENSITIVITY_250DPS
                            y_dps = int(parts[1]) * SENSITIVITY_250DPS
                            z_dps = int(parts[2]) * SENSITIVITY_250DPS
                            
                            if current_cycle != -1:
                                if current_cycle not in raw_cycle_data:
                                    raw_cycle_data[current_cycle] = {'x': [], 'y': [], 'z': []}
                                raw_cycle_data[current_cycle]['x'].append(x_dps)
                                raw_cycle_data[current_cycle]['y'].append(y_dps)
                                raw_cycle_data[current_cycle]['z'].append(z_dps)
                    except ValueError:
                        pass

            # If we parsed raw data, we need user input for temperatures
            if raw_cycle_data:
                results = process_raw_data(raw_cycle_data)
                if results:
                    save_processed_header(filename, results, lines)
            else:
                print("Error: No valid data found in file.")
                return

    except Exception as e:
        print(f"Error processing file: {e}")
        return

    # 3. Plotting
    if results:
        plot_data(results)
    else:
        print("No results to plot.")

def process_raw_data(cycle_data):
    """Asks user for temperature inputs for each detected cycle."""
    detected_cycles = sorted(cycle_data.keys())
    print(f"\nDetected {len(detected_cycles)} cycles: {detected_cycles}")
    print("Please enter the temperature (in °C) for each cycle:")

    results = []
    for c_id in detected_cycles:
        if not cycle_data[c_id]['x']:
            continue

        while True:
            try:
                temp_input = input(f"  Cycle {c_id} Temperature [°C]: ")
                temp_val = float(temp_input)
                
                avg_x = statistics.mean(cycle_data[c_id]['x'])
                avg_y = statistics.mean(cycle_data[c_id]['y'])
                avg_z = statistics.mean(cycle_data[c_id]['z'])
                
                results.append({
                    'temp': temp_val,
                    'avg_x': avg_x,
                    'avg_y': avg_y,
                    'avg_z': avg_z
                })
                break
            except ValueError:
                print("    Invalid number.")
    
    # Sort by temperature
    results.sort(key=lambda k: k['temp'])
    return results

def save_processed_header(filename, results, original_lines):
    """Prepends the calculated averages to the file."""
    print(f"\nUpdating {filename} with processed averages...")
    try:
        with open(filename, 'w') as f_write:
            f_write.write("Temperature_C,Avg_X_dps,Avg_Y_dps,Avg_Z_dps\n")
            for r in results:
                f_write.write(f"{r['temp']},{r['avg_x']:.6f},{r['avg_y']:.6f},{r['avg_z']:.6f}\n")
            f_write.write("\n" + "="*50 + "\n")
            f_write.write("RAW LOG DATA STARTS BELOW\n")
            f_write.write("="*50 + "\n\n")
            f_write.writelines(original_lines)
        print("File updated successfully.")
    except Exception as e:
        print(f"Error updating file: {e}")

def plot_data(results):
    temps = [r['temp'] for r in results]
    x_avgs = [r['avg_x'] for r in results]
    y_avgs = [r['avg_y'] for r in results]
    z_avgs = [r['avg_z'] for r in results]

    plt.figure(figsize=(10, 8))
    plt.scatter(x_avgs, temps, color='darkblue', s=80, label='X-axis', zorder=3)
    plt.scatter(y_avgs, temps, color='deepskyblue', s=80, label='Y-axis', zorder=3)
    plt.scatter(z_avgs, temps, color='purple', s=80, label='Z-axis', zorder=3)

    plt.title(f"Gyroscope Readings vs Temperature\n(Static Thermal Test)", fontsize=14, fontweight='bold')
    plt.ylabel("Temperature [°C]", fontsize=12, fontweight='bold')
    plt.xlabel("Angular Velocity [dps]", fontsize=12, fontweight='bold')
    plt.grid(True, linestyle='--', alpha=0.7, zorder=0)
    plt.legend(fontsize=12, shadow=True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parse_and_plot_from_file(LOG_FILE_NAME)