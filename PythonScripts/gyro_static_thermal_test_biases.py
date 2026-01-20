import matplotlib.pyplot as plt
import sys
import os

# ================= USER CONFIGURATION =================

# List the filenames of the thermal test logs
# UPDATE THIS LIST with your actual file names
LOG_FILES = [
    "TEST1_THERMAL.txt",
    "TEST2_THERMAL.txt",
    "TEST3_THERMAL.txt"
]

# ======================================================

def parse_summary_data(filename):
    """
    Parses the summary table at the top of the log file.
    Returns a dictionary: { temperature_float: {'x': val, 'y': val, 'z': val} }
    """
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found.")
        return None

    summary_data = {}
    reading_summary = False
    
    print(f"Parsing summary from: {filename}...")

    try:
        with open(filename, 'r') as f:
            for line in f:
                line = line.strip()
                
                # Detect start of summary table
                if "Temperature_C" in line and "Avg_X_dps" in line:
                    reading_summary = True
                    continue # Skip header line
                
                # Stop if we hit the separator or empty lines after data
                if reading_summary:
                    if not line or "=" in line or "RAW LOG DATA" in line:
                        break
                    
                    # Parse the CSV line
                    try:
                        parts = line.split(',')
                        if len(parts) >= 4:
                            temp = float(parts[0])
                            avg_x = float(parts[1])
                            avg_y = float(parts[2])
                            avg_z = float(parts[3])
                            
                            summary_data[temp] = {'x': avg_x, 'y': avg_y, 'z': avg_z}
                    except ValueError:
                        continue
                        
    except Exception as e:
        print(f"Error reading file {filename}: {e}")
        return None

    return summary_data

def main():
    # 1. Parse all files
    parsed_files = []
    for fname in LOG_FILES:
        data = parse_summary_data(fname)
        if data is None:
            return
        if not data:
            print(f"Warning: No summary data found in {fname}. Make sure the file has the 'Temperature_C...' header.")
            return
        parsed_files.append({'filename': fname, 'data': data})

    if len(parsed_files) != 3:
        print("Error: We need exactly 3 log files defined in LOG_FILES.")
        return

    # 2. Map Files to Positions
    print("\n" + "="*40)
    print("      POSITION MAPPING SELECTION")
    print("="*40)
    print("Please indicate which file corresponds to which Position.")
    print("  Position 1")
    print("  Position 2")
    print("  Position 3")
    print("-" * 40)
    
    for i, pfile in enumerate(parsed_files):
        print(f"[{i}] {pfile['filename']}")
    
    try:
        idx_p1 = int(input("\nEnter index for POSITION 1 file: "))
        idx_p2 = int(input("Enter index for POSITION 2 file: "))
        idx_p3 = int(input("Enter index for POSITION 3 file: "))
        
        # Verify indices are unique and valid
        if {idx_p1, idx_p2, idx_p3} != {0, 1, 2}:
            print("Error: Invalid selection. You must select unique indices 0, 1, and 2.")
            return

        pos1_data = parsed_files[idx_p1]['data']
        pos2_data = parsed_files[idx_p2]['data']
        pos3_data = parsed_files[idx_p3]['data']

    except ValueError:
        print("Error: Please enter valid numbers.")
        return

    # 3. Calculate Bias per Temperature
    # Find common temperatures
    temps_p1 = set(pos1_data.keys())
    temps_p2 = set(pos2_data.keys())
    temps_p3 = set(pos3_data.keys())
    
    common_temps = sorted(list(temps_p1.intersection(temps_p2).intersection(temps_p3)))
    
    if not common_temps:
        print("Error: No common temperatures found across the three files.")
        return

    print("\n" + "="*40)
    print("      CALCULATING BIASES")
    print("="*40)
    
    results = []

    for t in common_temps:
        # Retrieve averages for this temperature
        # Position 1
        z_pos = pos1_data[t]['z'] # +Z
        
        # Position 2
        x_pos = pos2_data[t]['x'] # +X
        y_neg = pos2_data[t]['y'] # -Y
        z_neg = pos2_data[t]['z'] # -Z
        
        # Position 3
        x_neg = pos3_data[t]['x'] # -X
        y_pos = pos3_data[t]['y'] # +Y
        
        # Calculate Biases using formulas
        # bx = (+X + -X) / 2
        b_x = (x_pos + x_neg) / 2.0
        
        # by = (+Y + -Y) / 2
        b_y = (y_pos + y_neg) / 2.0
        
        # bz = (+Z + -Z) / 2
        b_z = (z_pos + z_neg) / 2.0
        
        results.append({
            'temp': t,
            'b_x': b_x,
            'b_y': b_y,
            'b_z': b_z
        })
        
        print(f"Temp {t}°C: bx={b_x:.4f}, by={b_y:.4f}, bz={b_z:.4f}")

    # 4. Save Report
    out_filename = "gyro_static_thermal_test_biases.txt"
    try:
        with open(out_filename, 'w') as f:
            header = "Temperature_C,Bias_X_dps,Bias_Y_dps,Bias_Z_dps\n"
            f.write(header)
            for r in results:
                line = f"{r['temp']},{r['b_x']:.6f},{r['b_y']:.6f},{r['b_z']:.6f}\n"
                f.write(line)
            
            f.write("\n" + "="*50 + "\n")
            f.write("SOURCE MAPPING\n")
            f.write("="*50 + "\n")
            f.write(f"Position 1 (+Z): {parsed_files[idx_p1]['filename']}\n")
            f.write(f"Position 2 (+X,-Y,-Z): {parsed_files[idx_p2]['filename']}\n")
            f.write(f"Position 3 (-X,+Y): {parsed_files[idx_p3]['filename']}\n")

        print(f"\nSuccess! Final biases saved to: {out_filename}")
    except Exception as e:
        print(f"Error saving file: {e}")

    # 5. Plotting
    temps = [r['temp'] for r in results]
    bx = [r['b_x'] for r in results]
    by = [r['b_y'] for r in results]
    bz = [r['b_z'] for r in results]

    plt.figure(figsize=(10, 8))
    
    plt.plot(temps, bx, marker='o', color='darkblue', linestyle='-', linewidth=2, label='Bias X', zorder=3)
    plt.plot(temps, by, marker='s', color='deepskyblue', linestyle='-', linewidth=2, label='Bias Y', zorder=3)
    plt.plot(temps, bz, marker='^', color='purple', linestyle='-', linewidth=2, label='Bias Z', zorder=3)

    plt.title(f"Gyroscope Bias vs Temperature\n(Static Thermal Test)", fontsize=14, fontweight='bold')
    plt.xlabel("Temperature [°C]", fontsize=12, fontweight='bold')
    plt.ylabel("Bias [dps]", fontsize=12, fontweight='bold')
    
    plt.grid(True, linestyle='--', alpha=0.7, zorder=0)
    plt.legend(fontsize=12, shadow=True)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()