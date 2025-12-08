import serial
import time
import matplotlib.pyplot as plt
import sys
import statistics
from datetime import datetime

# ================= CONFIGURATION =================
COM_PORT = 'COM3'  # CHANGE THIS IF NEEDED
BAUD_RATE = 115200
TIMEOUT = 5

# L3G4200D Sensitivity (250 dps range)
SENSITIVITY_250DPS = 0.00875043752

# =================================================

def parse_and_plot():
    # Structure: { cycle_id: {'x': [], 'y': [], 'z': []} }
    cycle_data = {}
    current_cycle = -1
    
    # 1. Setup Log File
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"gyro_biases_static_thermal_test_{timestamp}.txt"
    print(f"Logging raw data to: {filename}")
    
    print(f"Connecting to {COM_PORT}...")
    print("Instructions: Run the thermal test cycles. When finished, the script will ask for temperatures.")
    
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)
        log_file = open(filename, "w")
        time.sleep(2) 
        
        # --- TRIGGER COMMAND ---
        print("Sending 'r' command to start reading...")
        ser.write(b'r') 
        
        print("Reading data... (Press Ctrl+C to force stop if needed, or wait for END DATA)")
        
        started = False
        
        while True:
            try:
                raw_line = ser.readline()
                line = raw_line.decode('utf-8', errors='ignore').strip()
                
                if line:
                    log_file.write(line + "\n")
                    # Optional: Print cycle updates to console so user knows it's working
                    if "CYCLE_ID" in line:
                        print(line)
            except KeyboardInterrupt:
                print("\nUser interrupted logging.")
                break
            except Exception:
                continue 

            if not line:
                continue 

            # --- PARSING ---
            if "START DATA" in line:
                started = True
                continue

            if "CYCLE_ID" in line:
                try:
                    parts = line.split(':')
                    current_cycle = int(parts[1])
                    if current_cycle not in cycle_data:
                        cycle_data[current_cycle] = {'x': [], 'y': [], 'z': []}
                except:
                    pass
                continue

            if "END DATA" in line:
                print("End of data command received. Closing port.")
                break

            if started and ',' in line:
                try:
                    parts = line.split(',')
                    if len(parts) >= 3:
                        # Parse Raw values
                        x_raw = int(parts[0])
                        y_raw = int(parts[1])
                        z_raw = int(parts[2])
                        
                        # Convert to DPS
                        x_dps = x_raw * SENSITIVITY_250DPS
                        y_dps = y_raw * SENSITIVITY_250DPS
                        z_dps = z_raw * SENSITIVITY_250DPS
                        
                        # Save to current cycle bucket
                        if current_cycle != -1:
                            # Initialize if missed header
                            if current_cycle not in cycle_data:
                                cycle_data[current_cycle] = {'x': [], 'y': [], 'z': []}
                            
                            cycle_data[current_cycle]['x'].append(x_dps)
                            cycle_data[current_cycle]['y'].append(y_dps)
                            cycle_data[current_cycle]['z'].append(z_dps)
                            
                except ValueError:
                    pass 

        ser.close()
        log_file.close()
        
        # --- USER INPUT: TEMPERATURE MAPPING ---
        detected_cycles = sorted(cycle_data.keys())
        if not detected_cycles:
            print("No cycles detected. Exiting.")
            return

        print(f"\nCaptured {len(detected_cycles)} cycles: {detected_cycles}")
        print("Please enter the temperature (in °C) for each cycle:")

        results = [] # List of dicts: {'temp': T, 'avg_x': val, 'avg_y': val, 'avg_z': val}

        for c_id in detected_cycles:
            while True:
                try:
                    temp_input = input(f"  Cycle {c_id} Temperature [°C]: ")
                    temp_val = float(temp_input)
                    
                    # Calculate Averages for this cycle
                    # Check if empty
                    if not cycle_data[c_id]['x']:
                        print(f"    Warning: Cycle {c_id} has no data. Skipping.")
                        break
                        
                    avg_x = statistics.mean(cycle_data[c_id]['x'])
                    avg_y = statistics.mean(cycle_data[c_id]['y'])
                    avg_z = statistics.mean(cycle_data[c_id]['z'])
                    
                    results.append({
                        'temp': temp_val,
                        'avg_x': avg_x,
                        'avg_y': avg_y,
                        'avg_z': avg_z,
                        'cycle': c_id
                    })
                    break
                except ValueError:
                    print("    Invalid number. Please enter a valid float (e.g., -30.5)")

        if not results:
            print("No valid data to plot.")
            return

        # --- SORTING ---
        # Sort by temperature (Low -> High)
        results.sort(key=lambda k: k['temp'])

        # --- SAVE PROCESSED DATA TO FILE (PREPEND) ---
        print(f"\nSaving processed averages to the top of {filename}...")
        try:
            # 1. Read existing raw logs
            with open(filename, 'r') as f_read:
                original_content = f_read.read()
            
            # 2. Rewrite file with CSV header + Data + Separator + Original Logs
            with open(filename, 'w') as f_write:
                # Header line for Excel/CSV tools
                f_write.write("Temperature_C,Avg_X_dps,Avg_Y_dps,Avg_Z_dps\n")
                
                # Write sorted data rows
                for r in results:
                    f_write.write(f"{r['temp']},{r['avg_x']:.6f},{r['avg_y']:.6f},{r['avg_z']:.6f}\n")
                
                # Separator
                f_write.write("\n" + "="*50 + "\n")
                f_write.write("RAW LOG DATA STARTS BELOW\n")
                f_write.write("="*50 + "\n\n")
                
                # Original raw data
                f_write.write(original_content)
                
            print("Success! File updated.")
            
        except Exception as e:
            print(f"Error updating file: {e}")

        # --- PLOTTING ---
        # Prepare lists for plotting
        temps = [r['temp'] for r in results]
        x_avgs = [r['avg_x'] for r in results]
        y_avgs = [r['avg_y'] for r in results]
        z_avgs = [r['avg_z'] for r in results]

        # Graph: Y-Axis = Temp, X-Axis = Angular Velocity
        plt.figure(figsize=(10, 8))
        
        # Plot X-Axis Data (Dark Blue)
        plt.scatter(x_avgs, temps, color='darkblue', s=80, label='X-axis', zorder=3)
        
        # Plot Y-Axis Data (Sky Blue)
        plt.scatter(y_avgs, temps, color='deepskyblue', s=80, label='Y-axis', zorder=3)
        
        # Plot Z-Axis Data (Purple)
        plt.scatter(z_avgs, temps, color='purple', s=80, label='Z-axis', zorder=3)

        plt.title(f"Gyroscope Bias vs Temperature\n(Static Thermal Test)", fontsize=14, fontweight='bold')
        plt.ylabel("Temperature [°C]", fontsize=12, fontweight='bold')
        plt.xlabel("Angular Velocity [dps]", fontsize=12, fontweight='bold')
        
        plt.grid(True, linestyle='--', alpha=0.7, zorder=0)
        plt.legend(fontsize=12, shadow=True)
        
        # Optional: Add text labels for exact values if points are sparse
        for r in results:
             t = r['temp']
             # Annotate X
             # plt.text(r['avg_x'], t, f"{r['avg_x']:.2f}", fontsize=8, va='center', ha='left')
        
        plt.tight_layout()
        plt.show()

    except serial.SerialException:
        print(f"Error: Could not open {COM_PORT}.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    parse_and_plot()