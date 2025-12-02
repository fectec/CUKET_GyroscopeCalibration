import serial
import time
import matplotlib.pyplot as plt
import sys
from datetime import datetime

# ================= CONFIGURATION =================
COM_PORT = 'COM3'  # CHANGE THIS IF NEEDED
BAUD_RATE = 115200
TIMEOUT = 5

# L3G4200D Sensitivity (250 dps range)
SENSITIVITY_250DPS = 0.00875043752

# =================================================

def parse_and_plot():
    data_store = {} 
    current_cycle = -1
    
    # 1. Setup Log File
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"gyro_thermal_data_{timestamp}.txt"
    print(f"Logging raw data to: {filename}")
    
    print(f"Connecting to {COM_PORT}...")
    
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)
        log_file = open(filename, "w")
        time.sleep(2) 
        
        # --- TRIGGER COMMAND ---
        print("Sending 'r' command to STM32...")
        ser.write(b'r') 
        
        print("Reading data stream...")
        
        started = False
        
        while True:
            try:
                raw_line = ser.readline()
                line = raw_line.decode('utf-8', errors='ignore').strip()
                
                # Save to file
                if line:
                    log_file.write(line + "\n")
                    
            except Exception:
                continue 

            if not line:
                continue 

            # --- PARSING LOGIC ---
            
            if "START DATA" in line:
                started = True
                continue

            if "CYCLE_ID" in line:
                try:
                    parts = line.split(':')
                    current_cycle = int(parts[1])
                    # Initialize lists for this cycle
                    data_store[current_cycle] = {'x': [], 'y': [], 'z': []}
                    print(f"-> Found New Cycle: {current_cycle}")
                except:
                    print("Error parsing Cycle ID")
                continue

            if "END DATA" in line:
                print("End of data detected. Closing port.")
                break

            if started and current_cycle != -1 and ',' in line:
                try:
                    parts = line.split(',')
                    if len(parts) == 3:
                        x_raw = int(parts[0])
                        y_raw = int(parts[1])
                        z_raw = int(parts[2])
                        
                        # Convert to dps immediately
                        data_store[current_cycle]['x'].append(x_raw * SENSITIVITY_250DPS)
                        data_store[current_cycle]['y'].append(y_raw * SENSITIVITY_250DPS)
                        data_store[current_cycle]['z'].append(z_raw * SENSITIVITY_250DPS)
                except ValueError:
                    pass 

        ser.close()
        log_file.close()
        
        # --- CALCULATION & PLOTTING LOGIC ---
        if not data_store:
            print("No valid data found to plot.")
            return

        print("\n=== PROCESSING THERMAL DATA ===")
        
        # Create 3 subplots sharing X axis
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(12, 12))
        fig.suptitle('Gyroscope Thermal Test', fontsize=16, fontweight='bold')

        # Loop through every cycle found
        for cycle_id, axis_data in data_store.items():
            samples = range(len(axis_data['x']))
            
            # Calculate Means (Bias approximation for this cycle)
            avg_x = sum(axis_data['x']) / len(axis_data['x']) if axis_data['x'] else 0
            avg_y = sum(axis_data['y']) / len(axis_data['y']) if axis_data['y'] else 0
            avg_z = sum(axis_data['z']) / len(axis_data['z']) if axis_data['z'] else 0

            print(f"Cycle {cycle_id} Averages -> X: {avg_x:.4f}, Y: {avg_y:.4f}, Z: {avg_z:.4f} dps")

            # --- PLOT X AXIS ---
            label_x = rf'Cycle {cycle_id} ($\overline{{\omega}}_x={avg_x:.2f}$)'
            ax1.plot(samples, axis_data['x'], label=label_x, linewidth=1.5, alpha=0.8)
            
            # --- PLOT Y AXIS ---
            label_y = rf'Cycle {cycle_id} ($\overline{{\omega}}_y={avg_y:.2f}$)'
            ax2.plot(samples, axis_data['y'], label=label_y, linewidth=1.5, alpha=0.8)
            
            # --- PLOT Z AXIS ---
            label_z = rf'Cycle {cycle_id} ($\overline{{\omega}}_z={avg_z:.2f}$)'
            ax3.plot(samples, axis_data['z'], label=label_z, linewidth=1.5, alpha=0.8)

        # --- FORMATTING ---
        # X-Axis Styling
        ax1.set_ylabel(r'$\hat{\omega}_x$ [deg/s]', fontsize=12)
        ax1.set_title('X-Axis Output', fontsize=10, fontweight='bold', loc='left')
        ax1.grid(True, alpha=0.4)
        ax1.legend(loc='upper right', fontsize='small', framealpha=0.9)

        # Y-Axis Styling
        ax2.set_ylabel(r'$\hat{\omega}_y$ [deg/s]', fontsize=12)
        ax2.set_title('Y-Axis Output', fontsize=10, fontweight='bold', loc='left')
        ax2.grid(True, alpha=0.4)
        ax2.legend(loc='upper right', fontsize='small', framealpha=0.9)

        # Z-Axis Styling
        ax3.set_ylabel(r'$\hat{\omega}_z$ [deg/s]', fontsize=12)
        ax3.set_title('Z-Axis Output', fontsize=10, fontweight='bold', loc='left')
        ax3.set_xlabel('Sample Number', fontsize=12)
        ax3.grid(True, alpha=0.4)
        ax3.legend(loc='upper right', fontsize='small', framealpha=0.9)

        plt.tight_layout()
        plt.show()

    except serial.SerialException:
        print(f"Error: Could not open {COM_PORT}. Is it plugged in?")
    except ModuleNotFoundError:
        print("Error: Missing libraries. Run: pip install pyserial matplotlib")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    parse_and_plot()