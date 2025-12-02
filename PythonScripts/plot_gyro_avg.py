import serial
import time
import matplotlib.pyplot as plt
import sys

# ================= CONFIGURATION =================
COM_PORT = 'COM3'  # CHANGE THIS IF NEEDED
BAUD_RATE = 115200
TIMEOUT = 5
# =================================================

def parse_and_plot():
    data_store = {} 
    current_cycle = -1
    
    print(f"Connecting to {COM_PORT}...")
    
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)
        time.sleep(2) 
        
        print("Sending 'r' command to STM32...")
        ser.write(b'r') 
        
        print("Reading data stream...")
        
        started = False
        
        while True:
            try:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
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
                        x = int(parts[0])
                        y = int(parts[1])
                        z = int(parts[2])
                        
                        data_store[current_cycle]['x'].append(x)
                        data_store[current_cycle]['y'].append(y)
                        data_store[current_cycle]['z'].append(z)
                except ValueError:
                    pass 

        ser.close()
        
        # --- CALCULATION & PLOTTING LOGIC ---
        if not data_store:
            print("No valid data found to plot.")
            return

        print("\n=== CALCULATED AVERAGES ===")
        
        fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True, figsize=(10, 10))
        fig.suptitle('MO-2 Gyroscope Chamber Data (with Averages)')

        # Loop through every cycle found
        for cycle_id, axis_data in data_store.items():
            samples = range(len(axis_data['x']))
            
            # Calculate Averages
            avg_x = sum(axis_data['x']) / len(axis_data['x']) if axis_data['x'] else 0
            avg_y = sum(axis_data['y']) / len(axis_data['y']) if axis_data['y'] else 0
            avg_z = sum(axis_data['z']) / len(axis_data['z']) if axis_data['z'] else 0

            # Print to Console
            print(f"Cycle {cycle_id}:")
            print(f"  w_hat_x = {avg_x:.4f}")
            print(f"  w_hat_y = {avg_y:.4f}")
            print(f"  w_hat_z = {avg_z:.4f}")

            # FIX: Use double backslashes \\hat and \\omega to avoid syntax warnings
            
            # X Axis
            label_x = f'Cycle {cycle_id} ($\\hat{{\\omega}}_x$ = {avg_x:.2f})'
            ax1.plot(samples, axis_data['x'], label=label_x)
            
            # Y Axis
            label_y = f'Cycle {cycle_id} ($\\hat{{\\omega}}_y$ = {avg_y:.2f})'
            ax2.plot(samples, axis_data['y'], label=label_y)
            
            # Z Axis
            label_z = f'Cycle {cycle_id} ($\\hat{{\\omega}}_z$ = {avg_z:.2f})'
            ax3.plot(samples, axis_data['z'], label=label_z)

        # Formatting with raw strings r'' for axis labels
        ax1.set_ylabel(r'$\omega_x$ (Raw)')
        ax1.grid(True, alpha=0.5)
        ax1.legend(loc='upper right', fontsize='small')

        ax2.set_ylabel(r'$\omega_y$ (Raw)')
        ax2.grid(True, alpha=0.5)
        ax2.legend(loc='upper right', fontsize='small')

        ax3.set_ylabel(r'$\omega_z$ (Raw)')
        ax3.set_xlabel('Sample Number')
        ax3.grid(True, alpha=0.5)
        ax3.legend(loc='upper right', fontsize='small')

        plt.tight_layout()
        plt.show()

    except serial.SerialException:
        print(f"Error: Could not open {COM_PORT}. Is it plugged in? Close other terminals.")
    except ModuleNotFoundError:
        print("Error: Missing libraries. Run: pip install pyserial matplotlib")

if __name__ == "__main__":
    parse_and_plot()