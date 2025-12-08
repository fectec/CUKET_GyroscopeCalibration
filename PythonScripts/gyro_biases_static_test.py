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
    # Store data separated by cycle ID
    # structure: { cycle_id: {'x': [], 'y': [], 'z': []} }
    cycle_data = {1: {'x':[], 'y':[], 'z':[]}, 
                  2: {'x':[], 'y':[], 'z':[]}, 
                  3: {'x':[], 'y':[], 'z':[]}}
    
    current_cycle = -1
    
    # 1. Setup Log File
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"gyro_biases_static_test_{timestamp}.txt"
    print(f"Logging raw data to: {filename}")
    
    print(f"Connecting to {COM_PORT}...")
    
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)
        log_file = open(filename, "w")
        time.sleep(2) 
        
        # --- TRIGGER COMMAND ---
        print("Sending 'r' command...")
        ser.write(b'r') 
        
        print("Reading data (Waiting for Cycles 1, 2, and 3)...")
        
        started = False
        
        while True:
            try:
                raw_line = ser.readline()
                line = raw_line.decode('utf-8', errors='ignore').strip()
                
                if line:
                    log_file.write(line + "\n")
                    
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
                    print(f"-> Switching to Cycle: {current_cycle}")
                except:
                    pass
                continue

            if "END DATA" in line:
                print("End of data. Closing port.")
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
                        
                        # Save to specific cycle bucket
                        if current_cycle in cycle_data:
                            cycle_data[current_cycle]['x'].append(x_dps)
                            cycle_data[current_cycle]['y'].append(y_dps)
                            cycle_data[current_cycle]['z'].append(z_dps)
                            
                except ValueError:
                    pass 

        ser.close()
        log_file.close()
        
        # --- DATA PROCESSING & CALCULATIONS ---
        # Check if we have enough data
        if not cycle_data[1]['x'] or not cycle_data[2]['x'] or not cycle_data[3]['x']:
            print("Error: Missing data for one of the cycles (1, 2, or 3).")
            return

        print("\n=== CALCULATING BIASES ===")

        # --- Y AXIS CALCULATION ---
        # Cycle 3 -> bar_omega_y_up (Updated: +Y is in Cycle 3)
        vals_y_c3 = cycle_data[3]['y']
        bar_omega_y_up = sum(vals_y_c3) / len(vals_y_c3)

        # Cycle 2 -> bar_omega_y_down (Updated: -Y is in Cycle 2)
        vals_y_c2 = cycle_data[2]['y']
        bar_omega_y_down = sum(vals_y_c2) / len(vals_y_c2)

        b_y = (bar_omega_y_up + bar_omega_y_down) / 2.0

        # --- Z AXIS CALCULATION ---
        # Cycle 1 -> bar_omega_z_up
        vals_z_c1 = cycle_data[1]['z']
        bar_omega_z_up = sum(vals_z_c1) / len(vals_z_c1)

        # Cycle 2 -> bar_omega_z_down
        vals_z_c2 = cycle_data[2]['z']
        bar_omega_z_down = sum(vals_z_c2) / len(vals_z_c2)

        b_z = (bar_omega_z_up + bar_omega_z_down) / 2.0

        # --- X AXIS CALCULATION ---
        # Cycle 2 -> bar_omega_x_up
        vals_x_c2 = cycle_data[2]['x']
        bar_omega_x_up = sum(vals_x_c2) / len(vals_x_c2)

        # Cycle 3 -> bar_omega_x_down
        vals_x_c3 = cycle_data[3]['x']
        bar_omega_x_down = sum(vals_x_c3) / len(vals_x_c3)

        b_x = (bar_omega_x_up + bar_omega_x_down) / 2.0

        # Print Results to Console
        print(f"X-Axis: Up={bar_omega_x_up:.4f}, Down={bar_omega_x_down:.4f} => Bias b_x={b_x:.4f}")
        print(f"Y-Axis: Up={bar_omega_y_up:.4f}, Down={bar_omega_y_down:.4f} => Bias b_y={b_y:.4f}")
        print(f"Z-Axis: Up={bar_omega_z_up:.4f}, Down={bar_omega_z_down:.4f} => Bias b_z={b_z:.4f}")

        # --- SAVE RESULTS TO TOP OF FILE ---
        print(f"\nSaving summary to the top of {filename}...")
        try:
            # 1. Read the raw log data captured so far
            with open(filename, 'r') as f_read:
                raw_content = f_read.read()

            # 2. Construct the CSV Header and Data Block
            # Format: Axis, Omega_Up, Omega_Down, Bias
            csv_header = "Axis,Omega_Up_dps,Omega_Down_dps,Bias_dps\n"
            line_x = f"X,{bar_omega_x_up:.6f},{bar_omega_x_down:.6f},{b_x:.6f}\n"
            line_y = f"Y,{bar_omega_y_up:.6f},{bar_omega_y_down:.6f},{b_y:.6f}\n"
            line_z = f"Z,{bar_omega_z_up:.6f},{bar_omega_z_down:.6f},{b_z:.6f}\n"
            
            separator = "\n" + "="*50 + "\nRAW LOG DATA STARTS BELOW\n" + "="*50 + "\n\n"

            # 3. Write everything back: Summary + Separator + Raw Content
            with open(filename, 'w') as f_write:
                f_write.write(csv_header)
                f_write.write(line_x)
                f_write.write(line_y)
                f_write.write(line_z)
                f_write.write(separator)
                f_write.write(raw_content)
                
            print("Success! Summary saved.")

        except Exception as e:
            print(f"Error saving summary to file: {e}")

        # --- PLOTTING ---
        fig, (ax_x, ax_y, ax_z) = plt.subplots(3, 1, figsize=(12, 12))
        plt.subplots_adjust(hspace=0.4, right=0.7) # Make room for text boxes
        
        fig.suptitle('Gyroscope Bias (Static Test)', fontsize=16, fontweight='bold')

        # === GRAPH 1: X AXIS (Cycle 2 + Cycle 3) ===
        # Stitch data for plotting: UP part first (C2), then DOWN part (C3)
        x_plot_data = vals_x_c2 + vals_x_c3
        split_idx_x = len(vals_x_c2)
        
        ax_x.plot(range(split_idx_x), vals_x_c2, color='green', label='Up (Cycle 2)')
        ax_x.plot(range(split_idx_x, len(x_plot_data)), vals_x_c3, color='blue', label='Down (Cycle 3)')
        ax_x.axhline(b_x, color='black', linestyle='--', linewidth=2, label=f'Bias {b_x:.3f}')
        
        ax_x.set_ylabel('X Angular Velocity [dps]')
        ax_x.set_title('X-Axis Bias')
        ax_x.legend(loc='upper left', fontsize='small')
        ax_x.grid(True, alpha=0.3)

        # Text Box X
        textstr_x = '\n'.join((
            r'$\mathbf{X-Axis Results}$',
            r'$\bar{\omega}_{x^{up}} = %.5f$' % (bar_omega_x_up, ),
            r'$\bar{\omega}_{x^{down}} = %.5f$' % (bar_omega_x_down, ),
            r'----------------',
            r'$\mathbf{b_x = %.5f}$' % (b_x, )))
        props = dict(boxstyle='round', facecolor='white', alpha=1.0, edgecolor='gray')
        ax_x.text(1.02, 0.5, textstr_x, transform=ax_x.transAxes, fontsize=11, verticalalignment='center', bbox=props)

        # === GRAPH 2: Y AXIS (Cycle 3 + Cycle 2) ===
        # UP part (C3), DOWN part (C2)
        # Note: Cycle 3 is +Y (Up), Cycle 2 is -Y (Down)
        y_plot_data = vals_y_c3 + vals_y_c2
        split_idx_y = len(vals_y_c3)

        ax_y.plot(range(split_idx_y), vals_y_c3, color='green', label='Up (Cycle 3)')
        ax_y.plot(range(split_idx_y, len(y_plot_data)), vals_y_c2, color='blue', label='Down (Cycle 2)')
        ax_y.axhline(b_y, color='black', linestyle='--', linewidth=2, label=f'Bias {b_y:.3f}')

        ax_y.set_ylabel('Y Angular Velocity [dps]')
        ax_y.set_title('Y-Axis Bias')
        ax_y.legend(loc='upper left', fontsize='small')
        ax_y.grid(True, alpha=0.3)

        # Text Box Y
        textstr_y = '\n'.join((
            r'$\mathbf{Y-Axis Results}$',
            r'$\bar{\omega}_{y^{up}} = %.5f$' % (bar_omega_y_up, ),
            r'$\bar{\omega}_{y^{down}} = %.5f$' % (bar_omega_y_down, ),
            r'----------------',
            r'$\mathbf{b_y = %.5f}$' % (b_y, )))
        ax_y.text(1.02, 0.5, textstr_y, transform=ax_y.transAxes, fontsize=11, verticalalignment='center', bbox=props)

        # === GRAPH 3: Z AXIS (Cycle 1 + Cycle 2) ===
        # UP part (C1), DOWN part (C2)
        z_plot_data = vals_z_c1 + vals_z_c2
        split_idx_z = len(vals_z_c1)

        ax_z.plot(range(split_idx_z), vals_z_c1, color='green', label='Up (Cycle 1)')
        ax_z.plot(range(split_idx_z, len(z_plot_data)), vals_z_c2, color='blue', label='Down (Cycle 2)')
        ax_z.axhline(b_z, color='black', linestyle='--', linewidth=2, label=f'Bias {b_z:.3f}')

        ax_z.set_ylabel('Z Angular Velocity [dps]')
        ax_z.set_xlabel('Sample Count')
        ax_z.set_title('Z-Axis Bias')
        ax_z.legend(loc='upper left', fontsize='small')
        ax_z.grid(True, alpha=0.3)

        # Text Box Z
        textstr_z = '\n'.join((
            r'$\mathbf{Z-Axis Results}$',
            r'$\bar{\omega}_{z^{up}} = %.5f$' % (bar_omega_z_up, ),
            r'$\bar{\omega}_{z^{down}} = %.5f$' % (bar_omega_z_down, ),
            r'----------------',
            r'$\mathbf{b_z = %.5f}$' % (b_z, )))
        ax_z.text(1.02, 0.5, textstr_z, transform=ax_z.transAxes, fontsize=11, verticalalignment='center', bbox=props)

        plt.show()

    except serial.SerialException:
        print(f"Error: Could not open {COM_PORT}.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    parse_and_plot()