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

# Calibration Settings
TABLE_GROUND_TRUTH_DPS = 60.0  # UPDATE THIS TO MATCH YOUR ROTARY TABLE VELOCITY

# =================================================

def parse_and_plot():
    # Store data separated by cycle ID
    # We expect 6 cycles in total based on the user manual steps
    cycle_data = {
        1: [], # Z CW
        2: [], # Z CCW
        3: [], # Y CW
        4: [], # Y CCW
        5: [], # X CW
        6: []  # X CCW
    }
    
    current_cycle = -1
    
    # 1. Setup Log File
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"gyro_dynamic_test_{timestamp}.txt"
    print(f"Logging raw data to: {filename}")
    print(f"Target Angular Velocity: {TABLE_GROUND_TRUTH_DPS} dps")
    
    print(f"Connecting to {COM_PORT}...")
    
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)
        log_file = open(filename, "w")
        time.sleep(2) 
        
        # --- TRIGGER COMMAND ---
        print("Sending 'r' command...")
        ser.write(b'r') 
        
        print("Reading data (Waiting for Cycles 1 through 6)...")
        print("Sequence expected: Z(CW), Z(CCW), Y(CW), Y(CCW), X(CW), X(CCW)")
        
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
                        
                        # Logic to grab specific axis based on Cycle ID
                        if current_cycle in [1, 2]:
                            # Z Axis Cycles
                            cycle_data[current_cycle].append(z_dps)
                        elif current_cycle in [3, 4]:
                            # Y Axis Cycles
                            cycle_data[current_cycle].append(y_dps)
                        elif current_cycle in [5, 6]:
                            # X Axis Cycles
                            cycle_data[current_cycle].append(x_dps)
                            
                except ValueError:
                    pass 

        ser.close()
        log_file.close()
        
        # --- DATA PROCESSING & CALCULATIONS ---
        # Verify we have data for all cycles
        missing_cycles = [k for k, v in cycle_data.items() if not v]
        if missing_cycles:
            print(f"Error: Missing data for cycles: {missing_cycles}")
            return

        print("\n=== CALCULATING SCALE FACTOR ERRORS ===")
        
        # Reference denominator (Total Span = 2 * omega_ref)
        denominator = 2.0 * TABLE_GROUND_TRUTH_DPS

        # --- Z AXIS (Cycles 1 & 2) ---
        vals_z_cw = cycle_data[1]
        vals_z_ccw = cycle_data[2]
        bar_w_z_cw = sum(vals_z_cw) / len(vals_z_cw)
        bar_w_z_ccw = sum(vals_z_ccw) / len(vals_z_ccw)
        
        # Scale Factor S_z
        # Since CW is negative and CCW is positive, Total Range = CCW - CW
        s_z = ((bar_w_z_ccw - bar_w_z_cw) / denominator) - 1.0

        # --- Y AXIS (Cycles 3 & 4) ---
        vals_y_cw = cycle_data[3]
        vals_y_ccw = cycle_data[4]
        bar_w_y_cw = sum(vals_y_cw) / len(vals_y_cw)
        bar_w_y_ccw = sum(vals_y_ccw) / len(vals_y_ccw)
        
        # Scale Factor S_y
        s_y = ((bar_w_y_ccw - bar_w_y_cw) / denominator) - 1.0

        # --- X AXIS (Cycles 5 & 6) ---
        vals_x_cw = cycle_data[5]
        vals_x_ccw = cycle_data[6]
        bar_w_x_cw = sum(vals_x_cw) / len(vals_x_cw)
        bar_w_x_ccw = sum(vals_x_ccw) / len(vals_x_ccw)
        
        # Scale Factor S_x
        s_x = ((bar_w_x_ccw - bar_w_x_cw) / denominator) - 1.0

        # Print Results
        print(f"Z-Axis: CW={bar_w_z_cw:.2f}, CCW={bar_w_z_ccw:.2f} -> s_z={s_z:.5f}")
        print(f"Y-Axis: CW={bar_w_y_cw:.2f}, CCW={bar_w_y_ccw:.2f} -> s_y={s_y:.5f}")
        print(f"X-Axis: CW={bar_w_x_cw:.2f}, CCW={bar_w_x_ccw:.2f} -> s_x={s_x:.5f}")

        # --- SAVE RESULTS TO TOP OF FILE ---
        print(f"\nSaving summary to the top of {filename}...")
        try:
            # 1. Read the raw log data captured so far
            with open(filename, 'r') as f_read:
                raw_content = f_read.read()

            # 2. Construct the CSV Header and Data Block
            # Format: Axis, Avg_Omega_CW, Avg_Omega_CCW, Scale_Factor_Error, Ground_Truth_DPS
            csv_header = "Axis,Avg_Omega_CW,Avg_Omega_CCW,Scale_Factor_Error,Ground_Truth_DPS\n"
            line_x = f"X,{bar_w_x_cw:.6f},{bar_w_x_ccw:.6f},{s_x:.6f},{TABLE_GROUND_TRUTH_DPS}\n"
            line_y = f"Y,{bar_w_y_cw:.6f},{bar_w_y_ccw:.6f},{s_y:.6f},{TABLE_GROUND_TRUTH_DPS}\n"
            line_z = f"Z,{bar_w_z_cw:.6f},{bar_w_z_ccw:.6f},{s_z:.6f},{TABLE_GROUND_TRUTH_DPS}\n"
            
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
        fig, (ax_z, ax_y, ax_x) = plt.subplots(3, 1, figsize=(12, 14))
        plt.subplots_adjust(hspace=0.4, right=0.7) 
        
        fig.suptitle(f'Gyroscope Scale Factor Errors (Target: {TABLE_GROUND_TRUTH_DPS} dps)', fontsize=16, fontweight='bold')

        # === GRAPH 1: Z AXIS (Cycle 1 + 2) ===
        z_plot_data = vals_z_cw + vals_z_ccw
        split_idx_z = len(vals_z_cw)
        
        ax_z.plot(range(split_idx_z), vals_z_cw, color='green', label='CW (Cycle 1)')
        ax_z.plot(range(split_idx_z, len(z_plot_data)), vals_z_ccw, color='blue', label='CCW (Cycle 2)')
        
        # Plot Averages (Segmented Lines)
        # CW Line (0 to split_idx_z)
        ax_z.plot([0, split_idx_z - 1], [bar_w_z_cw, bar_w_z_cw], color='lawngreen', linestyle='-', linewidth=2, label='Avg CW')
        # CCW Line (split_idx_z to end)
        ax_z.plot([split_idx_z, len(z_plot_data) - 1], [bar_w_z_ccw, bar_w_z_ccw], color='deepskyblue', linestyle='-', linewidth=2, label='Avg CCW')

        # Add targets for visual reference
        target_sign_z = 1 if bar_w_z_cw > 0 else -1
        ax_z.axhline(target_sign_z * TABLE_GROUND_TRUTH_DPS, color='gray', linestyle='--', alpha=0.5, label='Target Ref')
        ax_z.axhline(-target_sign_z * TABLE_GROUND_TRUTH_DPS, color='gray', linestyle='--', alpha=0.5)

        ax_z.set_ylabel('Z Angular Velocity [dps]')
        ax_z.set_title('Z-Axis Scale Factor')
        ax_z.legend(loc='upper left', fontsize='small')
        ax_z.grid(True, alpha=0.3)

        textstr_z = '\n'.join((
            r'$\mathbf{Z-Axis Results}$',
            r'$\bar{\omega}_{z^{cw}} = %.2f$' % (bar_w_z_cw, ),
            r'$\bar{\omega}_{z^{ccw}} = %.2f$' % (bar_w_z_ccw, ),
            r'----------------',
            r'$\mathbf{s_z = %.5f}$' % (s_z, )))
        
        props = dict(boxstyle='round', facecolor='white', alpha=1.0, edgecolor='gray')
        ax_z.text(1.02, 0.5, textstr_z, transform=ax_z.transAxes, fontsize=11, verticalalignment='center', bbox=props)

        # === GRAPH 2: Y AXIS (Cycle 3 + 4) ===
        y_plot_data = vals_y_cw + vals_y_ccw
        split_idx_y = len(vals_y_cw)
        
        ax_y.plot(range(split_idx_y), vals_y_cw, color='green', label='CW (Cycle 3)')
        ax_y.plot(range(split_idx_y, len(y_plot_data)), vals_y_ccw, color='blue', label='CCW (Cycle 4)')
        
        # Plot Averages (Segmented Lines)
        # CW Line
        ax_y.plot([0, split_idx_y - 1], [bar_w_y_cw, bar_w_y_cw], color='lawngreen', linestyle='-', linewidth=2, label='Avg CW')
        # CCW Line
        ax_y.plot([split_idx_y, len(y_plot_data) - 1], [bar_w_y_ccw, bar_w_y_ccw], color='deepskyblue', linestyle='-', linewidth=2, label='Avg CCW')

        target_sign_y = 1 if bar_w_y_cw > 0 else -1
        ax_y.axhline(target_sign_y * TABLE_GROUND_TRUTH_DPS, color='gray', linestyle='--', alpha=0.5, label='Target Ref')
        ax_y.axhline(-target_sign_y * TABLE_GROUND_TRUTH_DPS, color='gray', linestyle='--', alpha=0.5)

        ax_y.set_ylabel('Y Angular Velocity [dps]')
        ax_y.set_title('Y-Axis Scale Factor')
        ax_y.legend(loc='upper left', fontsize='small')
        ax_y.grid(True, alpha=0.3)

        textstr_y = '\n'.join((
            r'$\mathbf{Y-Axis Results}$',
            r'$\bar{\omega}_{y^{cw}} = %.2f$' % (bar_w_y_cw, ),
            r'$\bar{\omega}_{y^{ccw}} = %.2f$' % (bar_w_y_ccw, ),
            r'----------------',
            r'$\mathbf{s_y = %.5f}$' % (s_y, )))
        ax_y.text(1.02, 0.5, textstr_y, transform=ax_y.transAxes, fontsize=11, verticalalignment='center', bbox=props)

        # === GRAPH 3: X AXIS (Cycle 5 + 6) ===
        x_plot_data = vals_x_cw + vals_x_ccw
        split_idx_x = len(vals_x_cw)
        
        ax_x.plot(range(split_idx_x), vals_x_cw, color='green', label='CW (Cycle 5)')
        ax_x.plot(range(split_idx_x, len(x_plot_data)), vals_x_ccw, color='blue', label='CCW (Cycle 6)')
        
        # Plot Averages (Segmented Lines)
        # CW Line
        ax_x.plot([0, split_idx_x - 1], [bar_w_x_cw, bar_w_x_cw], color='lawngreen', linestyle='-', linewidth=2, label='Avg CW')
        # CCW Line
        ax_x.plot([split_idx_x, len(x_plot_data) - 1], [bar_w_x_ccw, bar_w_x_ccw], color='deepskyblue', linestyle='-', linewidth=2, label='Avg CCW')

        target_sign_x = 1 if bar_w_x_cw > 0 else -1
        ax_x.axhline(target_sign_x * TABLE_GROUND_TRUTH_DPS, color='gray', linestyle='--', alpha=0.5, label='Target Ref')
        ax_x.axhline(-target_sign_x * TABLE_GROUND_TRUTH_DPS, color='gray', linestyle='--', alpha=0.5)

        ax_x.set_ylabel('X Angular Velocity [dps]')
        ax_x.set_xlabel('Sample Count')
        ax_x.set_title('X-Axis Scale Factor')
        ax_x.legend(loc='upper left', fontsize='small')
        ax_x.grid(True, alpha=0.3)

        textstr_x = '\n'.join((
            r'$\mathbf{X-Axis Results}$',
            r'$\bar{\omega}_{x^{cw}} = %.2f$' % (bar_w_x_cw, ),
            r'$\bar{\omega}_{x^{ccw}} = %.2f$' % (bar_w_x_ccw, ),
            r'----------------',
            r'$\mathbf{s_x = %.5f}$' % (s_x, )))
        ax_x.text(1.02, 0.5, textstr_x, transform=ax_x.transAxes, fontsize=11, verticalalignment='center', bbox=props)

        plt.show()

    except serial.SerialException:
        print(f"Error: Could not open {COM_PORT}.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    parse_and_plot()