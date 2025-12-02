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
TABLE_GROUND_TRUTH_DPS = 25.0         

# =================================================

def parse_and_plot():
    # Store indices for plotting segments
    cycle_indices = {1: [], 2: []}
    
    # Store flat list for continuous plotting
    all_raw_dps = []
    
    current_cycle = -1
    global_idx_counter = 0
    
    # 1. Setup Log File
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"gyro_rotary_data_{timestamp}.txt"
    print(f"Logging raw data to: {filename}")
    
    print(f"Connecting to {COM_PORT}...")
    
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)
        log_file = open(filename, "w")
        time.sleep(2) 
        
        # --- TRIGGER COMMAND ---
        print("Sending 'r' command...")
        ser.write(b'r') 
        
        print("Reading data...")
        
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
                    if len(parts) == 3:
                        z_raw = int(parts[2]) # Only need Z
                        z_dps = z_raw * SENSITIVITY_250DPS
                        
                        # Save to continuous list
                        all_raw_dps.append(z_dps)
                        
                        # Save index to specific cycle bucket
                        if current_cycle in cycle_indices:
                            cycle_indices[current_cycle].append(global_idx_counter)
                        
                        global_idx_counter += 1
                            
                except ValueError:
                    pass 

        ser.close()
        log_file.close()
        
        # --- DATA PROCESSING ---
        if not cycle_indices[1] or not cycle_indices[2]:
            print("Error: Missing data for Cycle 1 (Up) or Cycle 2 (Down).")
            return

        print("\n=== CALIBRATION ===")
        
        # 1. Calculate Averages
        vals_c1 = [all_raw_dps[i] for i in cycle_indices[1]]
        vals_c2 = [all_raw_dps[i] for i in cycle_indices[2]]
        
        overline_w_z_pos = sum(vals_c1) / len(vals_c1) # Avg Cycle 1 (UP)
        overline_w_z_neg = sum(vals_c2) / len(vals_c2) # Avg Cycle 2 (DOWN)
        
        # 2. Apply Formulas
        w_z = TABLE_GROUND_TRUTH_DPS
        
        # Bias b_z
        b_z = (overline_w_z_pos + overline_w_z_neg) / 2.0
        
        # Scale Factor s_z
        s_z = ((overline_w_z_pos - overline_w_z_neg) - (2 * w_z)) / (2 * w_z)

        print(f"Avg Z-UP (Cycle 1): {overline_w_z_pos:.6f}")
        print(f"Avg Z-DOWN (Cycle 2): {overline_w_z_neg:.6f}")
        print(f"Bias (b_z): {b_z:.6f}")
        print(f"Scale Factor (s_z): {s_z:.6f}")

        # 3. CALCULATE CORRECTED DATASET
        # Formula: w_z = (w_z_hat - b_z) / (1 + s_z)
        scaling_term = 1.0 + s_z
        calibrated_z_dps = [(val - b_z) / scaling_term for val in all_raw_dps]

        # --- PLOTTING ---
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(14, 10), sharex=True)
        plt.subplots_adjust(hspace=0.3, right=0.65)
        
        fig.suptitle('Gyroscope Calibration Results', fontsize=16, fontweight='bold')

        # === PLOT 1: RAW DATA ===
        samples = range(len(all_raw_dps))
        ax1.plot(samples, all_raw_dps, color='lightgray', label='Raw Data', linewidth=1.5)
        
        # Plot Segments
        c1_start, c1_end = cycle_indices[1][0], cycle_indices[1][-1]
        ax1.plot([c1_start, c1_end], [overline_w_z_pos, overline_w_z_pos], color='green', linewidth=2.5, label=r'$\overline{\omega}_{z^+}$')
        
        c2_start, c2_end = cycle_indices[2][0], cycle_indices[2][-1]
        ax1.plot([c2_start, c2_end], [overline_w_z_neg, overline_w_z_neg], color='red', linewidth=2.5, label=r'$\overline{\omega}_{z^-}$')
        
        ax1.set_ylabel(r'Raw [deg/s]')
        ax1.set_title('1. Raw Sensor Data', fontsize=12, fontweight='bold')
        ax1.grid(True, alpha=0.4)
        ax1.legend(loc='upper right')

        # === TEXT BOX SIDEBAR ===
        textstr = '\n'.join((
            r'$\mathbf{Model:}$',
            r'$\hat{\omega}_{z} = (1 + s_z) \cdot \omega_z + b_z$',
            r'---------------------------',
            r'$\mathbf{Calibration\ Formulas:}$',
            r'$b_z = \frac{\overline{\omega}_{z^+} + \overline{\omega}_{z^-}}{2}$',
            r'$s_z = \frac{\overline{\omega}_{z^+} - \overline{\omega}_{z^-} - 2\omega_z}{2\omega_z}$',
            r'---------------------------',
            r'$\mathbf{Calibration\ Results:}$',
            r'$\omega_z = %.1f\ dps$' % (w_z, ),
            r'$\overline{\omega}_{z^+} = %.6f$' % (overline_w_z_pos, ),
            r'$\overline{\omega}_{z^-} = %.6f$' % (overline_w_z_neg, ),
            r'$b_z = %.6f\ dps$' % (b_z, ),
            r'$s_z = %.6f$' % (s_z, )))

        props = dict(boxstyle='round', facecolor='#f8f9fa', alpha=1.0, edgecolor='gray')
        
        ax1.text(1.05, 1.0, textstr, transform=ax1.transAxes, fontsize=12,
                verticalalignment='top', bbox=props)

        # === PLOT 2: CALIBRATED DATA ===
        ax2.plot(samples, calibrated_z_dps, label='Calibrated Data', color='blue', alpha=0.8)
        
        # Plot Target Segments
        ax2.plot([c1_start, c1_end], [w_z, w_z], color='green', linestyle='--', linewidth=2.5, label=r'Target $\omega_z = %.1f\ dps$' % w_z)
        ax2.plot([c2_start, c2_end], [-w_z, -w_z], color='red', linestyle='--', linewidth=2.5, label=r'Target $-\omega_z = -%.1f\ dps$' % w_z)
        
        ax2.set_ylabel(r'Calibrated [deg/s]')
        ax2.set_xlabel('Sample Number')
        
        ax2.set_title(r'2. Calibrated Sensor Data ($\omega_z = \frac{\hat{\omega}_{z} - b_z}{1 + s_z}$)', fontsize=12, fontweight='bold')
        
        ax2.grid(True, alpha=0.4)
        ax2.legend(loc='upper right')

        plt.show()

    except serial.SerialException:
        print(f"Error: Could not open {COM_PORT}.")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    parse_and_plot()