import serial
import time
import matplotlib.pyplot as plt
import numpy as np
import sys
from datetime import datetime
import os

# ================= CONFIGURATION =================
COM_PORT = 'COM3'  # UPDATE THIS TO YOUR PORT
BAUD_RATE = 115200
TIMEOUT = 300      # Long timeout for large data dumps

# L3G4200D Sensitivity (250 dps range)
SENSITIVITY_250DPS = 0.00875043752
FS = 100.0         # Sample Frequency in Hz
TS = 1.0 / FS      # Sample Period

# =================================================

def allan_deviation(data_arr, fs, max_num_m=200):
    """
    Computes Allan deviation (sigma) of time-series data.
    Obtained from: https://mwrona.com/posts/gyro-noise-analysis/
    """
    ts = 1.0 / fs
    N = len(data_arr)
    # Generate log-spaced averaging factors
    Mmax = 2**np.floor(np.log2(N / 2))
    M = np.logspace(np.log10(1), np.log10(Mmax), num=max_num_m)
    M = np.ceil(M)          # Round up to integer
    M = np.unique(M)        # Remove duplicates
    taus = M * ts           # Compute 'cluster durations' tau
    
    # Compute Allan variance
    allan_var = np.zeros(len(M))
    for i, mi in enumerate(M):
        two_mi = int(2 * mi)
        mi = int(mi)
        
        allan_var[i] = np.sum(
            (data_arr[two_mi:N] - (2.0 * data_arr[mi:N-mi]) + data_arr[0:N-two_mi])**2
        )
        allan_var[i] /= (2.0 * taus[i]**2) * (N - (2.0 * mi))

    return taus, np.sqrt(allan_var)     # Return deviation (dev = sqrt(var))

def find_valley(taus, adev):
    """
    Finds the 'valley' (Bias Instability point) by looking for the 
    first point where the slope changes from negative to positive.
    """
    # Calculate discrete difference (approx derivative)
    diffs = np.diff(adev)
    
    # Find indices where the derivative is positive (curve starts rising)
    rising_indices = np.where(diffs > 0)[0]
    
    if len(rising_indices) > 0:
        # The local minimum is the point just before it starts rising
        # We start checking from index 5 to avoid initial high-freq noise
        for idx in rising_indices:
            if idx > 5: 
                return taus[idx], adev[idx]
        
        # If all rising points were at the very start, return the first valid one
        return taus[rising_indices[0]], adev[rising_indices[0]]
    else:
        # Fallback: If curve is strictly monotonic decreasing (no valley found),
        # we take the last point (best guess given limited data)
        return taus[-1], adev[-1]

def calculate_noise_params(taus, adev):
    """
    Extracts ARW and Bias Instability using your specific formulas.
    """
    # --- 1. Angle Random Walk (ARW) ---
    # Formula: ARW = sigma(@1s) * 60  [deg/sqrt(hr)]
    sigma_1s = np.interp(1.0, taus, adev)
    arw_val = sigma_1s * 60.0
    
    # --- 2. Bias Instability (BI) ---
    # Formula: BI = (sigma_valley / 0.664) * 3600 [deg/hr]
    # Use the Valley Detection Method
    bi_tau, sigma_valley = find_valley(taus, adev)
    
    bias_instability_hr = (sigma_valley / 0.664) * 3600.0

    return arw_val, bias_instability_hr, (bi_tau, sigma_valley)

def parse_and_process():
    cycle_data = {'x': [], 'y': [], 'z': []}
    
    # 1. Setup Log File
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"gyro_allan_var_{timestamp}.txt"
    print(f"Logging raw data to: {filename}")
    
    # 2. Connect & Retrieve
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=TIMEOUT)
        log_file = open(filename, "w")
        time.sleep(2) 
        
        print("Sending 'r' command to retrieve data...")
        ser.write(b'r') 
        print("Reading data stream... (Press Ctrl+C if stuck)")
        
        started = False
        while True:
            try:
                raw_line = ser.readline()
                line = raw_line.decode('utf-8', errors='ignore').strip()
                
                if line:
                    log_file.write(line + "\n")
                    if "END DATA" in line:
                        print("End of data received.")
                        break
                    if "START DATA" in line:
                        started = True
                        continue

                    if started and ',' in line:
                        parts = line.split(',')
                        if len(parts) >= 3:
                            # Convert to DPS immediately
                            x = int(parts[0]) * SENSITIVITY_250DPS
                            y = int(parts[1]) * SENSITIVITY_250DPS
                            z = int(parts[2]) * SENSITIVITY_250DPS
                            
                            cycle_data['x'].append(x)
                            cycle_data['y'].append(y)
                            cycle_data['z'].append(z)
            except KeyboardInterrupt:
                break
            except Exception:
                continue

        ser.close()
        log_file.close()
        
    except Exception as e:
        print(f"Serial Error: {e}")
        return

    # 3. Process Data
    print("\nProcessing Allan Deviation...")
    gx = np.array(cycle_data['x'])
    gy = np.array(cycle_data['y'])
    gz = np.array(cycle_data['z'])
    
    if len(gx) < 1000:
        print("Error: Not enough data for meaningful analysis.")
        return

    # Integrate Rate [deg/s] -> Angle [deg]
    theta_x = np.cumsum(gx) * TS
    theta_y = np.cumsum(gy) * TS
    theta_z = np.cumsum(gz) * TS
    
    # Compute AD
    print("Computing AD for X...")
    tau_x, ad_x = allan_deviation(theta_x, FS)
    print("Computing AD for Y...")
    tau_y, ad_y = allan_deviation(theta_y, FS)
    print("Computing AD for Z...")
    tau_z, ad_z = allan_deviation(theta_z, FS)
    
    # Calculate Params
    arw_x, bi_x, p_bi_x = calculate_noise_params(tau_x, ad_x)
    arw_y, bi_y, p_bi_y = calculate_noise_params(tau_y, ad_y)
    arw_z, bi_z, p_bi_z = calculate_noise_params(tau_z, ad_z)
    
    # Print Table
    print("\n" + "="*70)
    print("           GYROSCOPE NOISE CHARACTERIZATION")
    print("="*70)
    print(f"{'Axis':<5} | {'ARW':<25} | {'Bias Instability':<25}")
    print(f"{'':<5} | {'[deg/sqrt(hr)]':<25} | {'[deg/hr]':<25}")
    print("-" * 70)
    print(f"{'X':<5} | {arw_x:<25.5f} | {bi_x:<25.5f}")
    print(f"{'Y':<5} | {arw_y:<25.5f} | {bi_y:<25.5f}")
    print(f"{'Z':<5} | {arw_z:<25.5f} | {bi_z:<25.5f}")
    print("="*70)

    # 4. Plotting
    plt.figure(figsize=(10, 8))
    
    plt.loglog(tau_x, ad_x, 'r-', label='X Axis')
    plt.loglog(tau_y, ad_y, 'g-', label='Y Axis')
    plt.loglog(tau_z, ad_z, 'b-', label='Z Axis')
    
    # Plot -0.5 Slope Reference (Gaussian White Noise)
    # Just for visual reference, anchored to X-axis
    ref_sigma_1 = arw_x / 60.0 # Convert back to deg/s for plotting
    plt.plot([0.1, 10], [ref_sigma_1 * np.sqrt(1/0.1), ref_sigma_1 * np.sqrt(1/10)], 'k--', alpha=0.5, label='Slope -0.5 (Gaussian White Noise)')
    
    # Highlight ARW Points (@ tau=1)
    sigma_1_x = arw_x / 60.0
    sigma_1_y = arw_y / 60.0
    sigma_1_z = arw_z / 60.0
    plt.plot(1.0, sigma_1_x, 'ro', markersize=6)
    plt.plot(1.0, sigma_1_y, 'go', markersize=6)
    plt.plot(1.0, sigma_1_z, 'bo', markersize=6)
    
    # Highlight Bias Instability Points (Triangles at the Valley)
    plt.plot(p_bi_x[0], p_bi_x[1], 'r^', markersize=10, markeredgecolor='k', zorder=5)
    plt.plot(p_bi_y[0], p_bi_y[1], 'g^', markersize=10, markeredgecolor='k', zorder=5)
    plt.plot(p_bi_z[0], p_bi_z[1], 'b^', markersize=10, markeredgecolor='k', zorder=5)

    plt.title(f'Gyro Allan Deviation\n', fontsize=14, fontweight='bold')
    plt.xlabel(r'$\tau$ [sec]', fontsize=12)
    plt.ylabel(r'Allan Deviation $\sigma(\tau)$ [deg/s]', fontsize=12)
    plt.grid(True, which="both", ls="-", alpha=0.4)
    plt.legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parse_and_process()
