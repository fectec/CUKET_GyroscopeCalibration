<h1>Implementation of the MO-2 Gyroscope Verification System</h1>

<p align="justify">
Over a period of four months, hands-on research activities and laboratory projects were conducted at the Kyushu Institute of Technology (Kyutech) in Japan, along with a group of students from Instituto Tecnológico y de Estudios Superiores de Monterrey (ITESM) in México. 
Kyutech, recognized as one of the most distinguished traditional national universities of Japan and notable for its aerospace research, established the Laboratory of Lean Satellite Enterprises and In-Orbit Experiments (LaSEINE) to promote the utilization of nanosatellites, especially the CubeSat, based on the new concept of Lean Satellite [1].</p>

<p align="justify">
This repository documents the implementation of a gyroscope verification system for Kyutech's MO-2 satellite.
</p>

<h2>Background</h2>

<p align="justify">
Gyroscopes are inertial sensors used to measure the angular velocity of the platform to which they are attached [2].
</p>

<p align="justify">
For the MO-2 mission, one primary objective is to observe the satellite's condition and reflect status data—specifically angular velocity—into a game designed to inspire interest in space among a wider audience. The selected sensor for this task, chosen for its extensive flight heritage, is the L3G4200D. Selecting a sensor without a proven space record would necessitate complex and costly qualification procedures, such as radiation testing.
</p>

<p align="justify">
To ensure accurate data, a calibration procedure is performed prior to the mission to estimate deterministic error terms. While a zero-order calibration can simply estimate biases, more comprehensive approaches, such as the six-position method, estimate error terms for all three axes. This latter method typically involves rotating the gyroscope on a turntable at a known velocity to produce a signal strong enough for accurate calibration. This mechanical assistance is indispensable for low-cost sensors, as they often lack the sensitivity to detect the Earth's turn rate [2].
</p>

<p align="justify">
Consequently, the task assigned to the ITESM student group was to validate the functionality of the MO-2 gyroscope mounted on a Digilent Pmod verification board and to perform the calibration tests.
</p>

<p align="justify">
The scope included conducting controlled laboratory experiments: first, executing the six-position method on a rate table to generate calibration data, and second, subjecting the gyroscope to thermal chamber testing to evaluate its temperature-dependent behavior across the standard space operational range (-20°C to +80°C).
</p>

<p align="justify">
A significant technical challenge in these tests involves data retrieval and cabling. In the proposed setup, a NUCLEO-F446RET6 development board connects to the gyroscope to read data. If the board is placed off the rotation table, long jumper wires are required to reach the spinning sensor, creating a risk of disconnection.
</p>

<p align="justify">
Alternatively, placing the board on the table alongside the sensor requires a USB cable connecting to a PC, which inevitably tangles during rotation. A similar issue arises in the thermal test, which would require a sufficiently long USB cable to exit the chamber.
</p>

<p align="justify">
To resolve these issues, an MT25QL01GBBB8ESF-0SIT TR flash memory module, also with flight heritage, was proposed. This allows data to be saved locally and retrieved after the test, eliminating the need for a continuous physical connection to a PC during dynamic operations.
</p>

<p align="justify">
Furthermore, to facilitate a safe and repeatable testing procedure, a custom Printed Circuit Board (PCB) was designed. This integration combines all necessary components onto a single platform, ensuring stable electrical connections while simplifying the physical mounting of the hardware onto the rotation table and inside the thermal chamber.
</p>

<h2>Six-Position Calibration as Model-based Gyroscope Calibration</h2>

<p align="justify">
The six-position calibration method is a model-based approach executed before the mission starts to estimate the constant error terms of the gyroscope. This procedure requires rotating the gyroscope into six distinct orientations: twice per axis, once with the axis pointing in the positive direction and once in the negative direction. Each rotation is sustained for a duration <em>T</em>, allowing the measurements to be averaged to eliminate sensor noise. For a single axis, this calculation simplifies into a system of two equations with two unknowns: bias and scale factor [2].
</p>

<p align="justify">
For a single axis <em>i</em>, the calculation is expressed as:
</p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?s_i=\frac{\bar{\omega}_{i^+}-\bar{\omega}_{i^-}-2\omega_i}{2\omega_i}" alt="Scale Factor Equation" />
  <br><br>
  <img src="https://latex.codecogs.com/svg.latex?b_i=\frac{\bar{\omega}_{i^+}+\bar{\omega}_{i^-}}{2}" alt="Bias Equation" />

<p align="justify">
where <img src="https://latex.codecogs.com/svg.latex?\bar{\omega}_{i^+}" /> is the average gyroscope output for axis <em>i</em> while pointing in the positive direction (up), <img src="https://latex.codecogs.com/svg.latex?\bar{\omega}_{i^-}" /> is the average output for axis <em>i</em> while pointing in the negative direction (down), and <img src="https://latex.codecogs.com/svg.latex?\omega_i" /> is the ground truth (GT) angular velocity of the turntable. Solving the equations yields the gyroscope axis bias, <em>b<sub>i</sub></em>, and the scale factor, <em>s<sub>i</sub></em> [2].
</p>

<p align="justify">For this project, calibration is performed specifically for the z-axis. The formulas are adapted as follows:</p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?s_z=\frac{\bar{\omega}_{z^+}-\bar{\omega}_{z^-}-2\omega_z}{2\omega_z}" alt="Scale Factor Equation" />
  <br><br>
  <img src="https://latex.codecogs.com/svg.latex?b_z=\frac{\bar{\omega}_{z^+}+\bar{\omega}_{z^-}}{2}" alt="Bias Equation" />

<p align="justify"> The Gyroscope Error Model for this axis is defined as: </p>

<p align="center"> <img src="https://latex.codecogs.com/svg.latex?\hat{\omega}_{z}=(1+s_z)\cdot\omega_z+b_z" alt="Error Model" /> </p>

<p align="justify"> Where <img src="https://latex.codecogs.com/svg.latex?\hat{\omega}_{z}" /> represents the raw measured gyroscope angular velocity data, and <img src="https://latex.codecogs.com/svg.latex?\omega_z" /> represents the calibrated true gyroscope angular velocity data. </p>

<p align="justify"> To calibrate, we invert this equation. For every measurement <img src="https://latex.codecogs.com/svg.latex?\hat{\omega}_{z}" />, we use the calculated <em>b<sub>z</sub></em> and <em>s<sub>z</sub></em> to obtain the true angular velocity: </p>

<p align="center"> <img src="https://latex.codecogs.com/svg.latex?\omega_z=\frac{\hat{\omega}_{z}-b_z}{1+s_z}" alt="Calibration Formula" /> </p>

<p align="justify"> In this model, when the raw measurement <img src="https://latex.codecogs.com/svg.latex?\hat{\omega}_{z}"/> equals the average positive reading <img src="https://latex.codecogs.com/svg.latex?\bar{\omega}_{z^+}"/>, the result equals the positive Ground Truth (GT). Similarly, when the raw measurement equals <img src="https://latex.codecogs.com/svg.latex?\bar{\omega}_{z^-}"/>, the result equals the negative Ground Truth (-GT). All other values are fitted according to this linear relationship. </p>

<h2>Hardware Design</h2>

<p align="justify"> Detailed documentation regarding the hardware design is maintained in a dedicated repository. For access to the files required to reproduce the custom PCB, please visit the <a href="https://github.com/fectec/MO-2_GyroscopeShield.git">MO-2_GyroscopeShield</a> repository. </p>

<p align="justify"> For the steps below, it is assumed that all components have been soldered onto the PCB. This should be done following the silkscreen labels provided on the PCB surface. When referring to the "System," this implies the custom PCB mounted on top of the NUCLEO-F446RE board. </p>

<p align="justify"> There are two distinct options for powering the system: <strong>E5V</strong> (External 5V) or <strong>U5V</strong> (USB 5V). <strong>These modes cannot coexist; you must choose one.</strong> </p>

<h3>Option 1: E5V</h3> <p align="justify"> In this mode, the power subsystem uses an LM2596 step-down switching regulator. This component regulates the input voltage from a main battery pack (two Lithium-Ion cells) to the stable 5V required by the NUCLEO-F446RE board. </p> <ul> <li><strong>Switch:</strong> A switch is required to power the system remotely. Solder a cable to each terminal of the switch and secure the free ends to the <strong>J5</strong> terminal on the PCB. If a switch is not used, a jumper wire must be installed in J5 to close the circuit; otherwise, the system will not power on.</li> <li><strong>Battery:</strong> The Li-Ion batteries must be charged using an appropriate charger and placed in the battery holder. Connect the battery holder cables to the <strong>J4</strong> power terminal, strictly following the polarity markings on the PCB silkscreen.</li> </ul>

<p align="justify"> <strong>WARNING:</strong> Failure to respect the following order of operations may damage the NUCLEO-F446RE board or the PC. </p> <ol> <li>Connect a jumper between <strong>Pin 2 and Pin 3</strong> of header <strong>JP5</strong> on the NUCLEO-F446RE board.</li> <li>Ensure that jumper <strong>JP1</strong> on the NUCLEO-F446RE board is <strong>removed</strong>.</li> <li>Mount the custom PCB onto the NUCLEO-F446RE board.</li> <li>Connect the battery holder cables to the <strong>J4</strong> terminal on the PCB (observe polarity).</li> <li>Verify that the red <strong>LD3 LED</strong> on the NUCLEO-F446RE board turns on.</li> <li>Only after these steps, if code upload is required, connect the PC to the USB connector <strong>CN1</strong>.</li> </ol>

<h3>Option 2: U5V</h3> <p align="justify"> In this mode, the system is powered directly via the ST-LINK USB connector (CN1). You may use a PC or a portable power bank capable of supplying 5V and at least 300 mA. </p> <p align="justify"> <strong>Note:</strong> If using a power bank, the firmware must be uploaded to the board before connecting the power bank. If using a PC, the code can be uploaded while powered. </p>

<p align="justify"> <strong>WARNING:</strong> Failure to respect the following order of operations may damage the board. </p> <ol> <li>Connect a jumper between <strong>Pin 1 and Pin 2</strong> of header <strong>JP5</strong> on the NUCLEO-F446RE board (this differs from E5V mode).</li> <li>Ensure that jumper <strong>JP1</strong> on the NUCLEO-F446RE board is <strong>removed</strong>.</li> <li>Mount the custom PCB onto the NUCLEO-F446RE board.</li> <li>Connect the PC or Power Bank to the USB connector <strong>CN1</strong> on the NUCLEO-F446RE.</li> </ol>

<p align="justify"> <strong>Note:</strong> During the actual tests, using a battery (E5V) or power bank (U5V) is mandatory. A USB connection to a PC is not feasible due to cable tangling. </p>

<h2>Calibration Procedure</h2>

<p align="justify"> The custom PCB includes a push-button that controls the data logging process. Pressing the button initiates an uninterrupted gyroscope data logging cycle for a specific duration. Once the cycle finishes, another can be initiated.</p> <ul> <li><strong>Thermal Chamber Test:</strong> Each cycle corresponds to a specific temperature stability point. The button logic is mandatory here, as the time required for the chamber to stabilize varies and cannot be automated with a simple timer.</li> <li><strong>Rotary Table Test:</strong> Each cycle corresponds to a specific rotation.</li> </ul>

<p align="justify"> The code supporting this button functionality is found in the <code>button_logic</code> branch. </p>

<p align="justify"> <strong>Alternative for High-Speed Rotation:</strong> If the rotary table spins too fast to safely press the button, a timer-based routine can be used to log data automatically without physical interaction. This code is found in the <code>timer_logic_rotary</code> branch. </p>

<h3>Firmware Setup</h3>

<p align="justify"> If the NUCLEO-F446RE board has not been programmed, follow these instructions: </p>

<ol> <li> <p align="justify"> <strong>Clone the Repository:</strong> Open your terminal, navigate to your STM32CubeIDE workspace directory, and execute: </p> <pre><code>git clone https://github.com/fectec/MO-2_GyroscopeVerification.git</code></pre> </li>

<li> <p align="justify"> <strong>Import Project:</strong> In STM32CubeIDE, go to <strong>File > Open Projects from File System</strong>. Browse to the <code>MO-2_GyroscopeVerification</code> folder. Ensure <em>"Search for nested projects"</em> and <em>"Detect and configure project natures"</em> are checked, then click <strong>Finish</strong>. </p> </li>

<li> <p align="justify"> <strong>Flash Firmware:</strong> Connect the board to the PC via USB. Open <code>Core/Src/main.c</code>, then click the <strong>Run</strong> button (Play icon) to compile and upload. </p> </li> </ol>

<p align="justify"> To switch between the manual button logic and the automatic timer logic, change the active Git branch in your terminal. STM32CubeIDE will automatically update the files. </p>

<p align="justify"> <strong>For Button Logic (Standard):</strong> </p> <pre><code>git checkout button_logic</code></pre>

<p align="justify"> <strong>For Timer Logic (Automatic Rotary Table):</strong> </p> <pre><code>git checkout timer_logic_rotary</code></pre>

<p align="justify"> After checking out the desired branch, return to STM32CubeIDE and click <strong>Run</strong> to program the board. </p>

<h3>Rotary Table Test Procedure</h3>

<p align="justify"> The NUCLEO-F446RE logs raw gyroscope data to the flash memory. The calculation of calibration parameters is performed by a Python script on the PC. </p> 

<p align="justify"> <strong>Prerequisite:</strong> Download the script located at <code>MO-2_GyroscopeVerification/PythonScripts/rotary_gyro_data_retrieve.py</code>. This script is compatible with Windows. </p>

<p align="justify"> <strong>Orientation Definitions:</strong> </p> <ul> <li><strong>+Z (Up):</strong> The PCB LED is pointing toward the floor/table.</li> <li><strong>-Z (Down):</strong> The PCB LED is pointing toward the ceiling.</li> </ul>

<p align="justify"> <strong>Execution Steps (Button Logic):</strong> </p>

<ol> <li>In STM32CubeIDE, open <code>Core/Src/main.c</code> and define the log duration in milliseconds by modifying <code>#define LOG_DURATION_MS</code>. This sets how long the system records data per button press.</li> <li>In the Python script (<code>rotary_gyro_data_retrieve.py</code>), update the variable <code>TABLE_GROUND_TRUTH_DPS</code> to match the angular velocity (deg/s) you will set on the rotary table.</li> <li>Power the system using the Battery Pack (E5V) or Power Bank (U5V). If using E5V, turn on the switch. The PCB LED will start toggling (blinking), indicating it is in Idle mode.</li> <li>Place the system on the rotary table in the <strong>+Z orientation</strong>. Ensure no USB cables are connected to the PC.</li> <li>Configure the rotary table to spin Counter-Clockwise (CCW) at the velocity defined in step 2 and start the rotation.</li> <li>Press the button on the PCB to start logging. The LED will stop blinking and remain <strong>solid ON</strong>. When the cycle finishes, the LED will return to blinking. Stop the rotary table.</li> <li>Flip the system and place it on the rotary table in the <strong>-Z orientation</strong>.</li> <li>Start the rotary table spinning Counter-Clockwise (CCW) at the same velocity.</li> <li>Press the button to start the second log. The LED will turn solid ON. Wait for it to return to blinking, then stop the table.</li> <li><strong>Data Retrieval:</strong> <ul> <li>If using <strong>E5V</strong>: Do <strong>not</strong> turn off the switch or remove batteries. Connect the USB cable from the NUCLEO-F446RE to the PC.</li> <li>If using <strong>U5V</strong>: Disconnect the USB cable from the power bank and connect it to the PC.</li> </ul> </li> <li>Open Device Manager on Windows and identify the COM port assigned to <strong>STMicroelectronics STLink Virtual COM Port</strong>.</li> <li>Update the <code>COM_PORT</code> variable in the Python script with this value (e.g., 'COM3') and run the script.</li> <li>The script will generate a text file containing the logs and a plot displaying the Raw Sensor Data (deg/s), Calibrated Sensor Data (deg/s), the measured averages <img src="https://latex.codecogs.com/svg.latex?\bar{\omega}{z^+}" /> and <img src="https://latex.codecogs.com/svg.latex?\bar{\omega}{z^-}" />, and the calculated calibration parameters <img src="https://latex.codecogs.com/svg.latex?b_z" /> and <img src="https://latex.codecogs.com/svg.latex?s_z"/>.</li></ol></p>

<p align="center">
<img src="https://github.com/user-attachments/assets/e83f2797-a672-4e6e-af5c-ffd23b0de8ae" alt="Example Calibration Plot 5 dps" width="80%" />
</p>

<p align="center">
<img src="https://github.com/user-attachments/assets/27a8258c-fb3d-402c-8485-eae5781bb0fd" alt="Example Calibration Plot 25 dps" width="80%" />
</p>

</li>

<p align="justify"> <strong>IMPORTANT:</strong> Before starting a new test (a new pair of +Z/-Z rotations), you must erase the previous data from the flash memory. Connect the system to the PC, open a Serial Terminal (like PuTTY or the Arduino Serial Monitor) on the correct COM port, and send the character <strong>'e'</strong>. This clears the memory. Failure to do this will result in corrupted data when reading the new test. </p>
 
<h2>References</h2>

<ol>
  <li>
    <p align="justify">
      Kyushu Institute of Technology, "Kyutech Brochure 2024," Kitakyushu, Japan, 2024. [Online]. Available: <a href="https://www.kyutech.ac.jp//media/014/202404/brochure2024.pdf">https://www.kyutech.ac.jp//media/014/202404/brochure2024.pdf</a>
    </p>
  </li>
  <li>
    <p align="justify">
      Z. Yampolsky and I. Klein, "Data-Driven Gyroscope Calibration," <em>arXiv preprint arXiv:2410.12485</em>, 2024. [Online]. Available: <a href="https://arxiv.org/pdf/2410.12485">https://arxiv.org/pdf/2410.12485</a>
    </p>
  </li>
</ol>
