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

<li>
    <p align="justify">
      <strong>Power Up & Cabling:</strong> Since standard Li-Ion batteries and power banks cannot safely operate under the extreme temperature ranges of the thermal chamber, the system must be powered using an external bench power supply via the <strong>E5V</strong> input. Place the system inside the chamber and route both the power supply cables and the extended logging control button cable through the chamber's side access hole, ensuring the controls and power source remain outside. Once the cables are routed, strictly seal the access hole with the provided plug (<em>"tapón"</em>) to ensure thermal isolation. Connect the power cables to the external supply set to <strong>5V</strong>, turn it on, and verify that the PCB LED is toggling (blinking), which indicates the system is in Idle mode. Ensure no USB cables are connected.
    </p>
  </li>

<p align="justify">
MEMS gyroscopes are widely adopted in CubeSat missions due to their compact size, low power consumption, cost-effectiveness, and precision. However, their accuracy tends to degrade over time as a result of  combined errors, including noise, biases, drift, and scale factor instability. If left uncorrected, these deterministic errors accumulate, leading to progressively larger discrepancies in position and orientation estimates, a phenomenon well-documented in previous missions utilizing MEMS sensors for Attitude Determination and Control Systems (ADCS) [3].
</p>

<p align="justify">
Inertial sensor errors are classified into random and deterministic categories. Deterministic errors, such as biases and scale factors, must be corrected through calibration. Furthermore, because these errors in MEMS sensors are highly sensitive to temperature fluctuations, the calibration model must explicitly account for thermal dependencies to prevent significant accuracy degradation over the mission duration [3].
</p>

<p align="justify">
To ensure accurate data, a calibration procedure is performed prior to the mission to estimate deterministic error terms. While a zero-order calibration can simply estimate biases, more comprehensive approaches, such as the six-position method, estimate error terms for all three axes. This latter method typically involves rotating the gyroscope on a rotary table at a known velocity to produce a signal strong enough for accurate calibration. This mechanical assistance is indispensable for low-cost sensors, as they often lack the sensitivity to detect the Earth's turn rate [2].
</p>

<p align="justify">
Consequently, the task assigned to the ITESM student group was to validate the functionality of the MO-2 L3G4200D gyroscope mounted on a Digilent Pmod verification board and to perform the calibration tests.
</p>

<p align="justify">
The scope included conducting controlled laboratory experiments: first, executing the six-position method to generate calibration data, and second, subjecting the gyroscope to thermal chamber testing to evaluate its temperature-dependent behavior across the standard space operational range.
</p>

<p align="justify">
A significant technical challenge in these tests involves data retrieval and cabling. In the proposed setup, a NUCLEO-F446RET6 development board connects to the gyroscope to read data. If the board is placed off the rotary table, long jumper wires are required to reach the spinning sensor, creating a risk of disconnection.
</p>

<p align="justify">
Alternatively, placing the board on the table alongside the sensor requires a USB cable connecting to a PC, which inevitably tangles during rotation. A similar issue arises in the thermal test, which would require a sufficiently long USB cable to exit the chamber.
</p>

<p align="justify">
To resolve these issues, an MT25QL01GBBB8ESF-0SIT TR flash memory module, also with flight heritage, was proposed. This allows data to be saved locally and retrieved after the test, eliminating the need for a continuous physical connection to a PC during dynamic operations.
</p>

<p align="justify">
Furthermore, to facilitate a safe and repeatable testing procedure, a custom Printed Circuit Board (PCB) was designed. This integration combines all necessary components onto a single platform, ensuring stable electrical connections while simplifying the physical mounting of the hardware onto the rotary table and inside the thermal chamber.
</p>

<h2>Gyroscope Error Model</h2>

<p align="justify">
An ideal MEMS gyroscope is characterized by the absence of noise or offset and perfect linearity—meaning it produces a strictly proportional and predictable output for any given rotation. However, real-world sensors are subject to several deterministic errors [3]:
</p>

<ul>
<li><p align="justify"><strong>Bias (Offset):</strong> The deviation of the gyroscope output from the expected theoretical value when the device is stationary. This "zero reading" tends to drift over time due to the integration of inherent device imperfections and internal noise [3].</p></li>
<li><p align="justify"><strong>Scale Factor Error:</strong> A metric describing the deviation of the sensor's sensitivity from unity. It quantifies the discrepancy between the sensor's measured output range and the actual input rotation range [3].</p></li>
<li><p align="justify"><strong>Non-orthogonalities (Misalignment):</strong> The error resulting from imperfect alignment of the gyroscope's sensing axes relative to an ideal mutually orthogonal coordinate system [3].</p></li>
</ul>

<p align="justify">
The output of a typical MEMS gyroscope can be modeled as a function of the true input angular velocity and the sensor's deterministic error variables. This relationship is expressed as [3]:
</p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?\mathbf{\hat{\omega}}=\mathbf{K}\mathbf{\omega}+\mathbf{b}\quad(1)" alt="Error Model (1)" />
</p>

<p align="justify">
Where <img src="https://latex.codecogs.com/svg.latex?\mathbf{\hat{\omega}}" /> represents the angular velocity recorded by the gyroscope (measured), and <img src="https://latex.codecogs.com/svg.latex?\mathbf{\omega}" /> represents the true input angular velocity. <img src="https://latex.codecogs.com/svg.latex?\mathbf{b}" /> represents the bias vector. <img src="https://latex.codecogs.com/svg.latex?\mathbf{K}" /> is the matrix accounting for non-orthogonality and scale factors [3]:
</p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?\mathbf{K}=\begin{bmatrix}m_{xx}&m_{xy}&m_{xz}\\m_{yx}&m_{yy}&m_{yz}\\m_{zx}&m_{zy}&m_{zz}\end{bmatrix}\quad(2)" alt="Matrix K (2)" />
</p>

<p align="justify">In this matrix, the diagonal elements <img src="https://latex.codecogs.com/svg.latex?(m_{xx},\;m_{yy},\;m_{zz})">
represent the scale factors, while the off-diagonal elements <img src="https://latex.codecogs.com/svg.latex?(m_{ij})">
 represent non-orthogonality (misalignment) errors [3].</p>

<p align="justify">
 Expanding Equation (1) into matrix form yields [3]:
</p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?\begin{bmatrix}\hat{\omega}_x\\\hat{\omega}_y\\\hat{\omega}_z\end{bmatrix}=\begin{bmatrix}m_{xx}&m_{xy}&m_{xz}\\m_{yx}&m_{yy}&m_{yz}\\m_{zx}&m_{zy}&m_{zz}\end{bmatrix}\begin{bmatrix}\omega_x\\\omega_y\\\omega_z\end{bmatrix}+\begin{bmatrix}b_x\\b_y\\b_z\end{bmatrix}\quad(3)" alt="Expanded Matrix Equation (3)" />
</p>

<p align="justify">Since misalignment angles are small in low-cost gyroscopes, only scale factors and bias errors are typically considered. The reduced model is therefore [2]:</p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?%5Cbegin%7Bbmatrix%7D%5Chat%7B%5Comega%7D_x%5C%5C%5Chat%7B%5Comega%7D_y%5C%5C%5Chat%7B%5Comega%7D_z%5Cend%7Bbmatrix%7D%3D%5Cbegin%7Bbmatrix%7D1%2Bs_x%260%260%5C%5C0%261%2Bs_y%260%5C%5C0%260%261%2Bs_z%5Cend%7Bbmatrix%7D%5Cbegin%7Bbmatrix%7D%5Comega_x%5C%5C%5Comega_y%5C%5C%5Comega_z%5Cend%7Bbmatrix%7D%2B%5Cbegin%7Bbmatrix%7Db_x%5C%5Cb_y%5C%5Cb_z%5Cend%7Bbmatrix%7D\quad(4)"
       alt="Reduced Model (4)" />
</p>

<p align="justify"> In order to solve Equation (4), a combination of static and dynamic tests using a rotary table will be performed [3]. This procedure is explained below.<p>

<h2>Six-Position Calibration as Model-based Gyroscope Calibration</h2>

<h3>Gyroscope Biases - Static Test</h3>

<p align="justify">
To determine the bias for each axis, the orthogonal gyroscope triad is positioned on a leveled surface. Each sensitive axis is oriented alternately in the upward and downward directions, resulting in a total of six distinct measurement positions. The bias is then calculated as [3]:
</p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?b_i=\frac{\bar{\omega}_{i^{up}}+\bar{\omega}_{i^{down}}}{2}\quad(5)" alt="Bias Calculation (5)" />
</p>

<p align="justify">
Where <img src="https://latex.codecogs.com/svg.latex?b_i" /> is the bias for each axis, and <img src="https://latex.codecogs.com/svg.latex?\bar{\omega}_{i^{up}}" /> and <img src="https://latex.codecogs.com/svg.latex?\bar{\omega}_{i^{down}}" /> represent the measured angular rates recorded by the gyroscope when pointing upward and downward, respectively [3].
</p>

<h3> Gyroscope Scale Factor Errors - Dynamic Test</h3>

<p align="justify">
The scale factor errors are determined using a procedure similar to the bias calculation, but this time a rotary table is employed to spin the gyroscope triad both clockwise and counter-clockwise for each sensitive axis (six different measurements) [3]. The scale factor error for a given axis <em>i</em> is calculated as [2]:
</p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?s_i=\frac{\bar{\omega}_{i^{cw}}+\bar{\omega}_{i^{ccw}}}{2\omega_{ref}}-1\quad(6)" alt="Scale Error Factor Calculation (6)" />
</p>

<p align="justify">
Where <img src="https://latex.codecogs.com/svg.latex?s_i" /> is the scale factor error for the pertinent axis, <img src="https://latex.codecogs.com/svg.latex?\bar{\omega}_{i^{cw}}" /> and <img src="https://latex.codecogs.com/svg.latex?\bar{\omega}_{i^{ccw}}" /> represent the angular rates measured by the gyroscope when spinning clockwise and counter-clockwise respectively, and <img src="https://latex.codecogs.com/svg.latex?\omega_{ref}" /> is the known reference angular velocity of the rotary table or ground truth (GT) [3].
</p>

<h4>Note</h4>

<p align="justify">
For both bias and scale factor error, the bar notation (<img src="https://latex.codecogs.com/svg.latex?\bar{\omega}" />) indicates that each measurement is sustained for a duration <em>T</em>, allowing the measurements to be averaged to eliminate sensor noise.
</p>

<h3>Gyroscope Calibration</h3>

<p align="justify"> To calibrate the sensor and recover the true angular velocity vector <img src="https://latex.codecogs.com/svg.latex?\mathbf{\omega}" />, the error model in Equation (1) must be inverted.</p>

<p align="center"> <img src="https://latex.codecogs.com/svg.latex?\mathbf{\omega}=\mathbf{K}^{-1}(\mathbf{\hat{\omega}}-\mathbf{b})\quad(7)" alt="Inverse Error Model (7)" /> </p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?%5Cbegin%7Bbmatrix%7D%5Comega_x%5C%5C%5Comega_y%5C%5C%5Comega_z%5Cend%7Bbmatrix%7D%3D%5Cbegin%7Bbmatrix%7D1%2Bs_x%260%260%5C%5C0%261%2Bs_y%260%5C%5C0%260%261%2Bs_z%5Cend%7Bbmatrix%7D%5E%7B-1%7D%5Cleft%28%5Cbegin%7Bbmatrix%7D%5Chat%7B%5Comega%7D_x%5C%5C%5Chat%7B%5Comega%7D_y%5C%5C%5Chat%7B%5Comega%7D_z%5Cend%7Bbmatrix%7D-%5Cbegin%7Bbmatrix%7Db_x%5C%5Cb_y%5C%5Cb_z%5Cend%7Bbmatrix%7D%5Cright%29%5Cquad%288%29"
       alt="Inverse Error Matrix Model (8)" />
</p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?%5Comega_x%3D%5Cfrac%7B%5Chat%7B%5Comega%7D_x-b_x%7D%7B1%2Bs_x%7D%2C%5Cquad%5Comega_y%3D%5Cfrac%7B%5Chat%7B%5Comega%7D_y-b_y%7D%7B1%2Bs_y%7D%2C%5Cquad%5Comega_z%3D%5Cfrac%7B%5Chat%7B%5Comega%7D_z-b_z%7D%7B1%2Bs_z%7D%5Cquad%289%29"
       alt="Scalar Calibration Formulas (9)" />
</p>

<h2>Thermal Calibration</h2>

<p align="justify">
Thermal calibration is performed to model the temperature dependencies of the gyroscope's deterministic errors: bias and scale factor. Two primary methods are employed for this characterization [4]:

</p>

<ul>
<li><p align="justify"><strong>Thermal Soak Method:</strong> The MEMS sensor is placed inside a thermal chamber and allowed to stabilize at a series of discrete temperature setpoints. Data collection is initiated only after the sensor has reached thermal equilibrium at each target temperature. By recording measurements and calculating sensor errors at these stable points, a dataset of error-versus-temperature values is generated. Errors at intermediate temperatures can then be estimated using interpolation techniques [4].</p></li>
<li><p align="justify"><strong>Thermal Ramp Method:</strong> In this approach, the sensor is polled continuously while the thermal chamber temperature is linearly increased or decreased across the desired operating range. While this method is inherently faster because it eliminates the time-consuming stabilization periods, it introduces two significant sources of error. First, a thermal gradient often exists between the inertial sensor core and the temperature sensor, leading to measurement discrepancies similar to hysteresis. Second, because the temperature evolves dynamically during the data acquisition window for a single calibration scheme, the resulting error parameters are derived from data collected at varying temperatures rather than a single thermal point [4].</p></li>
</ul>

<p align="justify">
Although time-intensive, the soak method ensures the most reliable error characterization [4]; therefore, it has been selected for this verification.
</p>

<h2>Hardware Design</h2>

<p align="justify"> Detailed documentation regarding the hardware design is maintained in a dedicated repository. For access to the files required to reproduce the custom PCB, please visit the <a href="https://github.com/fectec/MO-2_GyroscopeShield.git">MO-2_GyroscopeShield</a> repository. </p>

<p align="justify"> For the steps below, it is assumed that all components have been soldered onto the PCB. This should be done following the silkscreen labels provided on the PCB surface. When referring to the "System," this implies the custom PCB mounted on top of the NUCLEO-F446RE board. </p>

<p align="justify"> There are two distinct options for powering the system: <strong>E5V</strong> (External 5V) or <strong>U5V</strong> (USB 5V). <strong>These modes cannot coexist; you must choose one.</strong> </p>

<h3>Option 1: E5V</h3> <p align="justify"> In this mode, the power subsystem uses an LM2596 step-down switching regulator. This component regulates the input voltage from a main battery pack (two Lithium-Ion cells) to the stable 5V required by the NUCLEO-F446RE board. </p> <ul><li><strong>Switch:</strong> A switch is required to power the system remotely. Solder a cable to each terminal of the switch and secure the free ends to the <strong>J5</strong> terminal on the PCB. If a switch is not used, a jumper wire must be installed in J5 to close the circuit; otherwise, the system will not power on.</li> <li><strong>Battery:</strong> The Li-Ion batteries must be charged using an appropriate charger and placed in the battery holder. Connect the battery holder cables to the <strong>J4</strong> power terminal, strictly following the polarity markings on the PCB silkscreen.</li></ul>

<p align="justify"> <strong>WARNING:</strong> Failure to respect the following order of operations may damage the NUCLEO-F446RE board or the PC. </p> <ol><li>Connect a jumper between <strong>Pin 2 and Pin 3</strong> of header <strong>JP5</strong> on the NUCLEO-F446RE board.</li> <li>Ensure that jumper <strong>JP1</strong> on the NUCLEO-F446RE board is <strong>removed</strong>.</li> <li>Mount the custom PCB onto the NUCLEO-F446RE board.</li> <li>Connect the battery holder cables to the <strong>J4</strong> terminal on the PCB (observe polarity).</li> <li>Verify that the red <strong>LD3 LED</strong> on the NUCLEO-F446RE board turns on.</li> <li>Only after these steps, if code upload is required, connect the PC to the USB connector <strong>CN1</strong>.</li></ol>

<h3>Option 2: U5V</h3> <p align="justify"> In this mode, the system is powered directly via the ST-LINK USB connector (CN1). You may use a PC or a portable power bank capable of supplying 5V and at least 300 mA. </p> <p align="justify"> <strong>Note:</strong> If using a power bank, the firmware must be uploaded to the board before connecting the power bank. If using a PC, the code can be uploaded while powered. </p>

<p align="justify"> <strong>WARNING:</strong> Failure to respect the following order of operations may damage the board. </p> <ol><li>Connect a jumper between <strong>Pin 1 and Pin 2</strong> of header <strong>JP5</strong> on the NUCLEO-F446RE board (this differs from E5V mode).</li> <li>Ensure that jumper <strong>JP1</strong> on the NUCLEO-F446RE board is <strong>removed</strong>.</li> <li>Mount the custom PCB onto the NUCLEO-F446RE board.</li> <li>Connect the PC or Power Bank to the USB connector <strong>CN1</strong> on the NUCLEO-F446RE.</li></ol>

<p align="justify"> <strong>Note:</strong> During the actual tests, using a battery (E5V) or power bank (U5V) is mandatory. A USB connection to a PC is not feasible due to the cable. </p>

<h2>Calibration Procedure</h2>

<p align="justify"> The custom PCB includes a push-button that controls the data logging process. Pressing the button initiates an uninterrupted gyroscope data logging cycle for a specific duration <em>T</em>. Once the cycle finishes, another can be initiated.</p>
<ol type="1">
<li><p align="justify"><strong>Gyroscope Biases - Static Test:</strong> Each cycle corresponds to a specific position.</p></li>
<li><p align="justify"><strong>Gyroscope Scale Factor Errors - Dynamic Rotary Test:</strong> Each cycle corresponds to a specific rotation.</p></li>
<li><p align="justify"><strong>Gyroscope Biases - Static Thermal Test:</strong> Each cycle corresponds to a specific temperature stability point. The button logic is mandatory here, as the time required for the chamber to stabilize varies and cannot be automated with a simple timer. </p></li>
</ol>

<p align="justify"> The code supporting this button functionality is found in the <code>button_logic</code> branch. </p>

<h3>Firmware Setup</h3>

<p align="justify"> If the NUCLEO-F446RE board has not been programmed, follow these instructions: </p>

<ol type="1">
<li><p align="justify"> <strong>Clone the Repository:</strong> Open your terminal, navigate to your STM32CubeIDE workspace directory, and execute: </p> <pre><code>git clone https://github.com/fectec/MO-2_GyroscopeVerification.git</code></pre></li>
<li><p align="justify"> <strong>Import Project:</strong> In STM32CubeIDE, go to <strong>File > Open Projects from File System</strong>. Browse to the <code>MO-2_GyroscopeVerification</code> folder. Ensure <em>"Search for nested projects"</em> and <em>"Detect and configure project natures"</em> are checked, then click <strong>Finish</strong>. </p></li>
<li><p align="justify"> <strong>Flash Firmware:</strong> Connect the board to the PC via USB. Open <code>Core/Src/main.c</code>, then click the <strong>Run</strong> button (Play icon) to compile and upload. </p></li>
</ol>

<p align="justify"> <strong>IMPORTANT:</strong> Before starting a new test, you must erase the previous data from the flash memory. Connect the system to the PC, open a Serial Terminal (like PuTTY or the Arduino Serial Monitor) on the correct COM port, and send the character <strong>'e'</strong>. This clears the memory. Failure to do this will result in corrupted data when reading the new test. </p>

<h3>Gyroscope Biases - Static Test Procedure</h3>

<p align="justify">
The NUCLEO-F446RE logs raw gyroscope data to the flash memory. The calculation of the biases is performed by a Python script on the PC.
</p>

<p align="justify">
<strong>Prerequisite:</strong> Download the script located at <code>MO-2_GyroscopeVerification/PythonScripts/gyro_static_test.py</code>. This script is compatible with Windows.
</p>

<p align="justify"><strong>Execution Steps:</strong></p>

<ol type="1">
<li><p align="justify"><strong>Configure Firmware:</strong> In STM32CubeIDE, open <code>Core/Src/main.c</code> and modify <code>#define LOG_DURATION_MS</code> to set the log duration <em>T</em> (in milliseconds) for each button press.</p></li>
<li><p align="justify"><strong>Power Up:</strong> Connect the Battery Pack (E5V) or Power Bank (U5V). If using E5V, turn on the switch. Verify the PCB LED is toggling (blinking), indicating Idle mode. Ensure no USB cables are connected.</p></li>
<li><p align="justify"><strong>Perform Data Logging:</strong> Perform the logging routine for each of the required positions. <strong>CRITICAL: You must perform the tests in the exact order shown (Position 1 &rarr; Position 2 &rarr; Position 3).</strong> For every position listed below:</p>
<ul>
<li>Place the system on a leveled surface matching the reference image.</li>
<li>Press the button to start logging. The PCB LED will turn <strong>Solid ON</strong>.</li>
<li>Wait for the PCB LED to return to <strong>Blinking</strong> (cycle complete) before moving to the next position.</li>
</ul>
<p align="center">
<strong>Position 1</strong><br>
<img src="https://github.com/user-attachments/assets/ce7d5c47-3d76-469f-8495-888691b38281" alt="Position 1 Alignment"><br><br>
</p>
<p align="center">
<strong>Position 2</strong><br>
<img src="https://github.com/user-attachments/assets/689e6140-b162-4c93-ad7c-e60c2f0bd7d3" alt="Position 2 Alignment"><br><br>
</p>
<p align="center">
<strong>Position 3</strong><br>
<img src="https://github.com/user-attachments/assets/dc66da5c-dbad-4d4e-9aac-d4749e14ad1c" alt="Position 3 Alignment">
</p>
</li>
<li><p align="justify"><strong>Connect to PC:</strong></p>
<ul>
<li>If using <strong>E5V</strong>: Keep the batteries connected and switch ON. Connect the USB cable to the PC.</li>
<li>If using <strong>U5V</strong>: Disconnect the power bank and connect the USB cable to the PC.</li>
</ul>
</li>
<li><p align="justify"><strong>Identify Port:</strong> Open Windows Device Manager and find the COM port number for <strong>STMicroelectronics STLink Virtual COM Port</strong>.</p></li>
<li><p align="justify"><strong>Run Analysis:</strong> Update the <code>COM_PORT</code> variable in the Python script and run it.</p></li>
<li><p align="justify"><strong>Results:</strong> The script generates a text file containing the logs and a summary of the results, as well as a plot displaying the averaged angular rates (<img src="https://math.vercel.app?from=\bar{\omega}_{i^{up}}" /> and <img src="https://math.vercel.app?from=\bar{\omega}_{i^{down}}" />) per axis, along with the calculated biases.</p></li>
</ol>

<img width="2559" height="1415" alt="Results" src="https://github.com/user-attachments/assets/ae4e64fc-f1f5-419a-8647-0b2123fcd3b8" />

<h3>Gyroscope Scale Factor Errors - Dynamic Rotary Test:</h3>

<p align="justify">
The NUCLEO-F446RE logs raw gyroscope data to the flash memory. The calculation of the scale factor errors is performed by a Python script on the PC.
</p>

<p align="justify">
<strong>Prerequisite:</strong> Download the script located at <code>MO-2_GyroscopeVerification/PythonScripts/gyro_dynamic_test.py</code>. This script is compatible with Windows.
</p>

<p align="justify"><strong>Execution Steps:</strong></p>

<ol type="1">
<li><p align="justify"><strong>Configure Firmware:</strong> In STM32CubeIDE, open <code>Core/Src/main.c</code> and modify <code>#define LOG_DURATION_MS</code> to set the log duration <em>T</em> (in milliseconds) for each button press.</p></li>
<li><strong>Configure Script:</strong> In the Python script, update the variable <code>TABLE_GROUND_TRUTH_DPS</code> to match the angular velocity you will set on the rotary table.</li>
<li><p align="justify"><strong>Power Up:</strong> Connect the Battery Pack (E5V) or Power Bank (U5V). If using E5V, turn on the switch. Verify the PCB LED is toggling (blinking), indicating Idle mode. Ensure no USB cables are connected.</p></li>
<li><p align="justify"><strong>Perform Data Logging:</strong> Execute the logging sequence for the three positions shown below. <strong>CRITICAL: You must perform the tests in the exact order shown (Position 1 (CW &rarr; CCW) &rarr; Position 2 (CW &rarr; CCW) &rarr; Position 3 (CW &rarr; CCW)).</strong></p>
<p align="center">
<strong>Position 1</strong><br>
<img src="https://github.com/user-attachments/assets/3f993f3c-cc5a-4cf2-a2cc-20ae1826a5ea" alt="Position 1 Alignment"><br><br>
</p>
<p align="center">
<strong>Position 2</strong><br>
<img src="https://github.com/user-attachments/assets/80379b77-40c4-4bce-9b40-664e33ace2be" alt="Position 2 Alignment"><br><br>
</p>
<p align="center">
<strong>Position 3</strong><br>
<img src="https://github.com/user-attachments/assets/a56a6f36-c45e-4cf2-ae19-bbd03feb259b" alt="Position 3 Alignment">
</p>
<p align="justify">For <strong>each</strong> position, repeat the following steps:</p>
<ul>
<li>Place the system on the rotary table as shown in the corresponding image.</li>
<li><strong>Clockwise (CW):</strong> Configure the table to spin CW at the defined velocity and start rotation. Press the button to log (LED solid ON). When the LED returns to blinking, stop the table.</li>
<li><strong>Counter-Clockwise (CCW):</strong> Configure the table to spin CCW at the defined velocity and start rotation. Press the button to log (LED solid ON). When the LED returns to blinking, stop the table.</li>
</ul>
</li>
<li><p align="justify"><strong>Connect to PC:</strong></p>
<ul>
<li>If using <strong>E5V</strong>: Keep the batteries connected and switch ON. Connect the USB cable to the PC.</li>
<li>If using <strong>U5V</strong>: Disconnect the power bank and connect the USB cable to the PC.</li>
</ul>
</li>
<li><p align="justify"><strong>Identify Port:</strong> Open Windows Device Manager and find the COM port number for <strong>STMicroelectronics STLink Virtual COM Port</strong>.</p></li>
<li><p align="justify"><strong>Run Analysis:</strong> Update the <code>COM_PORT</code> variable in the Python script and run it.</p></li>
<li><p align="justify"><strong>Results:</strong> The script generates a text file containing the logs and a summary of the results, as well as a plot displaying the averaged angular rates (<img src="https://math.vercel.app?from=\bar{\omega}_{i^{cw}}" /> and <img src="https://math.vercel.app?from=\bar{\omega}_{i^{ccw}}" />) per axis, along with the calculated scale factor errors.</p></li>
</ol>

<img width="1536" height="850" alt="gyro_dynamic_test_10dps_1" src="https://github.com/user-attachments/assets/b4264b01-5523-4677-9cf8-4e0db0825d4c" />

<h3>Gyroscope Calibration</h3>

<p align="justify">
This procedure applies the calculated biases and scale factor errors to a selected raw data log. The Python script utilizes the inverse error model to recover the calibrated angular velocity (<img src="https://latex.codecogs.com/svg.latex?\mathbf{\omega}" />) and compares it against the raw measured data (<img src="https://latex.codecogs.com/svg.latex?\mathbf{\hat{\omega}}" />) for visual inspection.
</p>

<p align="justify">
<strong>Prerequisite:</strong> Download the script located at <code>MO-2_GyroscopeVerification/PythonScripts/log_gyro_calibration.py</code>.
</p>

<p align="justify"><strong>Execution Steps:</strong></p>

<ol type="1">
  <li>
    <p align="justify">
      <strong>Select Log File:</strong> Locate the text file containing the raw gyroscope data you wish to calibrate. Ensure this file is accessible to the script.
    </p>
  </li>

  <li>
    <p align="justify">
      <strong>Configure Script:</strong> Open the Python script in a text editor or IDE. You must manually update the <strong>USER CONFIGURATION</strong> section at the top of the file with the specific parameters obtained from previous tests:
    </p>
    <ul>
      <li><p align="justify">Update the <code>LOG_FILE_NAME</code> variable with the exact name of your target log file.</p></li>
      <li><p align="justify">Enter the calculated Bias values (<code>b_x, b_y, b_z</code>) in units of dps.</p></li>
      <li><p align="justify">Enter the calculated Scale Factor Error values (<code>s_x, s_y, s_z</code>).</p></li>
    </ul>
  </li>

  <li>
    <p align="justify">
      <strong>Run Analysis:</strong> Run the Python script. This script processes the file locally and does not require the hardware system to be connected.
    </p>
  </li>

  <li>
    <p align="justify">
      <strong>Results:</strong> The script will automatically detect the number of data cycles in the log file and generate a <strong>separate window</strong> for each cycle. Each window contains three subplots (X, Y, Z) displaying:
    </p>
    <ul>
       <li><p align="justify"><strong>Uncalibrated (Dark Blue):</strong> The raw angular velocity as originally measured by the sensor.</p></li>
       <li><p align="justify"><strong>Calibrated (Light Blue):</strong> The corrected angular velocity recovered using the manual input parameters.</p></li>
    </ul>
  </li>
</ol>

<p align="justify">
This visual comparison allows for the immediate verification of the calibration performance across different test positions or rotations.
</p>

<h3>Gyroscope Biases - Static Thermal Test:</h3>

<p align="justify">
This procedure determines the dependence of gyroscope bias on temperature by calculating the bias values at five distinct points: -20°C, 0°C, 20°C, 40°C, and 60°C. Since reorienting the system inside the thermal chamber during a test is not feasible, the standard static method is adapted into three separate thermal runs. Each run maintains a single static position while sweeping through the full temperature range.
</p>

<p align="justify">
The process begins with the system in <strong>Position 1</strong>, executing the thermal sweep, after which the data is retrieved and the flash memory is erased. This sequence is repeated identically for <strong>Position 2</strong> and <strong>Position 3</strong>, ensuring the memory is cleared between runs.</p>

<p align="justify">
Two scripts manage the data: the first retrieves the raw logs from each run, and the second stitches these three files together. By grouping the data from all three positions corresponding to the same temperature point, the script calculates the final bias for each of the five thermal steps.
</p>

<p align="justify">
<strong>Prerequisite:</strong> Download the retrieval script (<code>gyro_static_thermal_test_position.py</code>) and the calculation script (<code>gyro_static_thermal_test_biases.py</code>).</p>

<p align="justify"><strong>Execution Steps:</strong></p>

<ol type="1">
  <li>
    <p align="justify">
      <strong>Configure Firmware:</strong> In STM32CubeIDE, open <code>Core/Src/main.c</code> and modify <code>#define LOG_DURATION_MS</code> to set the log duration <em>T</em> (in milliseconds) for each temperature point. Ensure the duration is sufficient to capture stable data.
    </p>
  </li>

<li>
    <p align="justify">
      <strong>Power Up & Cabling:</strong> To ensure safety and operational stability across the full temperature range, lithium-ion batteries and portable power banks are strictly prohibited inside the thermal chamber. Instead, the system must be powered using a bench power supply placed inside the chamber alongside the device. Route the power supply's AC power cable and the extended logging control button cable through the chamber's side access port, ensuring that the system can be powered and controlled from the outside. Once the cables are routed, seal the access port to ensure thermal isolation. Connect the power supply to an external outlet, configure the output to 5V, and verify that the PCB LED is toggling to confirm the system is in Idle mode.
    </p>
</li>

<li>
    <p align="justify">
      <strong>Perform Thermal Run 1 (Position 1):</strong>
    </p>
    <ul>
      <li>
        <p align="justify">
          <strong>Verify Orientation:</strong> Ensure that the system was placed in <strong>Position 1</strong> during the initial setup.
        </p>
      </li>
      <li>
        <p align="justify">
          <strong>Temperature Sweep:</strong> For each temperature target (-20°, 0°, 20°, 40°, 60°): Set the chamber temperature and allow the system to stabilize for the designated <strong>Soak Time</strong> (typically <strong>15 minutes</strong>). Once stable, press the external button to log data (LED Solid ON). Wait for the LED to return to blinking before proceeding to the next temperature.
        </p>
      </li>
      <li>
        <p align="justify">
          <strong>Retrieve Data:</strong> Once the sweep is complete, turn off the power supply and remove the system from the chamber. Connect it to the PC, run the retrieval script, and save the log file with a unique name corresponding to Position 1.
        </p>
      </li>
      <li>
        <p align="justify">
          <strong>Erase Memory:</strong> Send the <strong>'e'</strong> command via the Serial Terminal to clear the flash memory. <strong>This step is critical to prevent data overlap.</strong>
        </p>
      </li>
    </ul>
  </li>

  <li>
    <p align="justify">
      <strong>Perform Thermal Run 2 (Position 2):</strong>
    </p>
    <ul>
      <li>
        <p align="justify">
          <strong>Reorient & Setup:</strong> Place the system back inside the thermal chamber, this time oriented in <strong>Position 2</strong>. Route the cables, seal the access port, and power up the system as before.
        </p>
      </li>
      <li>
        <p align="justify">
          <strong>Temperature Sweep:</strong> Repeat the logging process for all temperature targets, adhering to the 15-minute Soak Time for each step.
        </p>
      </li>
      <li>
        <p align="justify">
          <strong>Retrieve Data:</strong> Remove the system, retrieve the data, and save the log file with a unique name corresponding to Position 2.
        </p>
      </li>
      <li>
        <p align="justify">
          <strong>Erase Memory:</strong> Clear the flash memory using the <strong>'e'</strong> command.
        </p>
      </li>
    </ul>
  </li>

  <li>
    <p align="justify">
      <strong>Perform Thermal Run 3 (Position 3):</strong>
    </p>
    <ul>
      <li>
        <p align="justify">
          <strong>Reorient & Setup:</strong> Place the system back inside the thermal chamber, oriented in <strong>Position 3</strong>. Route the cables, seal the access port, and power up.
        </p>
      </li>
      <li>
        <p align="justify">
          <strong>Temperature Sweep:</strong> Repeat the logging process for all temperature targets, adhering to the 15-minute Soak Time for each step.
        </p>
      </li>
      <li>
        <p align="justify">
          <strong>Retrieve Data:</strong> Remove the system, retrieve the data, and save the final log file corresponding to Position 3.
        </p>
      </li>
    </ul>
  </li>

  <li>
    <p align="justify">
      <strong>Run Analysis:</strong> Open <code>gyro_static_thermal_test_biases.py</code>. Update the <code>LOG_FILES</code> list to include the three text files. Run the script and map the files to their positions when prompted.
    </p>
  </li>

  <li>
    <p align="justify">
      <strong>Results:</strong> The script generates a summary text file and a plot illustrating the bias drift for the X, Y, and Z axes across the tested range.
    </p>
  </li>
</ol>

<img width="1536" height="850" alt="gyro_thermal_test_comparison" src="https://github.com/user-attachments/assets/7f8abaf9-ad3e-48db-99f9-706db1fb201e" />

<h2>Gyroscope Noise and Allan Deviation</h2>

<p align="justify">
Gyroscopes are critical sensors in aerospace applications, such as rockets and satellites. To create accurate simulations, engineers must precisely model the sensors. This requires characterizing stochastic errors (noise) that cannot be removed by simple calibration [5].
</p>

<h3>Angle Random Walk (ARW)</h3>

<p align="justify">
Angle Random Walk describes the high-frequency "white noise" in the gyroscope's rate output. The concept is based on a "random walk" mathematical model, analogous to flipping a coin to decide whether to take a step forward or backward. Even with equal probability, after many flips, your position will drift randomly from the starting line. Since gyroscope rate measurements are integrated over time to compute angles, this white noise causes the calculated angle to take random steps, accumulating drift from sample to sample [5].
</p>

<h3>Bias Instability</h3>

<p align="justify">
While a gyroscope has a constant "turn-on bias" (the offset reading when stationary), this bias is not truly static—it wanders gradually over time. Bias Instability quantifies this low-frequency drift. It represents the limit of the sensor's stability; meaning that after a certain duration, averaging the data no longer improves accuracy but instead includes more error due to this drift. This is arguably the most critical parameter for sensor fusion algorithms (like Kalman Filters), which typically assume bias is constant; if instability is high, the filter may fail to track the bias correctly [5].
</p>

<h3>Allan Deviation</h3>

<p align="justify">
To quantify these parameters, we use the <strong>Allan Variance</strong>. Originally derived to measure the noise characteristics and frequency stability of clock oscillators, this method is used to separate random noise processes from systematic errors (such as temperature effects) [5].
</p>

<p align="justify">
The result is the <strong>Allan Deviation</strong> (<img src="https://latex.codecogs.com/svg.latex?\sigma" alt="sigma"/>), which is simply the square root of the Allan Variance. It is visualized as a plot on a log-log scale. The x-axis represents the <strong>averaging time</strong> (<img src="https://latex.codecogs.com/svg.latex?\tau" alt="tau"/>) in seconds, and the y-axis represents the deviation in degrees per second. By analyzing the slope and shape of this curve, we can extract the specific values for ARW and Bias Instability using the formulas below [5].
</p>

<h3>Calculating Noise Parameters</h3>

<p align="justify">
<strong>1. Angle Random Walk:</strong> On the plot, this appears as a slope of <strong>-0.5</strong>. It is calculated by taking the deviation value at <img src="https://latex.codecogs.com/svg.latex?\tau=1" alt="tau=1"/> second and converting it to standard units [5]:
</p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?ARW%20=%20\sigma(1)%20\frac{\text{deg}}{\text{s}}%20\times%2060%20\frac{\text{s}}{\sqrt{\text{hr}}}" alt="ARW Formula" />
</p>

<p align="justify">
<strong>2. Bias Instability:</strong> On the plot, this corresponds to the <strong>local minimum</strong> (the "valley" or flat region) of the curve. It is calculated using the minimum deviation value (<img src="https://latex.codecogs.com/svg.latex?\sigma_{min}" alt="sigma_min"/>) and the standard constant 0.664 from IEEE Standard 952-1997 [5]:
</p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?B_I%20=%20\sigma_{min}%20\frac{\text{deg}}{\text{s}}%20\times%20\frac{1}{0.664}%20\times%203600%20\frac{\text{s}}{\text{hr}}" alt="Bias Instability Formula" />
</p>

<h3>Prerequisite:</h3>
 Download the script located at <code>MO-2_GyroscopeVerification/PythonScripts/gyro_ARW_bias_instability.py</code>.

<h3>Execution Steps:</h3>

<ol type="1">
  <li>
    <p align="justify">
      <strong>Configure Firmware:</strong> In <code>Core/Src/main.c</code>, set <code>#define LOG_DURATION_MS</code> to <strong>14400000</strong> (4 Hours).
    </p>
  </li>
  <li>
    <p align="justify">
      <strong>Power Up:</strong> Use the Battery Pack (E5V) to ensure stable power for the 4-hour duration.
    </p>
  </li>
  <li>
    <p align="justify">
      <strong>Data Logging:</strong> Place the system in a vibration-free environment. Press the button to start the long logging cycle.
    </p>
  </li>
  <li>
    <p align="justify">
      <strong>Retrieve Data:</strong> Connect to the PC and send the <strong>'r'</strong> command. Note that retrieving 4 hours of data may take several minutes.
    </p>
  </li>
  <li>
    <p align="justify">
      <strong>Run Analysis:</strong> Run the Python script. It will parse the long log, calculate the Allan Deviation, and automatically identify the ARW and the Bias Instability.
    </p>
  </li>
  <li>
    <p align="justify">
      <strong>Results:</strong> The script outputs the calculated noise parameters and generates a plot of the Allan Deviation.
    </p>
  </li>
</ol>

<h2>References</h2>

<ol>
<li><p align="justify">Kyushu Institute of Technology, "Kyutech Brochure 2024," Kitakyushu, Japan, 2024. [Online]. Available: <a href="https://www.kyutech.ac.jp//media/014/202404/brochure2024.pdf">https://www.kyutech.ac.jp//media/014/202404/brochure2024.pdf</a></p></li>
<li><p align="justify">Z. Yampolsky and I. Klein, "Data-Driven Gyroscope Calibration," <em>arXiv preprint arXiv:2410.12485</em>, 2024. [Online]. Available: <a href="https://arxiv.org/pdf/2410.12485">https://arxiv.org/pdf/2410.12485</a></p></li>
<li><p align="justify">D. Royo Serrano, "Development of a calibration procedure for gyroscopes in CubeSat missions," Master's thesis, Luleå University of Technology, Luleå, Sweden, 2021. [Online]. Available: <a href="https://www.diva-portal.org/smash/get/diva2:1537570/FULLTEXT01.pdf">https://www.diva-portal.org/smash/get/diva2:1537570/FULLTEXT01.pdf</a></p></li>
<li><p align="justify">X. Niu, Y. Li, H. Zhang, Q. Wang, and Y. Ban, "Fast Thermal Calibration of Low-Grade Inertial Sensors and Inertial Measurement Units," <em>Sensors</em>, vol. 13, no. 9, pp. 12192-12217, 2013. [Online]. Available: <a href="https://doi.org/10.3390/s130912192">https://doi.org/10.3390/s130912192</a></p></li>
<li><p align="justify">M. Wrona, "Gyro Noise and Allan Deviation + IMU Example," <em>Michael Wrona's Blog</em>, May 9, 2021. [Online]. Available: <a href="https://mwrona.com/posts/gyro-noise-analysis/">https://mwrona.com/posts/gyro-noise-analysis/</a></p></li>