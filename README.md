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
For the MO-2 mission, one primary objective is to observe the satellite's condition and reflect status data—specifically angular velocity—into a game designed to inspire interest in space among a wider audience. The selected sensor for this task, chosen for its flight heritage, is the MEMS gyroscope L3G4200D. Selecting a sensor without a proven space record would necessitate complex and costly qualification procedures, such as radiation testing.
</p>

<p align="justify">
MEMS gyroscopes are widely adopted in CubeSat missions due to their compact size, low power consumption, cost-effectiveness, and precision. However, their accuracy tends to degrade over time as a result of  combined errors, including noise, biases, drift, and scale factor instability. If left uncorrected, these deterministic errors accumulate, leading to progressively larger discrepancies in position and orientation estimates, a phenomenon well-documented in previous missions utilizing MEMS sensors for Attitude Determination and Control Systems (ADCS) [3].
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
  <li>
    <p align="justify">
      <strong>Bias (Offset):</strong> The deviation of the gyroscope output from the expected theoretical value when the device is stationary. This "zero reading" tends to drift over time due to the integration of inherent device imperfections and internal noise [3].
    </p>
  </li>
  <li>
    <p align="justify">
      <strong>Scale Factor Error:</strong> A metric describing the deviation of the sensor's sensitivity from unity. It quantifies the discrepancy between the sensor's measured output range and the actual input rotation range [3].
    </p>
  </li>
  <li>
    <p align="justify">
      <strong>Non-orthogonalities (Misalignment):</strong> The error resulting from imperfect alignment of the gyroscope's sensing axes relative to an ideal mutually orthogonal coordinate system [3].
    </p>
  </li>
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
  <img src="https://latex.codecogs.com/svg.latex?\begin{bmatrix}\hat{\omega}_x\\\hat{\omega}_y\\\hat{\omega}_z\end{bmatrix}=\begin{bmatrix}1 + s_{x}&0&0\\0&1 + s_{y}&0\\0&0&1 + s_{z}\end{bmatrix}\begin{bmatrix}\omega_x\\\omega_y\\\omega_z\end{bmatrix}+\begin{bmatrix}b_x\\b_y\\b_z\end{bmatrix}\quad(4)" alt="Reduced Model (4)" />
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
The scale factor errors are determined using a procedure similar to the bias calculation, but this time a rotary table is employed to spin the gyroscope triad both clockwise and counter-clockwise for each sensitive axis (six different measurements). The scale factor error for a given axis <em>i</em> is calculated as [3]:
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
  <img src="https://latex.codecogs.com/svg.latex?\begin{bmatrix}\omega_x\\\omega_y\\\omega_z\end{bmatrix}=\begin{bmatrix}1 + s_{x}&0&0\\0&1 + s_{y}&0\\0&0&1 + s_{z}\end{bmatrix}^{-1}\left(\begin{bmatrix}\hat{\omega}_x\\\hat{\omega}_y\\\hat{\omega}_z\end{bmatrix}-\begin{bmatrix}b_x\\b_y\\b_z\end{bmatrix}\right)\quad(8)" alt="Inverse Error Matrix Model (8)" />
</p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?\omega_{x}=\frac{\hat{\omega}_{x}-b_{x}}{1 + s_{x}},\quad\omega_{y}=\frac{\hat{\omega}_{y}-b_{y}}{1 + s_{y}},\quad\omega_{z}=\frac{\hat{\omega}_{z}-b_{z}}{1 + s_{z}}\quad(9)" alt="Scalar Calibration Formulas (9)" />
</p>

<h2>Thermal Calibration</h2>

<p align="justify">
Thermal calibration is performed to model the temperature dependencies of the gyroscope's deterministic errors: bias and scale factor. Two primary methods are employed for this characterization [3]:

</p>

<ul>
  <li>
    <p align="justify">
      <strong>Thermal Soak Method:</strong> The MEMS sensor is placed inside a thermal chamber and allowed to stabilize at a series of discrete temperature setpoints. Data collection is initiated only after the sensor has reached thermal equilibrium at each target temperature. By recording measurements and calculating sensor errors at these stable points, a dataset of error-versus-temperature values is generated. Errors at intermediate temperatures can then be estimated using interpolation techniques [4]. 
    </p>
  </li>
  <li>
<p align="justify">
  <strong>Thermal Ramp Method:</strong> In this approach, the sensor is polled continuously while the thermal chamber temperature is linearly increased or decreased across the desired operating range. While this method is inherently faster because it eliminates the time-consuming stabilization periods, it introduces two significant sources of error. First, a thermal gradient often exists between the inertial sensor core and the temperature sensor, leading to measurement discrepancies similar to hysteresis. Second, because the temperature evolves dynamically during the data acquisition window for a single calibration scheme, the resulting error parameters are derived from data collected at varying temperatures rather than a single thermal point [4].
</p>

  </li>
</ul>

<p align="justify">
Although time-intensive, the soak method ensures the most reliable error characterization [4]; therefore, it has been selected for this verification.
</p>

<h2>Hardware Design</h2>

<p align="justify"> Detailed documentation regarding the hardware design is maintained in a dedicated repository. For access to the files required to reproduce the custom PCB, please visit the <a href="https://github.com/fectec/MO-2_GyroscopeShield.git">MO-2_GyroscopeShield</a> repository. </p>

<p align="justify"> For the steps below, it is assumed that all components have been soldered onto the PCB. This should be done following the silkscreen labels provided on the PCB surface. When referring to the "System," this implies the custom PCB mounted on top of the NUCLEO-F446RE board. </p>

<p align="justify"> There are two distinct options for powering the system: <strong>E5V</strong> (External 5V) or <strong>U5V</strong> (USB 5V). <strong>These modes cannot coexist; you must choose one.</strong> </p>

<h3>Option 1: E5V</h3> <p align="justify"> In this mode, the power subsystem uses an LM2596 step-down switching regulator. This component regulates the input voltage from a main battery pack (two Lithium-Ion cells) to the stable 5V required by the NUCLEO-F446RE board. </p> <ul> <li><strong>Switch:</strong> A switch is required to power the system remotely. Solder a cable to each terminal of the switch and secure the free ends to the <strong>J5</strong> terminal on the PCB. If a switch is not used, a jumper wire must be installed in J5 to close the circuit; otherwise, the system will not power on.</li> <li><strong>Battery:</strong> The Li-Ion batteries must be charged using an appropriate charger and placed in the battery holder. Connect the battery holder cables to the <strong>J4</strong> power terminal, strictly following the polarity markings on the PCB silkscreen.</li> </ul>

<p align="justify"> <strong>WARNING:</strong> Failure to respect the following order of operations may damage the NUCLEO-F446RE board or the PC. </p> <ol> <li>Connect a jumper between <strong>Pin 2 and Pin 3</strong> of header <strong>JP5</strong> on the NUCLEO-F446RE board.</li> <li>Ensure that jumper <strong>JP1</strong> on the NUCLEO-F446RE board is <strong>removed</strong>.</li> <li>Mount the custom PCB onto the NUCLEO-F446RE board.</li> <li>Connect the battery holder cables to the <strong>J4</strong> terminal on the PCB (observe polarity).</li> <li>Verify that the red <strong>LD3 LED</strong> on the NUCLEO-F446RE board turns on.</li> <li>Only after these steps, if code upload is required, connect the PC to the USB connector <strong>CN1</strong>.</li> </ol>

<h3>Option 2: U5V</h3> <p align="justify"> In this mode, the system is powered directly via the ST-LINK USB connector (CN1). You may use a PC or a portable power bank capable of supplying 5V and at least 300 mA. </p> <p align="justify"> <strong>Note:</strong> If using a power bank, the firmware must be uploaded to the board before connecting the power bank. If using a PC, the code can be uploaded while powered. </p>

<p align="justify"> <strong>WARNING:</strong> Failure to respect the following order of operations may damage the board. </p> <ol> <li>Connect a jumper between <strong>Pin 1 and Pin 2</strong> of header <strong>JP5</strong> on the NUCLEO-F446RE board (this differs from E5V mode).</li> <li>Ensure that jumper <strong>JP1</strong> on the NUCLEO-F446RE board is <strong>removed</strong>.</li> <li>Mount the custom PCB onto the NUCLEO-F446RE board.</li> <li>Connect the PC or Power Bank to the USB connector <strong>CN1</strong> on the NUCLEO-F446RE.</li> </ol>

<p align="justify"> <strong>Note:</strong> During the actual tests, using a battery (E5V) or power bank (U5V) is mandatory. A USB connection to a PC is not feasible due to the cable. </p>

<h2>Calibration Procedure</h2>

<p align="justify"> The custom PCB includes a push-button that controls the data logging process. Pressing the button initiates an uninterrupted gyroscope data logging cycle for a specific duration <em>T</em>. Once the cycle finishes, another can be initiated.</p> <ul> 

<li><p align="justify"><strong>Gyroscope Biases - Static Test:</strong>
Each cycle corresponds to a specific position.</p>
<li><p align="justify"><strong>Gyroscope Scale Factor Errors - Dynamic Rotary Test:</strong> Each cycle corresponds to a specific rotation.</p>
<li><p align="justify"><strong>Gyroscope Biases - Static Thermal Test:</strong> Each cycle corresponds to a specific temperature stability point. The button logic is mandatory here, as the time required for the chamber to stabilize varies and cannot be automated with a simple timer. </p>

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

<p align="justify"> <strong>IMPORTANT:</strong> Before starting a new test, you must erase the previous data from the flash memory. Connect the system to the PC, open a Serial Terminal (like PuTTY or the Arduino Serial Monitor) on the correct COM port, and send the character <strong>'e'</strong>. This clears the memory. Failure to do this will result in corrupted data when reading the new test. </p>

<h3>Gyroscope Biases - Static Test Procedure</h3>

<p align="justify">
The NUCLEO-F446RE logs raw gyroscope data to the flash memory. The calculation of the biases is performed by a Python script on the PC.
</p>

<p align="justify">
<strong>Prerequisite:</strong> Download the script located at <code>MO-2_GyroscopeVerification/PythonScripts/gyro_biases_static_test.py</code>. This script is compatible with Windows.
</p>

<p align="justify"><strong>Execution Steps:</strong></p>

<ol>
  <li>
    <p align="justify">
      <strong>Configure Firmware:</strong> In STM32CubeIDE, open <code>Core/Src/main.c</code> and modify <code>#define LOG_DURATION_MS</code> to set the log duration <em>T</em> (in milliseconds) for each button press.
    </p>
  </li>

  <li>
    <p align="justify">
      <strong>Power Up:</strong> Connect the Battery Pack (E5V) or Power Bank (U5V). If using E5V, turn on the switch. Verify the PCB LED is toggling (blinking), indicating Idle mode. Ensure no USB cables are connected.
    </p>
  </li>

  <li>
    <p align="justify">
      <strong>Perform Data Logging:</strong> Perform the logging routine for each of the required positions. <strong>CRITICAL: You must perform the tests in the exact order shown (Position 1 &rarr; Position 2 &rarr; Position 3).</strong> For every position listed below:
    </p>
    <ul>
      <li>Place the system on a leveled surface matching the reference image.</li>
      <li>Press the button to start logging. The PCB LED will turn <strong>Solid ON</strong>.</li>
      <li>Wait for the PCB LED to return to <strong>Blinking</strong> (cycle complete) before moving to the next position.</li>
    </ul>

  <p align="center">
    <strong>Position 1</strong><br>
    <img src="path_to_image_1.jpg" alt="Position 1 Alignment" width="300"><br><br>
  </p>

  <p align="center">
  <strong>Position 2</strong><br>
    <img src="path_to_image_2.jpg" alt="Position 2 Alignment" width="300"><br><br>
  </p> 

  <p align="center">
  <strong>Position 3</strong><br>
    <img src="path_to_image_3.jpg" alt="Position 3 Alignment" width="300">
  </p>

  </li>

  <li>
    <p align="justify">
      <strong>Connect to PC:</strong>
    </p>
    <ul>
      <li>If using <strong>E5V</strong>: Keep the batteries connected and switch ON. Connect the USB cable to the PC.</li>
      <li>If using <strong>U5V</strong>: Disconnect the power bank and connect the USB cable to the PC.</li>
    </ul>
  </li>

  <li>
    <p align="justify">
      <strong>Identify Port:</strong> Open Windows Device Manager and find the COM port number for <strong>STMicroelectronics STLink Virtual COM Port</strong>.
    </p>
  </li>

  <li>
    <p align="justify">
      <strong>Run Analysis:</strong> Update the <code>COM_PORT</code> variable in the Python script and run it.
    </p>
  </li>

  <li>
    <p align="justify">
      <strong>Results:</strong> The script generates a text file containing the logs and a summary of the results, as well as a plot displaying the averaged angular rates (<img src="https://math.vercel.app?from=\bar{\omega}_{i^{up}}" /> and <img src="https://math.vercel.app?from=\bar{\omega}_{i^{down}}" />) per axis, along with the calculated biases.
    </p>
  </li>
</ol>

<h3>Gyroscope Scale Factor Errors - Dynamic Rotary Test:</h3>

<p align="justify">
The NUCLEO-F446RE logs raw gyroscope data to the flash memory. The calculation of the scale factor errors is performed by a Python script on the PC.
</p>

<p align="justify">
<strong>Prerequisite:</strong> Download the script located at <code>MO-2_GyroscopeVerification/PythonScripts/gyro_scale_factor_errors_dynamic_rotary_test.py</code>. This script is compatible with Windows.
</p>

<p align="justify"><strong>Execution Steps:</strong></p>

<ol>
  <li>
    <p align="justify">
      <strong>Configure Firmware:</strong> In STM32CubeIDE, open <code>Core/Src/main.c</code> and modify <code>#define LOG_DURATION_MS</code> to set the log duration <em>T</em> (in milliseconds) for each button press.
    </p>
  </li>

  <li><strong>Configure Script:</strong> In the Python script, update the variable <code>TABLE_GROUND_TRUTH_DPS</code> to match the angular velocity you will set on the rotary table.</li>

  <li>
    <p align="justify">
      <strong>Power Up:</strong> Connect the Battery Pack (E5V) or Power Bank (U5V). If using E5V, turn on the switch. Verify the PCB LED is toggling (blinking), indicating Idle mode. Ensure no USB cables are connected.
    </p>
  </li>

<li>
  <p align="justify">
    <strong>Perform Data Logging:</strong> Execute the logging sequence for the three positions shown below. 
    <strong>CRITICAL: You must perform the tests in the exact order shown (Position 1 (CW &rarr; CCW) &rarr; Position 2 (CW &rarr; CCW) &rarr; Position 3 (CW &rarr; CCW)).</strong> 
  </p>

  <p align="center">
    <strong>Position 1</strong><br>
    <img src="path_to_image_1.jpg" alt="Position 1 Alignment" width="300"><br><br>
  </p>

  <p align="center">
  <strong>Position 2</strong><br>
    <img src="path_to_image_2.jpg" alt="Position 2 Alignment" width="300"><br><br>
  </p> 
   
  <p align="center">
  <strong>Position 3</strong><br>
    <img src="path_to_image_3.jpg" alt="Position 3 Alignment" width="300">
  </p>

  <p align="justify">For <strong>each</strong> position, repeat the following steps:</p>
  <ul>
    <li>Place the system on the rotary table as shown in the corresponding image.</li>
    <li><strong>Clockwise (CW):</strong> Configure the table to spin CW at the defined velocity and start rotation. Press the button to log (LED solid ON). When the LED returns to blinking, stop the table.</li>
    <li><strong>Counter-Clockwise (CCW):</strong> Configure the table to spin CCW at the defined velocity and start rotation. Press the button to log (LED solid ON). When the LED returns to blinking, stop the table.</li>
  </ul>
</li>

  <li>
    <p align="justify">
      <strong>Connect to PC:</strong>
    </p>
    <ul>
      <li>If using <strong>E5V</strong>: Keep the batteries connected and switch ON. Connect the USB cable to the PC.</li>
      <li>If using <strong>U5V</strong>: Disconnect the power bank and connect the USB cable to the PC.</li>
    </ul>
  </li>

  <li>
    <p align="justify">
      <strong>Identify Port:</strong> Open Windows Device Manager and find the COM port number for <strong>STMicroelectronics STLink Virtual COM Port</strong>.
    </p>
  </li>

  <li>
    <p align="justify">
      <strong>Run Analysis:</strong> Update the <code>COM_PORT</code> variable in the Python script and run it.
    </p>
  </li>

  <li>
    <p align="justify">
      <strong>Results:</strong> The script generates a text file containing the logs and a summary of the results, as well as a plot displaying the averaged angular rates (<img src="https://math.vercel.app?from=\bar{\omega}_{i^{cw}}" /> and <img src="https://math.vercel.app?from=\bar{\omega}_{i^{ccw}}" />) per axis, along with the calculated scale factor errors.
    </p>
  </li>
</ol>

<h3>Gyroscope Biases - Static Thermal Test:</h3>

<p align="justify">
The NUCLEO-F446RE logs raw gyroscope data to the flash memory. The plotting of the angular velocity  versus temperature is performed by a Python script on the PC.
</p>

<p align="justify">
<strong>Prerequisite:</strong> Download the script located at <code>MO-2_GyroscopeVerification/PythonScripts/gyro_biases_static_thermal_test.py</code>. This script is compatible with Windows.
</p>

<p align="justify"><strong>Execution Steps:</strong></p>

<ol>
  <li>
    <p align="justify">
      <strong>Configure Firmware:</strong> In STM32CubeIDE, open <code>Core/Src/main.c</code> and modify <code>#define LOG_DURATION_MS</code> to set the log duration <em>T</em> (in milliseconds) for each button press.
    </p>
  </li>

  <li>
    <p align="justify">
      <strong>Power Up:</strong> Connect the Battery Pack (E5V) or Power Bank (U5V). If using E5V, turn on the switch. Verify the PCB LED is toggling (blinking), indicating Idle mode. Ensure no USB cables are connected.
    </p>
  </li>

<li>
  <p align="justify">
    <strong>Perform Data Logging:</strong> Place the system inside the thermal chamber as shown in Position 1.
  </p>  

  <p align="center">
    <strong>Position 1</strong><br>
    <img src="path_to_image_1.jpg" alt="Position 1 Alignment" width="300"><br><br>
  </p> 
    
  <p>For each temperature point defined in your test plan, execute the following sequence:
  </p>
  <ul>
    <li><strong>Set Temperature:</strong> Configure the thermal chamber to the target temperature and allow sufficient time for it to stabilize (soak time).</li>
    <li><strong>Start Logging:</strong> Press the button on the PCB to start logging. The LED will stop blinking and remain <strong>solid ON</strong>.</li>
    <li><strong>Wait for Completion:</strong> When the logging cycle finishes, the LED will return to blinking.</li>
    <li><strong>Repeat:</strong> Change the chamber temperature to the next point and repeat the steps above.</li>
  </ul>
</li>

  <li>
    <p align="justify">
      <strong>Connect to PC:</strong>
    </p>
    <ul>
      <li>If using <strong>E5V</strong>: Keep the batteries connected and switch ON. Connect the USB cable to the PC.</li>
      <li>If using <strong>U5V</strong>: Disconnect the power bank and connect the USB cable to the PC.</li>
    </ul>
  </li>

  <li>
    <p align="justify">
      <strong>Identify Port:</strong> Open Windows Device Manager and find the COM port number for <strong>STMicroelectronics STLink Virtual COM Port</strong>.
    </p>
  </li>

  <li>
    <p align="justify">
      <strong>Run Analysis:</strong> Update the <code>COM_PORT</code> variable in the Python script and run it.
    </p>
  </li>

  <li>
    <p align="justify">
      <strong>Results:</strong> The script generates a text file containing the logs and a summary of the results, as well as a plot displaying the averaged angular rates for each axis at each temperature point.
    </p>
  </li>
</ol>

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
  <li>
    <p align="justify">
      D. Royo Serrano, "Development of a calibration procedure for gyroscopes in CubeSat missions," Master's thesis, Luleå University of Technology, Luleå, Sweden, 2021. [Online]. Available: <a href="https://www.diva-portal.org/smash/get/diva2:1537570/FULLTEXT01.pdf">https://www.diva-portal.org/smash/get/diva2:1537570/FULLTEXT01.pdf</a>
    </p>
  <li>
    <p align="justify">
      X. Niu, Y. Li, H. Zhang, Q. Wang, and Y. Ban, "Fast Thermal Calibration of Low-Grade Inertial Sensors and Inertial Measurement Units," <em>Sensors</em>, vol. 13, no. 9, pp. 12192-12217, 2013. [Online]. Available: <a href="https://doi.org/10.3390/s130912192">https://doi.org/10.3390/s130912192</a>
    </p>
  </li>
</ol>
  </li>
</ol>
