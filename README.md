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
The six-position calibration method is a model-based approach executed before the mission starts to estimate the constant error terms of the gyroscope. This procedure requires rotating the gyroscope into six distinct orientations: twice per axis, once clockwise and once counterclockwise. Each rotation is sustained for a duration <em>T</em>, allowing the measurements to be averaged to eliminate sensor noise. For a single axis, this calculation simplifies into a system of two equations with two unknowns: bias and scale factor [2].
</p>

<p align="justify">
For a single axis <em>i</em>, the calculation is expressed as:
</p>

<p align="center">
  <img src="https://latex.codecogs.com/svg.latex?s_i=\frac{\bar{\omega}_{i^-}-\bar{\omega}_{i^+}-2\omega_i}{2\omega_i}" alt="Scale Factor Equation" />
  <br><br>
  <img src="https://latex.codecogs.com/svg.latex?b_i=\frac{\bar{\omega}_{i^+}+\bar{\omega}_{i^-}}{2}" alt="Bias Equation" />
</p>

<p align="justify">
where <img src="https://latex.codecogs.com/svg.latex?\bar{\omega}_{i^+}" /> is the average gyroscope output for axis <em>i</em> while pointing in the positive direction (up), <img src="https://latex.codecogs.com/svg.latex?\bar{\omega}_{i^-}" /> is the average output for axis <em>i</em> while pointing in the negative direction (down), and <img src="https://latex.codecogs.com/svg.latex?\omega_i" /> is the ground truth (GT) angular velocity of the turntable. Solving the equations yields the gyroscope axis bias, <em>b<sub>i</sub></em>, and the scale factor, <em>s<sub>i</sub></em> [2].
</p>

<h2>Hardware Design</h2>

<p align="justify">
Detailed documentation regarding the hardware design is maintained in a dedicated repository. For access to the files required to reproduce the custom PCB, please visit the <a href="https://github.com/fectec/MO-2_GyroscopeShield.git">MO-2_GyroscopeShield</a> repository.
</p>

<h2>Firmware Implementation</h2>

<p align="justify">
The firmware is split into different branches depending on the test. The <code>thermal_chamber</code> branch contains the code for the temperature tests. The <code>rotary_table</code> branch contains the code for the calibration on the rotation table. Simply switch to the branch that matches the test you are currently running.
</p>

<ol>
  <li>
    <p align="justify">
      <strong>Clone the Repository:</strong> Open your terminal, navigate to your STM32CubeIDE workspace directory, and execute the following command:
    </p>
    <pre><code>git clone https://github.com/fectec/MO-2_GyroscopeVerification.git</code></pre>
  </li>

  <li>
    <p align="justify">
      <strong>Import Project:</strong> In STM32CubeIDE, go to the <strong>File</strong> tab and select <strong>Open Projects from File System</strong>. In the "Import source" directory field, browse and select the cloned <code>MO-2_GyroscopeVerification</code> folder. Ensure that <em>"Search for nested projects"</em> and <em>"Detect and configure project natures"</em> are checked, then click <strong>Finish</strong>.
    </p>
  </li>

  <li>
    <p align="justify">
      <strong>Flash Firmware:</strong> Connect the NUCLEO-F446RE board to the PC via USB. Navigate to <code>Core/Src/main.c</code> in the project explorer to verify the source. Finally, click the <strong>Run</strong> button (Play icon) to compile the code and program the board.
    </p>
  </li>
</ol>

<p align="justify">
To switch between the thermal test and the rotary table calibration, simply change the active Git branch in your terminal. This operation will automatically update the source code within your STM32CubeIDE workspace.
</p>

<p align="justify">
<strong>For Thermal Chamber Testing:</strong>
</p>
<pre><code>git checkout thermal_chamber</code></pre>

<p align="justify">
<strong>For Rotary Table Calibration:</strong>
</p>
<pre><code>git checkout rotary_table</code></pre>

<p align="justify">
After executing the checkout command, return to STM32CubeIDE. Open <code>Core/Src/main.c</code> to verify the change, and click the <strong>Run</strong> button to program the board with the selected test firmware.
</p>

<p align="justify">
Note: The repository defaults to the <code>thermal_chamber</code> branch upon cloning.
</p>

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
