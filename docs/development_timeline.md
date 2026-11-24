Project Development Timeline

Phase 1 – Mechanical Assembly and Hardware Setup
1. Chassis Assembly: Assembled the TI-RSLK MAX chassis, attached both drive wheels, and installed the castor wheel to complete the mechanical assembly.

2. Encoder Wiring and Pin Configuration: Identified the encoder signal connections and determined the required pin configuration. Soldered the necessary header pins to establish reliable electrical connections.

3. Arduino and Motor Driver Integration: Connected the Arduino Uno to the motor driver, DC motors, and wheel encoders, establishing the hardware interface for robot control and feedback.

Phase 2 – Motor and Encoder Characterization

4. Encoder Reading Verification: Developed and tested the encoder-reading functionality to obtain tick counts from both wheels. Verified encoder responses and configured the tick-to-distance conversion based on the wheel dimensions and encoder resolution.

5. Motor Direction and Speed Testing: Tested individual motor operation to verify rotational direction and functionality. Evaluated motor speed behavior and established suitable operating parameters.

6. Straight-Line Motion Control: Implemented encoder-based closed-loop speed control using a PI/PID-based approach. Gradually tuned the controller through repeated testing and parameter adjustments to minimize differences between the left and right wheel speeds and improve straight-line motion.


Phase 3 – Motion Sequence Implementation
7. Timed Stop-and-Go Sequence: Implemented the required motion sequence in which the robot drives for 3 seconds, stops for 1 second, and repeats this sequence for three stops before continuing forward.

8. Speed Variation and Operating Modes: Introduced different speed settings to accommodate the robot's operating requirements, including normal forward motion and reduced-speed operation during obstacle-related tasks.


Phase 4 – Obstacle Detection
9. Ultrasonic Sensor Integration: Connected the HC-SR04 ultrasonic sensor to the Arduino Uno and developed the distance-measurement functionality. Verified sensor readings through practical testing.

10. Obstacle Detection Threshold Configuration: Established and tested the obstacle trigger distance using ultrasonic measurements. Configured suitable detection, safety-stop, and resume thresholds to control the robot's response to nearby obstacles.


Phase 5 – Obstacle Avoidance Development
11. Research and Evaluation of Obstacle Avoidance Methods: Researched existing obstacle avoidance approaches and evaluated their suitability for the robot's differential-drive configuration, available sensors, and project requirements.

12. Selection of Obstacle Avoidance Strategy: Selected a Predefined Encoder-Based Right-Side Obstacle Avoidance Manoeuvre. Developed a predetermined sequence of turns and linear movements to navigate around an obstacle and return to the original path.

13. Integration with Straight-Line Motion Control: Integrated the obstacle avoidance sequence with the existing encoder-based motion-control system, ensuring that the robot could transition between normal driving, obstacle avoidance, and resumed straight-line movement.
Phase 6 – Mechanical Improvements and 3D Printing

14. Additional Chassis Layer Design and Fabrication: Designed and 3D printed an additional mounting layer for the chassis to accommodate the project's hardware and improve component arrangement.

15. Ultrasonic Sensor Mount Design and Fabrication: Designed and 3D printed a dedicated mounting bracket for the ultrasonic sensor to provide a stable and suitable mounting position on the robot.

Suggested GitHub commit history
For GitHub, I would keep the commit messages shorter and action-oriented. You can use the following as a chronological commit sequence:


No.	Suggested commit message
01	Initial TI-RSLK MAX chassis assembly
02	Configure and solder encoder connections
03	Integrate Arduino Uno with motor driver and encoders
04	Implement and verify encoder readings
05	Test motor direction and speed
06	Implement encoder-based closed-loop speed control
07	Tune PI controller for straight-line motion
08	Implement 3-second drive and 1-second stop sequence
09	Introduce variable speed operating modes
10	Integrate and test HC-SR04 ultrasonic sensor
11	Configure obstacle detection and safety thresholds
12	Research and select obstacle avoidance strategy
13	Implement predefined right-side obstacle avoidance
14	Integrate obstacle avoidance with straight-line control
15	Design and print additional chassis mounting layer
16	Design and print ultrasonic sensor mount