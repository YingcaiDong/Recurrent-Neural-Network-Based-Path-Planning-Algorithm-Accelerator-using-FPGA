# Recurrent Neural Network Based Path Planning Algorithm Accelerator using FPGA

- This prototype is written in C++ can simulate the robot’s movement in a workspace. The robot's moving trajectory is based on the collision-free path planning algorithm.
- The software allowed the user to simulate different workspace scenarios, which contain different obstacle distributions. 
-	To test this hardware accelerator, I compared the speed between non-hardware-optimized algorithm running time and optimized running time. The comparison’s result was ten times the speed acceleration on FPGA.
-	Using multiple methods to optimize the hardware acceleration (HLS perspective: apply different solutions and compare the data output rate. System perspective: ACP port, AXI4_Stream, DMA).
-	Development tools: Vivado HLS, Vivado, Vivado SDK.
-	Target Hardware: Zynq xc7z020clg484-1(ZedBoard).


There are three folders in this project. They are:
 - HLS
 - System Design Source File
 - Software Driver
 
# HLS
This folder contains three files; these files are used for functional check and generate IP-Core using Vivado High-Level Syntheses software.
# System Design Source File
This folder contains the system design used for this project. When you import this system design, you can choose to replace the hardware acceleration IP-Core with the newly generated one from Vivado HLS. Or you can decide to keep the original one.
# Software Driver
This folder contains all the hardware drivers you will need for the system design.
# Aim & Achieved Function
To simulate the robot movement in a workspace. The robot's moving trajectory is based on the Shunting model which is a collision-free path planning algorithm.

You can input a 32\*32 array. Which only using digits 4, 0, 9 and 1. 
  - 4 stands for the obstacles in the space.
  - 0 stands for the empty space where the robot can free to move.
  - 9 stands for the robot starting position.
  - 1 stands for the desired target location you wish the robot to reach.

Then the input will go through the FPGA and then give you the output which you can see in the software console.

The output is also a 32\*32 array. Which contains digits 4, 0, 9, 1 and 7.
  - The first four digits represent the same meanings as mentions before.
  - 7 stands for the trajectory of the robot traveling from the start point to the target.

# Other
Any other project details can be found in the 'Porject Documentation.pdf'
