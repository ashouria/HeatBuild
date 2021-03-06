************************ About ************************

HeatBuild is a script in MATLAB which can be used to find the optimial capacity of the heating system for a single building.

The script uses the concept of "simultaneous design and control". This implies that both the design (size of the components) and the control (operation of the component) are optimized at one stage. As a result, the optimality is meaningful. More information can be found in:

- A. Ashouri, "Simultaneous Design and Control of Energy Systems". Doctoral dissertation, Nr. 21908, ETH Zurich, 2014.

************************ System requirements ************************
You need to have the following software or packages installed on your device:
- MATLAB. Download from http://www.mathworks.com/products/matlab/
- Yalmip toolbox. Download from http://users.isy.liu.se/johanl/yalmip/
- GLPKMEX (Matlab mex for GLPK solver). Download from http://glpkmex.sourceforge.net/

************************ Running the script ************************
- Extract the Yalmip package in 'C:\yalmip'
- Copy the GLPK mex-file 'glpkcc.mexw32' (for 32-bit OS) or 'glpkcc.mexw64' (for 64-bit OS) into the foler 'C:\yalmip\solvers'
- Open and run the script 'HeatBuild.m' in MATLAB 

************************ License ************************
As described in the file 'License.txt' 