3D Modeling Level Sets Project <br />
Uses VVRFramework for graphics <br />
Run down of each part of the project: <br />
A_1) Find the differential coordinates of the mesh object <br />
A_2) Smooth the mesh with shrinking (parameter λ + iterations) <br />
A_3) Smooth the mesh with smart Taubing inflation (λ,μ,i) <br />
![taubin inflation img](resources/taubin.png?raw=true) <br />
In the above image left is the original object and on the right is the object after Taubin smoothing <br />
B_1) Find Signed Distance Function (sphere,AABB)
B_2) Normals
B_3) Collision Detection (with/without SDF) <br />
[Cube movement with rotation](resources/cube.mp4?raw=true) <br />
B_4) Find collision response reaction <br />
To simulate the collision: run the program , select 2 objects, press the keys 'g','s','shift+6','6' <br />
Then go to the terminal and select the method (3&4 are very slow) and after calculations finished press 'y'.<br />
After objects finish moving to contact point press '8','s' and 'shift+8' to show collision response vector.<br />
Finally, press 'space' to see how the objects move after the collision.<br />
Note that the objects may not move as expected since all their mass has been calculated on their surface and methods sphere&AABB have accuracy loss (check them with the appropriate keyboard shortcuts in the UI) <br />
[2 Objects colliding](resources/collision.mp4?raw=true)<br />
B_5) SDF calculation with Siren Neural Network <br />
After training a model with appropriate data for the Armadillo object and successfully recontructing it, the python script sdf_model_create uses the 1500th checkpoint to create a model for calculating the signed dist function faster based on the points it is given by the c++ program (txt files). Note that it is only faster for objects with thousands of points and not for collision detection since it calculates SDF many times (file w/r and python script run overhead). Ideally either the siren model would be transfered after training to c++ or the collision detection algorithm as a whole to the python script. <br />

<br />
To build use Visual Studio: <br />
.exe will be installed in: out\install\x86-Debug\CmakeConsole\CMakeConsole.exe <br /><br />
Note: CMakeLists will copy the VVRFramework dlls into the same folder as the .exe for it to work <br />
