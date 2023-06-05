3D Modeling Level Sets Project
To build use Visual Studio:
-change PROJECT_ROOT variable in cpp file to the directory you cloned the project in
-(optional) uncomment python_script variable (& comment old) and replace the first directory with either "py" or the path to your python.exe (!! for this need to pip install torch numpy)
So for example: "py "+PROJECT_ROOT+"/test/siren/sdf_model_create.py"
-build with VS
-.exe will be inside /out/install/x86-Debug/cmakeconsole/bin

Note: CMakeLists is set to copying the VVRFramwork dlls into the same folder as the .exe for it to work