import os
import sys

PROJECT_ROOT="C:/Users/thanl/Documents/CPP_VS22/CMakeConsole"
sys.path.append(PROJECT_ROOT+"/test/siren/content")

import torch
import modules, utils

model_type = "sine"
mode = "mlp"
resolution = 512
logging_root = PROJECT_ROOT+"/test/siren/logs"
experiment_name = "exp1"
checkpoint_to_load = "1500" #WRITE THE CHECKPOINT YOU WANT TO LOAD HERE
checkpoint_path = f"{PROJECT_ROOT}/test/siren/logs/{experiment_name}/checkpoints/model_epoch_{checkpoint_to_load}.pth"

class SDFDecoder(torch.nn.Module):
    def __init__(self):
        super().__init__()
        # Define the model.
        if mode == 'mlp':
            self.model = modules.SingleBVPNet(type=model_type, final_layer_factor=1, in_features=3)
        elif mode == 'nerf':
            self.model = modules.SingleBVPNet(type='relu', mode='nerf', final_layer_factor=1, in_features=3)
        self.model.load_state_dict(torch.load(checkpoint_path))
        self.model

    def forward(self, coords):
        model_in = {'coords': coords}
        return self.model(model_in)['model_out']


sdf_decoder = SDFDecoder()

#You can begin writing your code in this cell, using the provided rough outline:
#-------------------------------------------------------------------------------

#Load the trajectory points from the file
#...
f = open(f"{PROJECT_ROOT}/test/siren/COLAB CODE/points.txt", "r")
#Build a torch Tensor
#...
Lines = f.readlines()
count = 0
point_array = []
for line in Lines:
  str_cords = line.split()
  x = float(str_cords[0])
  y = float(str_cords[1])
  z = float(str_cords[2])
  p = [x,y,z]
  point_array.insert(count,p)
  count+=1
f.close()
point_tensor = torch.Tensor(point_array)
#Run the model
#...
output_sdf = sdf_decoder(point_tensor)
#Save the output to a file
#...
f2 = open(f"{PROJECT_ROOT}/test/siren/COLAB CODE/sdf.txt","w")
f2.write("")
f2.close()
f3 = open(f"{PROJECT_ROOT}/test/siren/COLAB CODE/sdf.txt","a")
for i in range(0,count-1):
  s = str(output_sdf[i][0])
  temp = s.split("(")
  temp_s = temp[1]
  temp2 = temp_s.split(",")
  final_s = temp2[0]
  string = final_s + "\n"
  f3.write(string)
f3.close()
#Now go back to your original program and use the provided sdf values. If they
#are <0 it means there's a collision!
