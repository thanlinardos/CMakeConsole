#You can begin writing your code in this cell, using the provided rough outline:
#-------------------------------------------------------------------------------

#Load the trajectory points from the file
#...
f = open("C:/Users/thanl/Documents/GEOMETRIA/2022/Model_load_with_Anim/test/siren/COLAB CODE/points.txt", "r")
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
print(point_array)
point_tensor = torch.Tensor(point_array).cuda()
#Run the model
#...
output_sdf = sdf_decoder(point_tensor)
print(output_sdf)
#Save the output to a file
#...
f2 = open("C:/Users/thanl/Documents/GEOMETRIA/2022/Model_load_with_Anim/test/siren/COLAB CODE/sdf.txt","w")
f2.write("")
f2.close()
f3 = open("C:/Users/thanl/Documents/GEOMETRIA/2022/Model_load_with_Anim/test/siren/COLAB CODE/sdf.txt","a")
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