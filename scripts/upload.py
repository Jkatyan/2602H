import os

name = "Hydra V2"
slot = 2

print("Running HYDRA Upload: As \"" + name + "\" at slot " + str(slot) + ":")
os.system("prosv5 upload --name \"" + name + "\" --slot " + str(slot) )
