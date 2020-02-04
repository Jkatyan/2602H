import sys
import json


print("Running HYDRA Apply")

data = json.loads( open("config.json", 'r').read() )

sys.stdout = open("src\\apply_test.cpp", 'w')


print("#include \"main.h\"")
