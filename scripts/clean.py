import os

print("Running HYDRA Clean...")
if os.system("rmdir bin /q /s") == 0:
    print("HYDRA Clean: binary file cleaned \033[5;32m[DONE]\033[0m")
