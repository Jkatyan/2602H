import os
import sys


show_program_info = False


if show_program_info:
    print("\033[5;31;47m[READ THIS BEFORE UPLOADING]\033[0m")
    print("\033[5;30;45mThis program belongs to:\n\
    Team 2602H for VEX 2020 Tower Takeover:\n\
    Alopa, Jay, Justin, Neel, Shishir, Tanya, Tejas, Zack\033[0m")
    print("\033[5;31;47mconfirmed to continue? [Y/N]\033[0m", end = ': ')

if not show_program_info or input()[0].lower() == 'y':
    os.system("python scripts\\apply.py")
    os.system("python scripts\\compile.py")
    os.system("python scripts\\upload.py")
