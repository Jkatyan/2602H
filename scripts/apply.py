import sys
import json


def print_constants(data):
    for unit in data:
        unit_type = type( data[unit] )

        if unit_type == dict:
            print_constants( data[unit] )

        if unit_type == int:
            print( "int " + unit + " = " + str(data[unit]) + ";")

        if unit_type == str:
            print( "char " + unit + "[] = \"" + data[unit] + "\";" )


def print_auton(data):
    print("void load_autonomous(){")
    ind = 0

    for unit in data:
        print( "\tAUTONOMOUS_SEQUENCE[" + str(ind) + "] = Autonomous_Section{" +
                unit["type"]   + ", " + str(unit["length"] ) + ", " +
            str(unit["speed"]) + ", " + str(unit["timeOut"]) + "};")
        ind += 1
    
    print( "\tAUTONOMOUS_SEQUENCE[" + str(ind) + "] = Autonomous_Section{end};")
    print( "}" )


print("Running HYDRA Apply")

data = json.loads( open("config.json", 'r').read() )

sys.stdout = open("src\\config.py_generated.cpp", 'w')


print("#include \"main.h\"")
print()

print_constants( data["constants"] )
print()

print_auton( data["autonomous"] )
