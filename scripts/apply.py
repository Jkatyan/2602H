import json
import os
import sys


target = "config_hydra_2.0.1"


print("Running HYDRA Apply: applying \"" + target + "\"")


class NonChangeException(Exception):
    pass


def print_constants(data):
    for unit in data:
        unit_type = type( data[unit] )

        if unit_type == dict:
            print_constants( data[unit] )

        if unit_type == int:
            print( "int " + unit + " = " + str(data[unit]) + ";")

        if unit_type == float:
            print( "double " + unit + " = " + str(data[unit]) + ";" )

        if unit_type == str:
            print( "char " + unit + "[] = \"" + data[unit] + "\";" )


def print_auton(data):
    print("void load_autonomous(){")
    ind = 0

    for unit in data:
        print("\tAUTONOMOUS_SEQUENCE[" + str(ind) + "] = Autonomous_Section{" +
                unit["type"]   + ", " + str(unit["length"] ) + ", " +
            str(unit["speed"]) + ", " + str(unit["timeOut"]) + "};")
        ind += 1
    
    print( "\tAUTONOMOUS_SEQUENCE[" + str(ind) + "] = Autonomous_Section{end};")
    print("}")


def print_keyBindings(data):
    print("void load_keyBindings(){")
    ind = 0

    for unit in data:
        print("\tKEY_MAP[" + str(ind) + "] = Key_Binding{" + unit + ", E_HYDRA_" +
                data[unit][0] + ", E_HYDRA_" + data[unit][1] + "};")
        ind += 1
    
    print( "\tKEY_MAP[" + str(ind) + "] = Key_Binding{end};" )
    print("}")

try:
    data = json.loads( open("configs\\" + target + ".json", 'r').read() )

    try:
        binary_data = json.loads( str(open("bin\config.bin", 'rb').read(), "ascii") )
        if data == binary_data:
            raise NonChangeException
    except FileNotFoundError:
        pass
    
    stdout = sys.stdout
    sys.stdout = open("src\\config.py_generated.cpp", 'w')

    print("#include \"main.h\"")
    print()

    print_constants( data["constants"] )
    print()

    #print_auton( data["autonomous"] )
    #print()
    #fix later

    #print_keyBindings( data["key_bindings"] )
    #fix later

    if not os.path.exists("bin"):
        os.makedirs("bin")

    open("bin\\config.bin", 'wb').write( bytes(json.dumps(data), "ascii") )

    sys.stdout = stdout
    print("HYDRA Apply: changes applied \033[5;32m[DONE]\033[0m")

except FileNotFoundError as error:
    print("\033[5;31m[HYDRA Apply error]\033[0m: config \"" + target + "\" not found")
    raise error

except NonChangeException:
    print("HYDRA Apply: nothing changed")
