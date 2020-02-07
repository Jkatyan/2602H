import sys
import json


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
        print( "\tAUTONOMOUS_SEQUENCE[" + str(ind) + "] = Autonomous_Section{" +
                unit["type"]   + ", " + str(unit["length"] ) + ", " +
            str(unit["speed"]) + ", " + str(unit["timeOut"]) + "};")
        ind += 1
    
    print( "\tAUTONOMOUS_SEQUENCE[" + str(ind) + "] = Autonomous_Section{end};")
    print( "}" )


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

    print_auton( data["autonomous"] )

    open("bin\config.bin", 'wb').write( bytes(json.dumps(data), "ascii") )

    sys.stdout = stdout
    print("HYDRA Apply: success")

except FileNotFoundError as error:
    print("\033[0;31m[HYDRA Apply error]\033[0m: config \"" + target + "\" not found")
    raise error

except NonChangeException:
    print("HYDRA Apply: nothing changed")
