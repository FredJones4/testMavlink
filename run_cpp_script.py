import subprocess
import time

def compile_cpp_code():
    print("Compiling C++ code...")
    compile_command = ["g++", "-std=c++17", "-o", "drone_control", "drone_control.cpp", "-lmavsdk"]
    result = subprocess.run(compile_command, capture_output=True, text=True)
    if result.returncode != 0:
        print("Compilation failed:")
        print(result.stderr)
        return False
    print("Compilation successful!")
    return True

def run_cpp_code():
    print("Running C++ executable...")
    result = subprocess.run(["./drone_control"], capture_output=True, text=True)
    print("Output:")
    print(result.stdout)
    if result.returncode != 0:
        print("Execution failed:")
        print(result.stderr)

if __name__ == "__main__":
    if compile_cpp_code():
        run_cpp_code()
