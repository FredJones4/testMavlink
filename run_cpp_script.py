import subprocess
import os

def compile_cpp_code():
    print("Compiling C++ code...")
    include_dir = "."  # Current directory where plugin_base.h is located
    mavsdk_lib_dir = "~/usr/local/lib"  # Directory where mavsdk library might be installed

    compile_command = [
        "g++",
        "-std=c++17",
        "-o", "drone_control",
        "-I", include_dir,  # Include directory for header files
        "-I", "/usr/local/include",  # Also include the system-wide directory
        "drone_control.cpp",
        "-L", mavsdk_lib_dir,  # Library directory
        "-lmavsdk"  # Link against the mavsdk library
    ]

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
