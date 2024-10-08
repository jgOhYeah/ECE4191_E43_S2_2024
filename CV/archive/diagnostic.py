import sys
import os
import site

def check_module(module_name):
    print(f"\nChecking for {module_name}:")
    try:
        module = __import__(module_name)
        print(f"  Successfully imported {module_name}")
        print(f"  {module_name} path: {module.__file__}")
        if hasattr(module, '__version__'):
            print(f"  {module_name} version: {module.__version__}")
    except ImportError as e:
        print(f"  Failed to import {module_name}: {str(e)}")
    
    for path in sys.path + site.getsitepackages():
        module_path = os.path.join(path, module_name)
        if os.path.exists(module_path):
            print(f"  Found at: {module_path}")
            if os.path.islink(module_path):
                print(f"    It's a symlink to: {os.readlink(module_path)}")
            print(f"    Contents: {os.listdir(module_path)}")

print("Python version:", sys.version)
print("Python executable:", sys.executable)
print("Virtualenv:", sys.prefix)
print("\nPYTHONPATH:", os.environ.get('PYTHONPATH', 'Not set'))
print("\nsys.path:")
for path in sys.path:
    print(f"  {path}")
print("\nsite-packages:")
for path in site.getsitepackages():
    print(f"  {path}")

check_module('libcamera')
check_module('picamera2')