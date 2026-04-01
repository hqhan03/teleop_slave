import os
import glob

cpp_dir = '/home/nrel/curobo/src/curobo/curobolib/cpp'
files = glob.glob(f"{cpp_dir}/**/*.h", recursive=True) + glob.glob(f"{cpp_dir}/**/*.cu", recursive=True) + glob.glob(f"{cpp_dir}/**/*.cpp", recursive=True)

for file in files:
    with open(file, 'r') as f:
        content = f.read()
    
    # Rename the definition in helper_math.h
    if "helper_math.h" in file:
        content = content.replace('inline __device__ __host__ float lerp(float a, float b, float t)', 'inline __device__ __host__ float curobo_lerp(float a, float b, float t)')
    
    # Rename all calls
    content = content.replace(' lerp(', ' curobo_lerp(')
    content = content.replace('(lerp(', '(curobo_lerp(')
    content = content.replace('=lerp(', '=curobo_lerp(')
    
    with open(file, 'w') as f:
        f.write(content)
print(f"Patched {len(files)} files to rename lerp to curobo_lerp")
