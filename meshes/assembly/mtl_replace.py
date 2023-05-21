import os
import sys

# find *.mtl files in same folder
mtl_files = [f for f in os.listdir('.') if f.endswith('.mtl')]

for mtl in mtl_files:
    # open mtl file and replace last line
    with open(mtl, 'r') as f:
        lines = f.readlines()
        
        if 'map_Kd' in lines[-1] and 'nicky' in lines[-1]:
            # remove strings before the 'maps' in the last line
            print(lines[-1])
            lines[-1] = 'map_Kd maps/' + lines[-1].split('maps\\\\')[1]
            lines[-1] = lines[-1].replace('_8.', '.')
            print(lines[-1])
            # write to file
            with open(mtl, 'w') as f:
                f.writelines(lines)