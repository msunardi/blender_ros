The stream.py script needs to be run by Blender
either through the command line or Blender GUI

Python version: 3.4.3
Blender Python version: 3.5.1
Blender version: 2.69 (Linux), 2.78 (Windows)

Requires:
- Redis (sudo apt-get install redis)
- Redis-py for Python3 (pip install redis-py)

Install Redis Python module into Blender Python:
- copy redis directories (redis/, redis-2.10.5.dist-info/) to Blender Python  library:
  - Linux: /usr/share/blender/scripts/addons/modules
    (See: https://www.blender.org/manual/getting_started/installing/configuration/directories.html#path-layout)
  - Windows: C:\Program Files\Blender Foundation\Blender\2.78\python\lib
