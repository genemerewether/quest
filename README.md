# QUEST

## One-time setup:

You'll need the submodules and dependencies, so clone like so:
```
git clone --recurse-submodules REPO_ADDRESS
```

Or, after cloning, run:
```
git submodule update --init --recursive 
```

## Building quest_gnc with Python bindings (specify version manually)

```
cd quest_gnc
mkdir build && cd build
cmake ../ -DNO_ROS=1 -DPYBIND11_PYTHON_VERSION=2.7
make
```

## Running Jupyter notebook

Add the build folder above to your path:
```
import sys
sys.path.append('../quest_gnc/build27')
```
