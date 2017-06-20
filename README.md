# Tigrillo simulation and optimization project

## Installation on Ubuntu 16.04
- Download the project:
```
git clone http://github.com/Gabs48/tigrillo/
```
- Install dependencies:
```
# Bullet3
git clone https://github.com/bulletphysics/bullet3
cd bullet3 && mkdir -p build && cd build
cmake -DBUILD_PYBULLET=ON -DBUILD_PYBULLET_NUMPY=OFF -DUSE_DOUBLE_PRECISION=ON -DCMAKE_BUILD_TYPE=Release .. && make -j4 && make install
```
- Compile C++ code:
```
mkdir -p build && cd build && cmake .. && make
```

## Create and load a model
To create the defaults tigrillo.sdf files and tigrillo.mjcf in the models folder, execute:
```
cd src/experiments/ && python create_model.py
```
The morphology configuration can be easily changed in the script itself. To vizualize it in a very simple way, just use:
```
cd src/experiments/ && python create_model.py
```

## Optimizing a controller
To run a optimization, use:
```
cd src/experiments/ && python optimization.py opt [NUMBER_OF_ITERATIONS]
```
The best model is saved in the result folder, you can retrieve the simulation with:
```
cd src/experiments/ && python optimization.py run [FILENAME]
```
