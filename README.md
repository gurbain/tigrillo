# Tigrillo simulation and optimization project

## Installation on Ubuntu 16.04
- Download the project:
```
git clone http://github.com/Gabs48/tigrillo/
```
- Install dependencies:
```
sud
```
- Compile C++:
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
