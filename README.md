# StaticESDFSim

Simple header-only simulator for ESDF from static pointcloud.

## Getting Started

### Dependencies
* [Bonxai](https://github.com/facontidavide/Bonxai/tree/main)
* [nanoflann](https://github.com/jlblancoc/nanoflann)

### Build example
```
mkdir build
cd build

# replace with desired generator
cmake .. -G "Visual Studio 17 2022"
```

### TODO
* Fix crash on StaticEsdfSim destructor when reading from file
* Perform LOS checks when querying ESDF