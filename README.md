## A C++ port of bepuphysics2
This project is an ongoing attempt at porting the excellent physics library [bepuphysics2](https://github.com/bepu/bepuphysics2) to C++.
It is very much work in progress.

Currently only the broad phase is "fully" implemented. Multithreading is not yet supported.

## Demo
An OpenGL based demo is included that shows the broad phase in action in a scene of randomly generated cubes.
An initial bounding volume hierarchy is created and then refitted every frame.

![image](https://github.com/alektron/CepuPhysics/assets/9118389/fe1e2e80-eaad-47d6-bcce-54874249bf93)

## Requirements
Not included in this repository but necessary to build and run the demo:
- Libraries/glew: glew.h and binaries in include, lib, and bin directories
- Libraries/glm: [glm](https://github.com/icaven/glm) math library

## Credits
- [Ross Nordby](https://github.com/RossNordby), creator of [bepuphysics2](https://www.bepuentertainment.com/)
