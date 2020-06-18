# PcbDecompaction

Printed Circuit Board (PCB) decompactor

### Prerequisites

- GCC >=4.8
- G++ >= 4.8
- SWIG >= 2.0
- Boost >= 1.6
- CMake >= 3.1
- Current support for .kicad_pcb format derived from KiCad v5.1.2

### Installing

Clone
```
git clone --recurse-submodules https://github.com/The-OpenROAD-Project/PcbDecompaction.git
```

Build
```
mkdir build
cd build
cmake ..
make
```

Run
```
mkdir output
./bin/pcbdrc [input_filename].kicad_pcb 
```



## Status
Interaction between router and decompactor is not done yet.


## License
  * BSD-3-clause License [[Link]](LICENSE)
