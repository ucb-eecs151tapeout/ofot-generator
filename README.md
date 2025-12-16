# OFO Tile Generator

Part of EECS151T's One Fifty One Tile project. 
Refer to [lab instructions](https://github.com/ucb-eecs151tapeout/eecs151t-labs/tree/main) for more information.
For more general info, check out the website: https://151tapeout.berkie.ee/

As of Fall 2024, tested on a single core, not (yet) silicon proven. A test chip was submitted. 

Note that this repo should not contain EECS151 RTL directly, which should stay private. 
Make sure any EECS151 RTL is a submodule on a private repo. 
The Makefile will generate a preprocessed Verilog file which should stay .gitignore'd. 