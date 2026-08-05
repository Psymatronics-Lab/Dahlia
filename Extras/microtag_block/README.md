# MicroTag Blocks
Microtag blocks are a custom fiducial component designed to test CV pipelines and be low cost to print and assemble, while still acting as workable fiducials for small scale testing and localization.

Each block is a 35mm cube, printed from black PLA plastic (or an equivalent material), with 9 smaller 7mm cubes embedded into one face, 5 white and 4 black. Each fiducial pattern is orientable and uniquely identifable by the 3 by 3 pattern which is formed by the 9 smaller cubes. Only a single corner of the larger square contains a white cube, which determines orientation, and the strict limitation of 5 white and 4 black cubes provides error checking.

From these restrictions, 5 unique patterns are possible, of which 3 are unique when removing reflections. These are listed as follows, with `O` being a white block and `X` being a black block.
```text
Pattern 1:
O O X
O X O
X O X

Pattern 2:
O X X       O O X
O O O  -->  X O O
X O X       X O X

Pattern 3:
O O X       O O X
O O X  -->  O O O
X O X       X X X
```

MicroTag Blocks can thus be used as fiducials as well as pick-and-place, trackable objects by camera-enabled robotic arms. They can be easily 3D printed at relatively low cost, stand on their own in any orthogonal orientation without needing additional supports, and be stacked and layered.