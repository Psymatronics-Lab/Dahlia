# MicroTag Blocks
Microtag blocks are a custom fiducial component designed to test CV pipelines and be low cost to print and assemble, while still acting as workable fiducials for small scale testing and localization.

Each block is a 35mm cube, printed from white PLA plastic (or an equivalent material), with a 25mm frame of black PLA plastic embedded into all six faces, containing a 15mm square of 5mm pieces in a 3 by 3 format. Each 3 by 3 contains 5 white adn 4 black pieces, and each fiducial pattern is orientable and uniquely identifiable by the 3 by 3 pattern. Only a single corner of the larger 3 by 3 square contains a white or black cube (while the other 3 corners contain opposing colors), and this corner determines the pattern orientation. Each face of the MicroTag block contains a different 3 by 3 pattern. 

From these restrictions, a few unique patterns are possible. We select 6 to be our canonical side patterns, with 3 having white corner orienations and 3 having black corner orientations. These are listed as follows, with `O` being a white block and `X` being a black block.
```text
Pattern 1:
O O X
O X O
X O X

Pattern 2:
X O O
O X X
O X O

Pattern 3:
O X X
O O O
X O X

Pattern 4:
X O O
X O X
O X O

Pattern 5:
O O X
O O X
X O X

Pattern 6:
X X O
O X O
O X O
```

MicroTag Blocks can thus be used as fiducials as well as pick-and-place, trackable objects by camera-enabled robotic arms. They can be easily 3D printed at relatively low cost, stand on their own in any orthogonal orientation without needing additional supports, be localized and posed when viewed from any camer orientation, and be stacked and layered.