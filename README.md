## Overview
I follow the book _Curves and Surfaces for CAGD: A Practical Guide_, 4th edition by Gerald Farin.

The currently implemented functionality includes:
- The de Casteljau algorithm for finding a point on a Bezier curve given its control polygon (Chapter 3.2).
- The blossoming algorithm, which is like the de Casteljau algorithm but with different weights at
different iterations (Chapter 3.4).
- The subdivision procedure (which also includes extrapolation) (Chapter 4.6).

## Code Structure
There is a `Geometry` namespace. Inside it is the `Bezier` namespace, which is defined
in the `geometry/bezier/deCasteljau.h` header (named after the de Casteljau algorithm,
which occurs in every function in this header).

The `src/main.cpp` file currently includes some basic tests for the `Bezier` namespace.
This file may be compiled with the following command from the root directory:
```
  g++ -std=c++1z src/main.cpp -o bin/test
```

The -std=c++1z option is necessary because I use the
```
namespace Geometry::Bezier {
...
}
```
syntax in `deCasteljau.h` to declare a nested namespace, and possibly for some other
reasons too.
