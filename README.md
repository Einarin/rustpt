 # RustPT #

Based on http://www.kevinbeason.com/smallpt/ but written in Rust and enhanced. An unbiased monte carlo path tracer.

## Example Render ##
![example.png shows off some of the features of this path tracer](https://github.com/einarin/rustpt/raw/master/example.png "Example Render")
- The spheres in the top back demonstrate varying the metalness (bottom metal, top non-metal) and roughness (smooth on the left to rough on the right)
- The box lower left with varied texture and inset gold demonstrates mesh rendering with texture mapping material properties. Credit to freepbr.com for rusted metal material used.
- the glass teapot and sphere demonstrate refractive materials on analytic spheres and meshes, with physically correct caustics and interaction with the scene
- the red and blue walls demonstrate correct radiosity effects

## LICENSE ##

Copyright (c) 2015 John Dickinson (kc0itv@gmail.com)

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
