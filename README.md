# War3BatchModelPreviewerQt

A fast batch preview tool for **Warcraft III (Classic) .mdx models**, built with **Qt 6** + **OpenGL (3.3 Core)**.

This project focuses on **fast static preview** (geometry + basic materials/textures), not full animation or skinning.

## Features
- Scan a folder recursively and list all `.mdx` files.
- OpenGL renderer with orbit camera (drag to rotate, mouse wheel to zoom).
- Supports common MDX geoset primitives: **triangle list / strip / fan / quads**.
- Basic material handling from `MTLS/LAYS`:
  - filter mode (opaque/blend/additive/modulate) and alpha test
  - unshaded / two-sided / no depth test / no depth write flags
- **BLP (War3) textures**: palettized direct content with 0/1/4/8-bit alpha (most Warcraft III textures).
  - If a texture can’t be resolved or decoded, a magenta placeholder is used.

## Build (Visual Studio 2022)
1. Install **Qt 6 (MSVC kit)** (e.g. Qt 6.6+).
2. Open the folder in **Visual Studio 2022** (CMake project).
3. Configure CMake:
   - Set `CMAKE_PREFIX_PATH` to your Qt install folder, e.g.
     - `C:/Qt/6.6.2/msvc2019_64`
   - or set `Qt6_DIR` to the folder containing `Qt6Config.cmake`.

## Notes
- Texture lookup: the app uses the scanned folder as the “asset root”.
  - It first tries the texture path directly (e.g. `Textures/Foo.blp`), then falls back to searching by basename.

## References
- MDX format overview & chunk layout (HiveWorkshop specs)
- Retera's Model Studio (rendering/reference behaviors)
