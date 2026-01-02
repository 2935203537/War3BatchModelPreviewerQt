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
  - If a texture cannot be resolved or decoded, a magenta placeholder is used.
- **MPQ support (Warcraft III Classic)** via StormLib.

## Build (Visual Studio 2022)
1. Install **Qt 6 (MSVC kit)** (e.g. Qt 6.6+).
2. Open the folder in **Visual Studio 2022** (CMake project).
3. Configure CMake:
   - Set `CMAKE_PREFIX_PATH` to your Qt install folder, e.g.
     - `C:/Qt/6.6.2/msvc2019_64`
   - or set `Qt6_DIR` to the folder containing `Qt6Config.cmake`.
4. Configure will fetch StormLib (MPQ support) via CMake FetchContent.

## Warcraft III Root + MPQ support
- **War3 Root Path** default: `E:\Warcraft III Frozen Throne`
- The app mounts these MPQs (if present):
  - `War3.mpq`
  - `War3x.mpq`
  - `War3xLocal.mpq`
  - `War3Patch.mpq` (highest priority)
- Status bar shows: `MPQ mounted: N`

## Texture lookup order
1. Model directory
2. Asset root + original path (e.g. `Textures/Foo.blp`)
3. Common folders (Textures, ReplaceableTextures, Units, Doodads, etc.)
4. `war3mapImported/`
5. Basename search (logged as `basename-search`)

## Controls
- **Left drag**: orbit
- **Mouse wheel**: zoom
- **F**: fit-to-bounds
- **W**: wireframe toggle
- **A**: alpha-test toggle (debug)

## Notes
- Texture lookup uses the scanned folder as the asset root.
- Replaceable textures: TeamColor/TeamGlow use built-in placeholders.

## FAQ
- **Model loads but nothing is visible**
  - Check the log and status bar: verts/tris/drawcalls/fps. If verts > 0, use **F** to fit view.
  - Alpha test is off by default; toggle **A** only after mesh is visible.
- **verts=0 / tris=0**
  - Many effect models are particle-only or empty mesh. This is not an error.
- **Missing textures**
  - You will still see the mesh with a magenta placeholder. Check the Missing Textures dock and logs.

## References
- MDX format overview & chunk layout (HiveWorkshop specs)
- Retera's Model Studio (rendering/reference behaviors)
