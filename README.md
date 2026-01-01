# War3 Batch Model Previewer (Qt + VS2022)

A lightweight batch preview tool for Warcraft III **MDX** models:
- Pick a folder and it lists `.mdx` files
- Click a file to preview its mesh in a fast OpenGL viewer
- Mouse controls: orbit / pan / zoom
- Keyboard: ↑/↓ to move selection, F to frame model

> Notes
- This is a **minimal MDX mesh previewer**: it loads the first geoset's vertices + triangles and renders a basic shaded mesh.
- Materials, textures, particles, bones, and animations are **not implemented** (yet). For batch inspection and quick sanity checks, the mesh preview is still very useful.

## Build (Visual Studio 2022)

### Prerequisites
- Visual Studio 2022 (Desktop development with C++)
- Qt 6.x for MSVC (e.g. `C:\Qt\6.6.2\msvc2019_64` or `msvc2022_64`)
  - Make sure Qt includes: Widgets, OpenGLWidgets, Concurrent

### Option A: Open Folder (CMake)
1. Open VS2022 → **File → Open → Folder...** → select this project folder.
2. Configure CMake:
   - Set `CMAKE_PREFIX_PATH` to your Qt install path.
   - You can edit `CMakePresets.json` or configure in VS UI.
3. Build and Run.

### Option B: Command line
```bat
cmake -S . -B out\build -G "Visual Studio 17 2022" -A x64 -DCMAKE_PREFIX_PATH="C:\Qt\6.6.2\msvc2019_64"
cmake --build out\build --config RelWithDebInfo
out\build\RelWithDebInfo\War3BatchModelPreviewerQt.exe
```

## Controls
- **Left Mouse Drag**: Orbit camera
- **Right Mouse Drag**: Pan camera
- **Mouse Wheel**: Zoom
- **F**: Frame / fit model
- **↑ / ↓**: Change selection in list

## Roadmap Ideas
- Load multiple geosets, wireframe toggle
- MDL (text) support
- Simple animation playback (geoset/sequence)
- Texture preview and team color
- Generate thumbnails cache
