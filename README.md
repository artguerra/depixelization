# Interactive Depixelization (Spring-Based Vectorizer)

Non-official implementation of the research paper "Interactive Depixelization of Pixel Art through Spring Simulation".
A small C++/OpenCV tool that turns low-res pixel art into clean, editable SVGs by relaxing contour nodes with a spring system and preserving shape area.

## Overview

Given a pixel image, we cluster colors, trace contours, build a spring-mass graph, relax it to smooth the geometry while keeping corners sharp and areas stable, then fit Bézier curves and export to SVG. The UI currently supports crossing disambiguation and adjustment of some simulation parameters.

## Pipeline overview

### 1. **Color Similarity Graph & Clustering**

Build a pixel adjacency graph weighted by color distance; cluster into regions.

**Similarity graph**
<img width="1262" height="1219" alt="image" src="https://github.com/user-attachments/assets/de74692b-8349-4690-8f63-2efa0970fd96" />

### 2. **Crossing Disambiguation (Interactive)**

Detect X-junctions/ambiguous crossings; user resolves intended connectivity by clicking on the crossing.

**Ambiguous crossing editing interface:**
<img width="1025" height="676" alt="image" src="https://github.com/user-attachments/assets/f9d8c753-ea59-4e00-bb3d-7421ff7268cb" />

**Comparison in results between different crossings resolutions:**
| Default algorithm resolution  |  Toggled crossing (top-left)  |
:-------------------------:|:-------------------------:
<img width="1525" height="1057" alt="image" src="https://github.com/user-attachments/assets/7e5a0dd7-227a-41b3-9168-f5408bdd3895" />  |  <img width="1525" height="1057" alt="image" src="https://github.com/user-attachments/assets/61e71f91-fcb2-4651-aaf9-bbe58d520110" />

### 3. **Region Coloring**

Propagate/clean labels for coherent color regions (helps later contour extraction and layering).

### 4. **Path Generation**

Trace region boundaries → ordered boundary loops; construct a graph of **corner** and **edge** nodes for the spring simulation.

**Path graph visualization (after spring relaxation):**
<img width="1262" height="1219" alt="image" src="https://github.com/user-attachments/assets/99639f23-aef1-4622-b38f-9a961e423f06" />

### 5. **Spring Simulation (Force-Directed Relaxation)**

   * Edge springs for smoothness (neighbor tension).
   * Corner anchoring to preserve sharp features.
   * **Area preservation:** compute polygon area via the **shoelace formula** and apply a corrective area force (prevents shrinking).
   * (TODO) Interactive “stiffness brush” to locally sharpen/smooth segments; adaptive step size for stable iteration.

### 6. **Bézier Fitting & SVG Export**

Fit cubic Béziers to relaxed polylines; write layered SVG with one path per region.

## Some results

| Original pixel art  |  Vectorized result (only algorithm - no user input)  |
:-------------------------:|:-------------------------:
| <img width="949" height="949" alt="image" src="https://github.com/user-attachments/assets/af797894-104a-466c-82e3-56db24029a15" />  |  <img width="949" height="949" alt="image" src="https://github.com/user-attachments/assets/6a7baa48-6da3-4f21-99be-3200dee0a1c4" />  |
|  <img width="949" height="949" alt="image" src="https://github.com/user-attachments/assets/e0706de4-7932-4bd2-b286-c616cdc1f392" /> |  <img width="949" height="949" alt="image" src="https://github.com/user-attachments/assets/b42728d8-4e4b-4408-8c7c-53415e3bde7f" />  |
|  <img width="949" height="949" alt="image" src="https://github.com/user-attachments/assets/6fd4c75e-a655-4cf9-8d5f-4dc7938ae0d3" />  |  <img width="949" height="949" alt="image" src="https://github.com/user-attachments/assets/24c4677a-999b-4269-ad2f-af47ee91e085" />  |


## Key Features

* **Area-aware smoothing:** spring relaxation that resists shrinkage/ballooning.
* **Corner-safe:** preserves salient corners while smoothing internal edges.
* **User control:** instant crossing fixes (and local stiffness edits - not yet implemented).
* **Clean output:** compact Bézier paths ready for editing in vector tools.

**Technologies:** C++, OpenCV, OpenGL/GLSL.

