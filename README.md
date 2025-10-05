# Interactive Depixelization (Spring-Based Vectorizer)

Implementation of the research paper "Interactive Depixelization of Pixel Art through Spring Simulation" (non-official).
A small C++/OpenCV tool that turns low-res pixel art into clean, editable SVGs by relaxing contour nodes with a spring system and preserving shape area.

## Overview

Given a pixel image, we cluster colors, trace contours, build a spring-mass graph, relax it to smooth the geometry while keeping corners sharp and areas stable, then fit Bézier curves and export to SVG. The UI currently supports crossing disambiguation and adjustment of some simulation parameters.

## Pipeline overview

### 1. **Color Similarity Graph & Clustering**

Build a pixel adjacency graph weighted by color distance; cluster into regions (palette-aware merging).

### 2. **Crossing Disambiguation (Interactive)**

Detect X-junctions/ambiguous crossings; user resolves intended connectivity with lightweight clicks.

![Crossing Disambiguation](path/to/crossing_disambiguation.png)

### 3. **Region Coloring**

Propagate/clean labels for coherent color regions (helps later contour extraction and layering).

### 4. **Path Generation**

Trace region boundaries → ordered boundary loops; construct a graph of **corner** and **edge** nodes for the spring simulation.

### 5. **Spring Simulation (Force-Directed Relaxation)**

   * Edge springs for smoothness (neighbor tension).
   * Corner anchoring to preserve sharp features.
   * **Area preservation:** compute polygon area via the **shoelace formula** and apply a corrective area force (prevents shrinking).
   * (TODO) Interactive “stiffness brush” to locally sharpen/smooth segments; adaptive step size for stable iteration.

### 6. **Bézier Fitting & SVG Export**

Fit cubic Béziers to relaxed polylines; write layered SVG with one path per region.

## Some results

![Results](path/to/results_grid.png)

## Key Features

* **Area-aware smoothing:** spring relaxation that resists shrinkage/ballooning.
* **Corner-safe:** preserves salient corners while smoothing internal edges.
* **User control:** instant crossing fixes (and local stiffness edits - not yet implemented).
* **Clean output:** compact Bézier paths ready for editing in vector tools.

**Technologies:** C++, OpenCV, OpenGL/GLSL.

