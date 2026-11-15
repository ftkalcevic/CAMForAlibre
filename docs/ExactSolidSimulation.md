# Exact solid (B-rep) simulation: options

> **Exploring (2026-11-15).** Frank asked whether full geometric simulation can be done without a third-party library. Nothing is built. If it's picked up, Frank prefers Alibre's own kernel (option B), and doesn't want CAMForAlibre to have its own kernel (option A) or a third-party one (option C).

## Where things stand

The tri-dexel stock (`core\TriDexel`) is exact along each ray: every crossing comes from the tool's true swept shape. Between rays it's approximate: the mesh is a surface net over the crossings, so sharp edges are softened to about one ray spacing, and the error is set by the resolution.

## A. Our own kernel (declined)

Keep the stock as a B-rep and subtract every move's swept volume. For 3-axis moves with our tool types the swept surfaces are few:

| Move | Flat | Ball | Corner radius |
|---|---|---|---|
| Plunge or level line | planes, cylinders | cylinders, spheres | planes, cylinders, tori |
| Sloped line | planes, elliptic cylinders | cylinders, spheres | adds a swept torus (not a standard surface) |
| Level arc | planes, cylinders | tori, spheres | tori |
| Helix | helical sweeps (spline approximation) | same | same |

- 2.5D only (flat mill, level moves): each level's cut is a 2D region of lines and arcs pushed straight down, so exact 2D booleans and planar and cylindrical faces are enough. Weeks of work.
- Full 3D: surface-surface intersection for spheres, cylinders and tori, spline surfaces for the rest, and topology that survives near-tangent cuts. Every finishing pass leaves thousands of thin scallop faces, and each move is a boolean on a growing solid. Months of work, and slow.

## B. Alibre's own kernel, through its API (preferred, if pursued)

What 27.0.1's API has (from the generated headers, all unproven):

- `AddSweptCutout`: sweeps a sketch profile along a path.
- `IADDesignBooleanFeature` with `ADBooleanOperator_AD_SUBTRACTION`: booleans between bodies.
- `AddMeshBooleanFeature`.
- `ImportSTEPFile`.

Its likely use is an "in-process stock part" made after an operation, with the moves combined into features per level or pass, not per move. That gives a real Alibre solid the user can measure, section, export as STEP, or model on. Each feature takes roughly 0.1 to several seconds and runs on Alibre's thread, so it can't be used for live playback.

## C. Third-party kernels (not wanted)

- Open CASCADE: LGPL, free, has booleans and pipe sweeps. Large, and slow for many booleans.
- Parasolid, ACIS, C3D, SMLib: commercial licences.
- Manifold: Apache licence, fast and robust mesh booleans, but exact only as a mesh.

## D. Hybrids

- Option A's 2.5D solid for flat-mill operations, with tri-dexels for 3D finishing.
- An alternative that isn't a B-rep: exact tri-dexel reconstruction. Record each dexel end's cutting move and exact normal, and rebuild the mesh by dual contouring, so edges are sharp and flat and round walls are true. In-house, near exact at the surface, with only slivers thinner than the ray spacing still approximate.

## Open questions, if it's picked up

- What the solid is for: an in-process stock part, exact gouge and remaining-stock checks, or the simulation's view.
- How moves are combined into features (per level, per pass, per operation).
- Whether `AddSweptCutout` accepts a 3D path, and how a tool's profile is swept along it.
- How long Alibre takes per feature, on a real toolpath.
