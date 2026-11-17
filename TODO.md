# CAMForAlibre to-do list

Work still to do, beyond the build order in `CLAUDE.md`. Decisions about any of
these belong in the spec (`docs\SPEC.md`) once they're made.

- Operations
  - 🔴 V-carve
  - 3D
    - Project finishing
    - 3D offset pocketing and profiling
    - Pencil tracing (idea saved in `docs\PencilMachining.md`)
    - Valley remachining
    - Plateau machining
    - Parallel hill machining
    - Radial machining
    - Spiral machining
    - Curve machining
    - Between 2 curves
- More stock alignment options
- 4th axis: indexing is built (Rotate steps, Set Origins solved for the rotary angle, the 4 axis post, the simulated turn). Still to do: simultaneous 4th axis (wrapping, rotary machining); the APT CL post's rotary output
- 5 axis indexing (3+2): machine kinematics (table-table A/C or B/C, head-head, head-table), pivot offsets, axis limits, choosing between two solutions
- Tilted plane output (G68.2, CYCLE800, PLANE) as a post option, for controls that have it
- Lathe support: machine type, rod stock, turning operations, tools, post and simulation (staged plan saved in `docs\LatheSupport.md`)
- Exact solid (B-rep) in-process stock, probably through Alibre's own kernel (exploring; options in `docs\ExactSolidSimulation.md`)
- Thread milling: external threads (internal threads built, and simulated with the tool's teeth; see the help's Thread Milling tab); perhaps a first tooth height on the tool, if the lowest crest isn't at the tip
- 🔴 Show some summary information under each op, eg tool
- New dialogs should inherit speed/feeds for the last (?)  copy/paste(?)
- Rest machining: built for Waterline Roughing and Pocketing. Still to do: Facing, and pencil passes for finishing
- More posts (GRBL, Mach3/4, UCCNC, Masso)
- Undo in the CAM tree
- Inch display units
- Tool holder from a model: pick an Alibre part (or file) as the holder's shape, used in the simulation in place of the plain cylinder
- Pictures on the dialog tabs showing what each parameter means, where it helps
- Performance
  - Spline faces are now read as NURBS (`GetBSplineCurvesData`) and worked out
    by CamCore. A face whose data can't be read that way (several surfaces in
    one face, or no reading fitting its facets) still asks Alibre a point at a
    time; if that turns up, read those too.

Priority: 🔴 high, 🟡 medium, ⚪ low. Items without a mark aren't prioritised yet.