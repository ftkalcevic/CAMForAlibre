// TriDexel.h : the simulation's stock as three grids of rays (tri-dexels). No Alibre or MFC code belongs here.
//
// Everything is in the assembly's coordinate system, in millimetres. The stock box
// holds three grids of rays at one spacing, along X, Y and Z. Each ray keeps the
// exact stretches of it that are inside the stock. The rays lie on the lines of one
// cubic lattice whose nodes are at the centres of the cells a voxel grid of the same
// spacing would have, so every lattice edge lies along exactly one ray.

#pragma once

#include <cstdint>
#include <vector>

#include "Stock.h"

namespace camcore
{
    class VoxelStock;
    struct ToolProfile;
    struct StockMesh;

    // The dexel spacings the Resolution slider offers in tri-dexel mode, fine to
    // coarse: those whose largest grid has at most maxRays rays, at most seven.
    std::vector<double> DexelResolutionStops(const StockBox& box, double maxRays = 4.0e6);

    class TriDexelStock
    {
    public:
        static constexpr int kChunk = 32;   // lattice nodes along each side of a chunk, which is meshed on its own

        struct Interval
        {
            float enter;
            float exit;
        };

        // An empty stock over the box, its rays this far apart.
        void Reset(const StockBox& box, double spacing);
        void FillAll();

        // Fills each ray where it's inside a closed triangle mesh (three points to a
        // triangle), from its crossings with the triangles.
        void FillMesh(const std::vector<Vec3>& triangles);

        const StockBox& Box() const { return m_box; }
        const Vec3& Lower() const { return m_lower; }
        double Spacing() const { return m_spacing; }

        // Lattice nodes along an axis (0 X, 1 Y, 2 Z), and the box's length along it.
        int Size(int axis) const { return m_n[axis]; }
        double Length(int axis) const { return m_length[axis]; }

        // A ray along an axis is found by its node numbers along the other two: U, then V.
        static int U(int axis) { return (axis + 1) % 3; }
        static int V(int axis) { return (axis + 2) % 3; }
        size_t RayCount(int axis) const { return m_rays[axis].size(); }
        size_t RayIndex(int axis, int iu, int iv) const
        {
            return static_cast<size_t>(iv) * static_cast<size_t>(m_n[U(axis)]) + static_cast<size_t>(iu);
        }

        // A ray's stretches inside the stock, in order, in mm along it from the box's lower corner.
        void Intervals(int axis, size_t ray, std::vector<Interval>& out) const;

        // How much of a ray's stock lies between a and b.
        double Overlap(int axis, size_t ray, double a, double b) const;
        bool AnyStock(int axis, size_t ray, double a, double b) const;

        // Removes a to b from a ray, returning the length removed. Doesn't mark chunks.
        double Subtract(int axis, size_t ray, double a, double b);

        // Whether a lattice node is inside, from its Z ray. Outside the lattice it isn't.
        bool Inside(int i, int j, int k) const;

        // Where a ray leaves (or enters) the stock between a and b along it, or a
        // negative number if it doesn't.
        double Crossing(int axis, size_t ray, double a, double b, bool leaving) const;

        // The stock's volume, from the Z rays (each a spacing square).
        double Volume() const;
        size_t IntervalCount() const;
        size_t MemoryBytes() const;

        // The stock as voxels at the same spacing: a cell is filled where its centre,
        // a lattice node, is inside its Z ray.
        void ToVoxels(VoxelStock& voxels) const;

        // The chunks, and which have changed since they were last meshed.
        int ChunksAlong(int axis) const { return (m_n[axis] + kChunk - 1) / kChunk; }
        int ChunkCount() const { return ChunksAlong(0) * ChunksAlong(1) * ChunksAlong(2); }
        bool ChunkDirty(int chunk) const { return m_dirty[static_cast<size_t>(chunk)] != 0; }
        void SetChunkClean(int chunk) { m_dirty[static_cast<size_t>(chunk)] = 0; }
        void MarkAllDirty();

        // Marks the chunks whose surface may change when the lattice nodes from
        // (i0, j0, k0) to (i1, j1, k1) do: those holding them and their neighbours.
        void MarkDirtyNodes(int i0, int j0, int k0, int i1, int j1, int k1);

        // Marks the chunks a change to a ray from a to b along it can reach.
        void MarkDirtyRay(int axis, int iu, int iv, double a, double b);

    private:
        struct Node
        {
            float enter;
            float exit;
            std::int32_t more;      // the next stretch in m_pool, or -1
        };

        static bool Empty(const Node& node) { return node.enter > node.exit; }
        void SetIntervals(int axis, size_t ray, const Interval* list, size_t count);
        std::int32_t Allocate();

        template <typename F>
        void Walk(int axis, size_t ray, F f) const
        {
            const Node* node = &m_rays[axis][ray];
            if (Empty(*node))
                return;
            for (;;)
            {
                if (!f(node->enter, node->exit))
                    return;
                if (node->more < 0)
                    return;
                node = &m_pool[static_cast<size_t>(node->more)];
            }
        }

        StockBox m_box;
        Vec3 m_lower;
        double m_spacing = 1.0;
        int m_n[3] = { 0, 0, 0 };
        double m_length[3] = { 0.0, 0.0, 0.0 };
        std::vector<Node> m_rays[3];        // each ray's first stretch
        std::vector<Node> m_pool;           // the stretches after the first
        std::vector<std::int32_t> m_free;   // pool entries no longer used
        std::vector<Interval> m_scratch;
        std::vector<char> m_dirty;
    };

    // Removes the stock inside the tool as its tip moves from `from` to `to` along a
    // straight line, the tool pointing along `axis` (a unit vector from the tip up
    // the tool). Returns the length of ray removed, in mm.
    //
    // A ray along the tool's axis or square to it is cut exactly by the tool's
    // whole sweep. A ray at a slant to it (a tilted tool) is cut in short steps,
    // each the tool's shape at its lower end swept square to the axis.
    double CutDexels(TriDexelStock& stock, const ToolProfile& profile, const Vec3& from, const Vec3& to,
                     const Vec3& axis);

    // Whether the tool would sweep through more than `tolerance` of any ray's stock in that move.
    bool DexelsTouch(const TriDexelStock& stock, const ToolProfile& profile, const Vec3& from, const Vec3& to,
                     const Vec3& axis, double tolerance = 1e-3);

    // A chunk's surface, as surface nets: a vertex in each lattice cell the surface
    // passes through, at the middle of where the rays along its edges cross it, and a
    // quad across each lattice edge it crosses. Chunks meet without gaps. With
    // mergeFlats, quads lying flat on a lattice plane are merged into rectangles,
    // which leaves T-junctions where they meet the rest (fine to draw); without, the
    // mesh is closed, each edge shared by two triangles (for files).
    StockMesh MeshDexelChunk(const TriDexelStock& stock, int chunk, bool mergeFlats = true);

    // Meshes the listed chunks into their slots of meshes, on all cores.
    void MeshDexelChunks(const TriDexelStock& stock, const std::vector<int>& chunks, std::vector<StockMesh>& meshes,
                         bool mergeFlats = true);

    // Every chunk that has changed since it was last meshed.
    std::vector<int> DirtyDexelChunks(const TriDexelStock& stock);

    // The whole stock's surface.
    StockMesh MeshDexelStock(const TriDexelStock& stock, bool mergeFlats = true);
}
