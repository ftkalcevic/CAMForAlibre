// Materials.h : the shared list of materials a job can be cut from. No Alibre or MFC code belongs here.
//
// The list is one file for every machine, %APPDATA%\CAMForAlibre\materials.json.
// Tools keep feeds per material by the material's ID, so a material can be renamed.

#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "CamJob.h"     // LoadStatus, SaveResult
#include "ToolLibrary.h"    // ToolMaterial

namespace camcore
{
    // Version 2 added each material's cutting values per tool material, version 3 its
    // specific cutting force.
    constexpr int kMaterialsFormatVersion = 3;

    // How a material is cut with tools of one tool material, for the feeds calculator.
    struct MaterialCutting
    {
        bool set = false;
        double surfaceSpeed = 0.0;      // m/min
        double chipLoadFactor = 100.0;  // percent of the tool's chip load

        bool operator==(const MaterialCutting& other) const
        {
            return set == other.set && surfaceSpeed == other.surfaceSpeed && chipLoadFactor == other.chipLoadFactor;
        }
    };

    struct Material
    {
        std::string id;     // stable
        std::string name;
        MaterialCutting cutting[kToolMaterialCount];    // by ToolMaterial

        // The specific cutting force (Kc), N/mm^2: the simulation's power and force
        // come from it. 0 is not known.
        double cuttingForce = 0.0;

        const MaterialCutting& CuttingFor(ToolMaterial material) const { return cutting[static_cast<int>(material)]; }
        MaterialCutting& CuttingFor(ToolMaterial material) { return cutting[static_cast<int>(material)]; }
    };

    struct MaterialList
    {
        std::vector<Material> materials;
    };

    // Returns a sentence for each problem. Empty means the list is valid.
    // Every material needs an ID and a name, and names and IDs are unique
    // (names ignoring case). A cutting value that's set needs a surface speed
    // over 0 and a chip load factor from 1 to 500%. The cutting force can't be negative.
    std::vector<std::string> ValidateMaterials(const MaterialList& list);

    const Material* FindMaterial(const MaterialList& list, const std::string& id);

    struct MaterialsLoadResult
    {
        LoadStatus status = LoadStatus::Unreadable;
        MaterialList list;
        std::string message;    // empty when status is Ok
    };

    MaterialsLoadResult LoadMaterials(const std::filesystem::path& path);
    SaveResult SaveMaterials(const MaterialList& list, const std::filesystem::path& path);
}
