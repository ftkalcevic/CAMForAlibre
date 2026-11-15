// Materials.cpp : the shared list of materials a job can be cut from.

#include "Materials.h"

#include <cctype>
#include <set>

#include <nlohmann/json.hpp>

#include "FileIO.h"

namespace camcore
{
    namespace
    {
        namespace fs = std::filesystem;
        using nlohmann::json;

        // The add-on was once called AlibreCAM; the files keep this name so they still load.

        const char* const kFormatName = "AlibreCAM materials";

        MaterialsLoadResult LoadFailure(LoadStatus status, const std::string& message)
        {
            MaterialsLoadResult result;
            result.status = status;
            result.message = message;
            return result;
        }

        std::string LowerCase(std::string text)
        {
            for (char& c : text)
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            return text;
        }
    }

    std::vector<std::string> ValidateMaterials(const MaterialList& list)
    {
        std::vector<std::string> problems;
        std::set<std::string> ids;
        std::set<std::string> names;
        for (const Material& material : list.materials)
        {
            if (material.id.empty())
                problems.push_back("A material has no ID.");
            if (material.name.empty())
                problems.push_back("A material needs a name.");
            if (!material.id.empty() && !ids.insert(material.id).second)
                problems.push_back("Two materials have the same ID.");
            if (!material.name.empty() && !names.insert(LowerCase(material.name)).second)
                problems.push_back("There's already a material called " + material.name + ".");
            for (ToolMaterial toolMaterial : ToolMaterials())
            {
                const MaterialCutting& cutting = material.CuttingFor(toolMaterial);
                if (!cutting.set)
                    continue;
                const std::string what = material.name + "'s " + ToolMaterialTitle(toolMaterial);
                if (!(cutting.surfaceSpeed > 0.0))
                    problems.push_back(what + " surface speed must be more than 0.");
                if (!(cutting.chipLoadFactor >= 1.0 && cutting.chipLoadFactor <= 500.0))
                    problems.push_back(what + " chip load factor must be from 1 to 500%.");
            }
        }
        return problems;
    }

    const Material* FindMaterial(const MaterialList& list, const std::string& id)
    {
        for (const Material& material : list.materials)
        {
            if (material.id == id)
                return &material;
        }
        return nullptr;
    }

    MaterialsLoadResult LoadMaterials(const fs::path& path)
    {
        std::error_code error;
        if (!fs::exists(path, error))
            return LoadFailure(LoadStatus::NotFound, "File not found: " + path.u8string());

        std::string text;
        if (!ReadWholeFile(path, text))
            return LoadFailure(LoadStatus::Unreadable, "Could not read " + path.u8string());

        const json root = json::parse(text, nullptr, false);
        if (root.is_discarded())
            return LoadFailure(LoadStatus::Unreadable, "Not valid JSON: " + path.u8string());
        if (!root.is_object())
            return LoadFailure(LoadStatus::NotACamFile, "Not a materials list: " + path.u8string());

        const auto format = root.find("format");
        if (format == root.end() || !format->is_string() || format->get<std::string>() != kFormatName)
            return LoadFailure(LoadStatus::NotACamFile, "Not a materials list: " + path.u8string());

        const auto version = root.find("formatVersion");
        if (version == root.end() || !version->is_number_integer() || version->get<int>() < 1)
            return LoadFailure(LoadStatus::NotACamFile, "The format version is missing or invalid: " + path.u8string());
        if (version->get<int>() > kMaterialsFormatVersion)
        {
            return LoadFailure(LoadStatus::NewerVersion,
                               "This materials list uses format version " + std::to_string(version->get<int>()) +
                                   ", but this add-on supports up to version " +
                                   std::to_string(kMaterialsFormatVersion) + ".");
        }

        MaterialsLoadResult result;
        try
        {
            for (const json& item : root.at("materials"))
            {
                Material material;
                material.id = item.at("id").get<std::string>();
                material.name = item.at("name").get<std::string>();

                // Version 1 has no cutting values.
                const auto cutting = item.find("cutting");
                if (cutting != item.end())
                {
                    for (auto entry = cutting->begin(); entry != cutting->end(); ++entry)
                    {
                        ToolMaterial toolMaterial;
                        if (!ParseToolMaterial(entry.key(), toolMaterial))
                            continue;   // a tool material from a newer build
                        MaterialCutting& value = material.CuttingFor(toolMaterial);
                        value.set = true;
                        value.surfaceSpeed = entry.value().at("surfaceSpeed").get<double>();
                        value.chipLoadFactor = entry.value().at("chipLoadFactor").get<double>();
                    }
                }
                result.list.materials.push_back(material);
            }
        }
        catch (const std::exception& problem)
        {
            return LoadFailure(LoadStatus::NotACamFile, "The materials list is damaged: " + std::string(problem.what()));
        }

        const std::vector<std::string> problems = ValidateMaterials(result.list);
        if (!problems.empty())
            return LoadFailure(LoadStatus::NotACamFile, problems.front());

        result.status = LoadStatus::Ok;
        return result;
    }

    SaveResult SaveMaterials(const MaterialList& list, const fs::path& path)
    {
        SaveResult result;
        const std::vector<std::string> problems = ValidateMaterials(list);
        if (!problems.empty())
        {
            result.message = problems.front();
            return result;
        }

        json materials = json::array();
        for (const Material& material : list.materials)
        {
            json item = { { "id", material.id }, { "name", material.name } };
            json cutting = json::object();
            for (ToolMaterial toolMaterial : ToolMaterials())
            {
                const MaterialCutting& value = material.CuttingFor(toolMaterial);
                if (value.set)
                {
                    cutting[ToolMaterialText(toolMaterial)] =
                        { { "surfaceSpeed", value.surfaceSpeed }, { "chipLoadFactor", value.chipLoadFactor } };
                }
            }
            if (!cutting.empty())
                item["cutting"] = cutting;
            materials.push_back(item);
        }
        const json root =
        {
            { "format", kFormatName },
            { "formatVersion", kMaterialsFormatVersion },
            { "materials", materials }
        };
        result.ok = WriteFileAtomically(path, root.dump(2) + "\n", result.message);
        return result;
    }
}
