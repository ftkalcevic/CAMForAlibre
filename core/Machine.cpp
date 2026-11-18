// Machine.cpp : a machine, with its axes and its tools.

#include "Machine.h"

#include <algorithm>
#include <cctype>

#include <nlohmann/json.hpp>

#include "FileIO.h"
#include "ToolJson.h"

namespace camcore
{
    namespace
    {
        namespace fs = std::filesystem;
        using nlohmann::json;

        // The add-on was once called AlibreCAM; the files keep this name so they still load.

        const char* const kFormatName = "AlibreCAM machine";

        MachineLoadResult LoadFailure(LoadStatus status, const std::string& message)
        {
            MachineLoadResult result;
            result.status = status;
            result.message = message;
            return result;
        }

        bool RotaryAxisFromName(const std::string& name, RotaryAxis& axis)
        {
            for (RotaryAxis candidate : { RotaryAxis::MinusX, RotaryAxis::PlusX, RotaryAxis::MinusY, RotaryAxis::PlusY })
            {
                if (name == RotaryAxisName(candidate))
                {
                    axis = candidate;
                    return true;
                }
            }
            return false;
        }

        std::string LowerCase(std::string text)
        {
            for (char& c : text)
                c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
            return text;
        }
    }

    std::string RotaryAxisName(RotaryAxis axis)
    {
        switch (axis)
        {
        case RotaryAxis::MinusX: return "-X";
        case RotaryAxis::PlusX:  return "+X";
        case RotaryAxis::MinusY: return "-Y";
        default:                 return "+Y";
        }
    }

    std::vector<std::string> ValidateMotion(const MotionLimits& motion)
    {
        std::vector<std::string> problems;
        const struct
        {
            const char* name;
            const AxisLimit& limit;
        } axes[] = { { "X", motion.x }, { "Y", motion.y }, { "Z", motion.z } };
        for (const auto& axis : axes)
        {
            if (!(axis.limit.velocity > 0.0))
                problems.push_back(std::string("The ") + axis.name + " axis's maximum velocity must be more than 0.");
            if (!(axis.limit.acceleration > 0.0))
                problems.push_back(std::string("The ") + axis.name +
                                   " axis's maximum acceleration must be more than 0.");
        }
        if (!(motion.rotaryVelocity > 0.0))
            problems.push_back("The rotary axis's maximum velocity must be more than 0.");
        if (!(motion.rotaryAcceleration > 0.0))
            problems.push_back("The rotary axis's maximum acceleration must be more than 0.");
        if (!(motion.blendTolerance >= 0.0))
            problems.push_back("The blend tolerance can't be less than 0.");
        return problems;
    }

    bool operator==(const MotionLimits& a, const MotionLimits& b)
    {
        return a.x.velocity == b.x.velocity && a.x.acceleration == b.x.acceleration &&
               a.y.velocity == b.y.velocity && a.y.acceleration == b.y.acceleration &&
               a.z.velocity == b.z.velocity && a.z.acceleration == b.z.acceleration &&
               a.rotaryVelocity == b.rotaryVelocity && a.rotaryAcceleration == b.rotaryAcceleration &&
               a.blendTolerance == b.blendTolerance;
    }

    std::vector<std::string> ValidateMachine(const Machine& machine)
    {
        std::vector<std::string> problems;
        if (machine.id.empty())
            problems.push_back("The machine has no ID.");
        if (machine.name.empty())
            problems.push_back("The machine needs a name.");
        if (machine.axes != 3 && machine.axes != 4)
            problems.push_back("The machine must have 3 or 4 axes.");
        if (!(machine.travelX > 0.0) || !(machine.travelY > 0.0) || !(machine.travelZ > 0.0))
            problems.push_back("Each axis's travel must be more than 0.");
        for (const std::string& problem : ValidateMotion(machine.motion))
            problems.push_back(problem);
        if (!(machine.maxSpindleRpm > 0.0))
            problems.push_back("The spindle's maximum speed must be more than 0.");
        if (!(machine.spindlePower >= 0.0))
            problems.push_back("The spindle's power can't be negative.");
        if (!(machine.holderDiameter > 0.0) || !(machine.holderDepth > 0.0))
            problems.push_back("The tool holder's diameter and depth must be more than 0.");
        for (const std::string& problem : ValidateArcFitting(machine.arcs))
            problems.push_back(problem);
        for (const std::string& problem : ValidateLibrary(machine.tools))
            problems.push_back(problem);
        return problems;
    }

    MachineLoadResult LoadMachine(const fs::path& path)
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
            return LoadFailure(LoadStatus::NotACamFile, "Not a machine: " + path.u8string());

        const auto format = root.find("format");
        if (format == root.end() || !format->is_string() || format->get<std::string>() != kFormatName)
            return LoadFailure(LoadStatus::NotACamFile, "Not a machine: " + path.u8string());

        const auto version = root.find("formatVersion");
        if (version == root.end() || !version->is_number_integer() || version->get<int>() < 1)
            return LoadFailure(LoadStatus::NotACamFile, "The format version is missing or invalid: " + path.u8string());
        if (version->get<int>() > kMachineFormatVersion)
        {
            return LoadFailure(LoadStatus::NewerVersion,
                               "This machine uses format version " + std::to_string(version->get<int>()) +
                                   ", but this add-on supports up to version " +
                                   std::to_string(kMachineFormatVersion) + ".");
        }

        MachineLoadResult result;
        try
        {
            result.machine.id = root.at("id").get<std::string>();
            result.machine.name = root.at("name").get<std::string>();
            result.machine.axes = root.at("axes").get<int>();
            if (!RotaryAxisFromName(root.at("rotaryAxis").get<std::string>(), result.machine.rotary))
                return LoadFailure(LoadStatus::NotACamFile, "The rotary axis isn't one this add-on knows: " +
                                                                path.u8string());
            // Version 1 files have no travel or spindle speed, so they keep the defaults.
            const auto travel = root.find("travel");
            if (travel != root.end())
            {
                result.machine.travelX = travel->at(0).get<double>();
                result.machine.travelY = travel->at(1).get<double>();
                result.machine.travelZ = travel->at(2).get<double>();
            }
            const auto spindle = root.find("maxSpindleRpm");
            if (spindle != root.end())
                result.machine.maxSpindleRpm = spindle->get<double>();
            const auto power = root.find("spindlePower");
            if (power != root.end())
                result.machine.spindlePower = power->get<double>();
            // Before the motion limits, one rapid rate for every axis.
            const auto rapid = root.find("rapidRate");
            if (rapid != root.end())
            {
                const double rate = rapid->get<double>();
                MotionLimits& motion = result.machine.motion;
                motion.x.velocity = motion.y.velocity = motion.z.velocity = rate / 60.0;
            }
            const auto motion = root.find("motion");
            if (motion != root.end())
            {
                MotionLimits& limits = result.machine.motion;
                const auto readAxis = [&](const char* name, AxisLimit& axis) {
                    const auto found = motion->find(name);
                    if (found == motion->end())
                        return;
                    // In mm/s; "velocity" was mm/min, before Frank changed it.
                    if (found->contains("maxVelocity"))
                        axis.velocity = found->at("maxVelocity").get<double>();
                    else if (found->contains("velocity"))
                        axis.velocity = found->at("velocity").get<double>() / 60.0;
                    axis.acceleration = found->value("acceleration", axis.acceleration);
                };
                readAxis("x", limits.x);
                readAxis("y", limits.y);
                readAxis("z", limits.z);
                const auto rotary = motion->find("rotary");
                if (rotary != motion->end())
                {
                    limits.rotaryVelocity = rotary->value("velocity", limits.rotaryVelocity);
                    limits.rotaryAcceleration = rotary->value("acceleration", limits.rotaryAcceleration);
                }
                limits.blendTolerance = motion->value("blendTolerance", limits.blendTolerance);
            }
            const auto holder = root.find("toolHolder");
            if (holder != root.end())
            {
                result.machine.holderDiameter = holder->at("diameter").get<double>();
                result.machine.holderDepth = holder->at("depth").get<double>();
            }
            const auto arcs = root.find("arcs");
            if (arcs != root.end())
            {
                ArcFitting& fitting = result.machine.arcs;
                fitting.enabled = arcs->value("enabled", fitting.enabled);
                fitting.xy = arcs->value("xy", fitting.xy);
                fitting.zx = arcs->value("zx", arcs->value("xz", fitting.zx));    // "xz" before it was renamed
                fitting.yz = arcs->value("yz", fitting.yz);
                fitting.tolerance = arcs->value("tolerance", fitting.tolerance);
            }

            for (const json& item : root.at("tools"))
                result.machine.tools.tools.push_back(ToolFromJson(item));
        }
        catch (const std::exception& problem)
        {
            return LoadFailure(LoadStatus::NotACamFile, "The machine is damaged: " + std::string(problem.what()));
        }

        const std::vector<std::string> problems = ValidateMachine(result.machine);
        if (!problems.empty())
            return LoadFailure(LoadStatus::NotACamFile, problems.front());

        result.status = LoadStatus::Ok;
        return result;
    }

    SaveResult SaveMachine(const Machine& machine, const fs::path& path)
    {
        SaveResult result;
        const std::vector<std::string> problems = ValidateMachine(machine);
        if (!problems.empty())
        {
            result.message = problems.front();
            return result;
        }

        std::string text;
        try
        {
            json tools = json::array();
            for (const Tool& tool : machine.tools.tools)
                tools.push_back(ToolToJson(tool));
            json root =
            {
                { "format", kFormatName },
                { "formatVersion", kMachineFormatVersion },
                { "id", machine.id },
                { "name", machine.name },
                { "axes", machine.axes },
                { "rotaryAxis", RotaryAxisName(machine.rotary) },
                { "travel", json::array({ machine.travelX, machine.travelY, machine.travelZ }) },
                { "maxSpindleRpm", machine.maxSpindleRpm },
                { "motion",
                  { { "x", { { "maxVelocity", machine.motion.x.velocity }, { "acceleration", machine.motion.x.acceleration } } },
                    { "y", { { "maxVelocity", machine.motion.y.velocity }, { "acceleration", machine.motion.y.acceleration } } },
                    { "z", { { "maxVelocity", machine.motion.z.velocity }, { "acceleration", machine.motion.z.acceleration } } },
                    { "rotary",
                      { { "velocity", machine.motion.rotaryVelocity },
                        { "acceleration", machine.motion.rotaryAcceleration } } },
                    { "blendTolerance", machine.motion.blendTolerance } } },
                { "toolHolder", { { "diameter", machine.holderDiameter }, { "depth", machine.holderDepth } } },
                { "arcs",
                  { { "enabled", machine.arcs.enabled },
                    { "xy", machine.arcs.xy },
                    { "zx", machine.arcs.zx },
                    { "yz", machine.arcs.yz },
                    { "tolerance", machine.arcs.tolerance } } },
                { "tools", tools }
            };
            if (machine.spindlePower > 0.0)
                root["spindlePower"] = machine.spindlePower;
            text = root.dump(2) + "\n";
        }
        catch (const std::exception& problem)
        {
            result.message = std::string("Could not prepare the file: ") + problem.what();
            return result;
        }

        result.ok = WriteFileAtomically(path, text, result.message);
        return result;
    }

    fs::path MachineFilePath(const fs::path& folder, const std::string& id)
    {
        return folder / fs::u8path(id + ".json");
    }

    std::vector<MachineEntry> FindMachines(const fs::path& folder)
    {
        std::vector<MachineEntry> entries;
        std::error_code error;
        if (!fs::is_directory(folder, error))
            return entries;

        for (const fs::directory_entry& file : fs::directory_iterator(folder, error))
        {
            if (!file.is_regular_file(error) || LowerCase(file.path().extension().u8string()) != ".json")
                continue;
            MachineEntry entry;
            entry.path = file.path();
            const MachineLoadResult loaded = LoadMachine(file.path());
            entry.status = loaded.status;
            entry.machine = loaded.machine;
            entry.message = loaded.message;
            entries.push_back(entry);
        }

        std::sort(entries.begin(), entries.end(), [](const MachineEntry& a, const MachineEntry& b) {
            const bool aOk = a.status == LoadStatus::Ok;
            const bool bOk = b.status == LoadStatus::Ok;
            if (aOk != bOk)
                return aOk;
            if (aOk)
                return LowerCase(a.machine.name) < LowerCase(b.machine.name);
            return a.path < b.path;
        });
        return entries;
    }

    int CountSharedTools(const ToolLibrary& into, const ToolLibrary& from)
    {
        int shared = 0;
        for (const Tool& tool : from.tools)
        {
            if (FindToolById(into, tool.id) != nullptr)
                ++shared;
        }
        return shared;
    }

    ToolImportResult ImportTools(ToolLibrary& into, const ToolLibrary& from, bool replaceShared)
    {
        ToolImportResult result;
        for (Tool tool : from.tools)
        {
            const auto existing = std::find_if(into.tools.begin(), into.tools.end(),
                                               [&](const Tool& other) { return other.id == tool.id; });
            if (existing != into.tools.end())
            {
                if (!replaceShared)
                {
                    ++result.skipped;
                    continue;
                }
                into.tools.erase(existing);
                ++result.replaced;
            }
            else
            {
                ++result.added;
            }

            const bool numberTaken = std::any_of(into.tools.begin(), into.tools.end(),
                                                 [&](const Tool& other) { return other.number == tool.number; });
            if (numberTaken)
            {
                tool.number = NextFreeToolNumber(into);
                ++result.renumbered;
            }
            into.tools.push_back(tool);
        }
        return result;
    }
}
