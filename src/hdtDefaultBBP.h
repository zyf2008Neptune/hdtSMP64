#pragma once

namespace hdt
{
    class DefaultBBP
    {
    public:
        using RemapEntry_t = std::pair<int, std::string>;
        using NameSet_t = std::unordered_set<std::string>;
        using NameMap_t = std::unordered_map<std::string, NameSet_t>;
        using PhysicsFile_t = std::pair<std::string, NameMap_t>;

        struct Remap
        {
            std::string name;
            std::set<RemapEntry_t> entries;
            std::unordered_set<std::string> required;
        };

        static auto instance() -> DefaultBBP*;
        auto scanBBP(RE::NiNode* scan) -> PhysicsFile_t;

    private:
        DefaultBBP();

        std::unordered_map<std::string, std::string> bbpFileList;
        std::vector<Remap> remaps;

        auto loadDefaultBBPs() -> void;
        auto scanDefaultBBP(RE::NiNode* scan) -> PhysicsFile_t;
        auto getNameMap(RE::NiNode* armor) -> NameMap_t;
        static auto defaultNameMap(RE::NiNode* armor) -> NameMap_t;
    };
}
