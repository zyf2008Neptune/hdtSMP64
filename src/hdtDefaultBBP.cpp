#include "hdtDefaultBBP.h"

#include <ranges>

#include "XmlReader.h"
#include "NetImmerseUtils.h"

namespace hdt
{
    auto DefaultBBP::instance() -> DefaultBBP*
    {
        static DefaultBBP s;
        return std::addressof(s);
    }

    auto DefaultBBP::scanBBP(RE::NiNode* scan) -> PhysicsFile_t
    {
        for (auto i = 0; i < scan->extraDataSize; ++i)
        {
            const auto stringData = netimmerse_cast<RE::NiStringExtraData*>(scan->extra[i]);
            if (stringData && stringData->name == "HDT Skinned Mesh Physics Object" && stringData->value)
            {
                return {{std::string(stringData->value)}, defaultNameMap(scan)};
            }
        }

        return scanDefaultBBP(scan);
    }

    DefaultBBP::DefaultBBP()
    {
        loadDefaultBBPs();
    }

    auto DefaultBBP::loadDefaultBBPs() -> void
    {
        const auto path = "SKSE/Plugins/hdtSkinnedMeshConfigs/defaultBBPs.xml";

        auto loaded = readAllFile(path);
        if (loaded.empty())
        {
            return;
        }

        const auto saved_locale = std::locale();
        std::locale::global(std::locale("en_US"));

        try
        {
            XMLReader reader(reinterpret_cast<uint8_t*>(loaded.data()), loaded.size());

            reader.nextStartElement();
            if (reader.GetName() != "default-bbps")
            {
                std::locale::global(saved_locale);
                return;
            }

            while (reader.Inspect())
            {
                if (reader.GetInspected() == Xml::Inspected::StartTag)
                {
                    if (reader.GetName() == "map")
                    {
                        try
                        {
                            auto shape = reader.getAttribute("shape");
                            auto file = reader.getAttribute("file");
                            bbpFileList.insert(std::make_pair(shape, file));
                        }
                        catch (...)
                        {
                            logger::warn("defaultBBP({},{}) : invalid map", reader.GetRow(), reader.GetColumn());
                        }
                        reader.skipCurrentElement();
                    }
                    else if (reader.GetName() == "remap")
                    {
                        const auto target = reader.getAttribute("target");
                        Remap remap = {.name = target, .entries = {}, .required = {}};
                        while (reader.Inspect())
                        {
                            if (reader.GetInspected() == Xml::Inspected::StartTag)
                            {
                                if (reader.GetName() == "source")
                                {
                                    auto priority = 0;
                                    try
                                    {
                                        priority = reader.getAttributeAsInt("priority");
                                    }
                                    catch (...)
                                    {
                                        throw;
                                    }
                                    auto source = reader.readText();
                                    remap.entries.insert({priority, source});
                                }
                                else if (reader.GetName() == "requires")
                                {
                                    auto req = reader.readText();
                                    remap.required.insert(req);
                                }
                                else
                                {
                                    logger::warn("defaultBBP({},{}) : unknown element", reader.GetRow(),
                                                 reader.GetColumn());
                                    reader.skipCurrentElement();
                                }
                            }
                            else if (reader.GetInspected() == Xml::Inspected::EndTag)
                            {
                                break;
                            }
                        }
                        remaps.emplace_back(remap);
                    }
                    else
                    {
                        logger::warn("defaultBBP({},{}) : unknown element", reader.GetRow(), reader.GetColumn());
                        reader.skipCurrentElement();
                    }
                }
                else if (reader.GetInspected() == Xml::Inspected::EndTag)
                {
                    break;
                }
            }
        }
        catch (...)
        {
            std::locale::global(saved_locale);
            throw;
        }

        std::locale::global(saved_locale);
    }

    auto DefaultBBP::scanDefaultBBP(RE::NiNode* armor) -> PhysicsFile_t
    {
        static std::mutex s_lock;
        std::scoped_lock l(s_lock);

        if (bbpFileList.empty())
        {
            return {{std::string("")}, {}};
        }

        auto remappedNames = instance()->getNameMap(armor);

        const auto it = std::ranges::find_if(bbpFileList,
                                             [&](const std::pair<std::string, std::string>& e)
                                             {
                                                 return remappedNames.contains(e.first);
                                             });
        return {it == bbpFileList.end() ? "" : it->second, remappedNames};
    }

    auto DefaultBBP::getNameMap(RE::NiNode* armor) -> NameMap_t
    {
        auto nameMap = defaultNameMap(armor);

        for (auto remap : remaps)
        {
            auto doRemap = true;
            for (const auto& req : remap.required)
            {
                if (!nameMap.contains(req))
                {
                    doRemap = false;
                }
            }

            if (doRemap)
            {
                auto start = std::ranges::find_if(std::views::reverse(remap.entries), [&](const auto& e)
                {
                    return nameMap.contains(e.second);
                });
                const auto end = std::find_if(start, remap.entries.rend(), [&](const auto& e)
                {
                    return e.first != start->first;
                });
                if (start != remap.entries.rend())
                {
                    auto&& s = nameMap.insert({remap.name, {}}).first;
                    std::for_each(start, end, [&](const auto& e)
                    {
                        auto it = nameMap.find(e.second);
                        if (it != nameMap.end())
                        {
                            std::for_each(it->second.begin(), it->second.end(), [&](const std::string& name)
                            {
                                s->second.insert(name);
                            });
                        }
                    });
                }
            }
        }
        return nameMap;
    }

    auto DefaultBBP::defaultNameMap(RE::NiNode* armor) -> NameMap_t
    {
        std::unordered_map<std::string, std::unordered_set<std::string>> nameMap;

        // This case never happens to a lurker skeleton, thus we don't need to test.
        if (const auto skinned = findNode(armor, "BSFaceGenNiNodeSkinned"))
        {
            const auto& skinnedNodechildren = skinned->GetChildren();
            for (const auto& i : skinnedNodechildren)
            {
                if (!i)
                {
                    continue;
                }

                const auto tri = i->AsTriShape();
                if (!tri || tri->name.empty())
                {
                    continue;
                }


                nameMap.emplace(tri->name.c_str(), std::unordered_set{std::string(tri->name.c_str())});
            }
        }

        const auto& armorNodechildren = armor->GetChildren();
        for (const auto& i : armorNodechildren)
        {
            if (!i)
            {
                continue;
            }

            const auto tri = i->AsTriShape();
            if (!tri || tri->name.empty())
            {
                continue;
            }

            nameMap.emplace(tri->name.c_str(), std::unordered_set{std::string(tri->name.c_str())});
        }

        return nameMap;
    }
}
