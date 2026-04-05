#include "DynamicHDT.h"

#include <boost/beast/core/string.hpp>

#include "hdtSkyrimSystem.h"
#include "hdtSkinnedMesh/hdtSkinnedMeshSystem.h"

auto hdt::util::splitArmorAddonFormID(const std::string& nodeName) -> uint32_t
{
    try
    {
        return std::stoul(nodeName.substr(1, 8), nullptr, 16);
    }
    catch (...)
    {
        return 0;
    }
}

auto hdt::util::UInt32toString(uint32_t formID) -> std::string { return fmt::format("{:08X}", formID); }

auto _deprefix(std::string_view str_with_prefix) -> std::string
{
    std::string str_no_prefix{str_with_prefix};
    static constexpr auto autoRenameSubstr = "hdtSSEPhysics_AutoRename_"sv;

    // follows case-insensitivity semantics of BSFixedString
    static constexpr auto str_size = autoRenameSubstr.size();
    // if (str_with_prefix.size() >= autoRenameSubstr.size() &&
    //     boost::beast::iequals(str_with_prefix.substr(0, str_size), autoRenameSubstr))
    if (str_with_prefix.size() >= str_size && _memicmp(str_with_prefix.data(), autoRenameSubstr.data(), str_size) == 0)
    {
        str_no_prefix = str_with_prefix.substr(str_with_prefix.find(' ') + 1);
    }

    return str_no_prefix;
}

auto _match_name(const RE::BSFixedString& a, const RE::BSFixedString& b) -> bool
{
    if (a.empty() || b.empty())
    {
        return false;
    }
    //return boost::beast::iequals(_deprefix(a), _deprefix(b));
    return _stricmp(_deprefix(a).c_str(), _deprefix(b).c_str()) == 0;
}

auto hdt::util::transferCurrentPosesBetweenSystems(SkyrimSystem* src, SkyrimSystem* dst) -> void
{
    for (const auto& b1 : src->getBones())
    {
        if (!b1)
        {
            continue;
        }
        for (const auto& b2 : dst->getBones())
        {
            if (!b2)
            {
                continue;
            }

            if (_match_name(b1->m_name, b2->m_name))
            {
                btTransform nodeWorldTrans = b1->m_rig.getWorldTransform() * b1->m_rigToLocal;
                btTransform newRigTrans = nodeWorldTrans * b2->m_localToRig;

                b2->m_rig.setWorldTransform(newRigTrans);
                b2->m_rig.setInterpolationWorldTransform(newRigTrans);

                b2->m_currentTransform = b1->m_currentTransform;
                b2->m_origToSkeletonTransform = b1->m_origToSkeletonTransform;
                b2->m_origTransform = b1->m_origTransform;
                btVector3 oldCoM = b1->m_rig.getWorldTransform().getOrigin();
                btVector3 newCoM = newRigTrans.getOrigin();
                btVector3 comShift = newCoM - oldCoM;

                btVector3 angVel = b1->m_rig.getAngularVelocity();

                btVector3 linVel = b1->m_rig.getLinearVelocity() + angVel.cross(comShift);

                constexpr float transitionDamping = 0.8f;
                b2->m_rig.setLinearVelocity(linVel * transitionDamping);
                b2->m_rig.setAngularVelocity(angVel * transitionDamping);
                b2->m_rig.setInterpolationLinearVelocity(linVel * transitionDamping);
                b2->m_rig.setInterpolationAngularVelocity(angVel * transitionDamping);

                break;
            }
        }
    }
}
