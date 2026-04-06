#pragma once

#include "NetImmerseUtils.h"
#include "hdtSkinnedMesh/hdtBulletHelper.h"

namespace hdt
{
    auto convertNi(const RE::NiMatrix3& rhs) -> btQuaternion;

    inline auto convertNi(const RE::NiPoint3& rhs) -> btVector3 { return btVector3(rhs.x, rhs.y, rhs.z); }

    inline auto convertNi(const RE::NiTransform& rhs) -> btQsTransform
    {
        btQsTransform ret;
        ret.setBasis(convertNi(rhs.rotate));
        ret.setOrigin(convertNi(rhs.translate));
        ret.setScale(rhs.scale);
        return ret;
    }

    auto convertBt(const btVector3& rhs) -> RE::NiPoint3;
    auto convertBt(const btMatrix3x3& rhs) -> RE::NiMatrix3;
    auto convertBt(const btQuaternion& rhs) -> RE::NiMatrix3;
    auto convertBt(const btQsTransform& rhs) -> RE::NiTransform;

    static const float scaleRealWorld = 0.01425f;
    static const float scaleSkyrim = 1 / scaleRealWorld;
} // namespace hdt
