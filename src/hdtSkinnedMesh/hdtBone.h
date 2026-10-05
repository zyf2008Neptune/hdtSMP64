#pragma once

#include <array>
#include "hdtBulletHelper.h"

namespace hdt
{
    _CRT_ALIGN(16)
    struct Bone
    {
        Bone() { _mm_store_ps(m_reserved.data(), _mm_setzero_ps()); }

        // cache from rigidbody
        btMatrix4x3T m_vertexToWorld;

        std::array<float, 3> m_reserved{}; // reserved for float4 aligned
        float m_maginMultipler{}; // scaled margin
    };
} // namespace hdt
