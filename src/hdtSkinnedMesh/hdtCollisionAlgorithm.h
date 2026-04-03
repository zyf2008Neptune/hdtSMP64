#pragma once

#include "hdtCollider.h"

namespace hdt
{
    struct CollisionResult
    {
        btVector3 posA;
        btVector3 posB;
        btVector3 normOnB;
        Collider* colliderA;
        Collider* colliderB;
        float depth;
    };

    struct CheckTriangle
    {
        CheckTriangle(const btVector3& p0, const btVector3& p1, const btVector3& p2, float margin, float prenetration);

        btVector3 p0;
        btVector3 p1;
        btVector3 p2;
        btVector3 normal;
        btVector3 edge0;
        btVector3 edge1;
        btVector3 edge2;
        float margin;
        float prenetration;
        float d00;
        float d01;
        float d11;
        float invDenom;
        bool valid;
    };

    auto closestPointOnTriangle(const btVector3& p, const btVector3& a, const btVector3& b,
                                const btVector3& c) -> btVector3;
    auto checkSphereSphere(const btVector3& a, const btVector3& b, float ra, float rb, CollisionResult& res) -> bool;
    auto checkSphereTriangle(const btVector3& s, float r, const CheckTriangle& tri, CollisionResult& res) -> bool;

#ifndef CUDA
    static auto BaryCoord(const btVector3& a,
                          const btVector3& b,
                          const btVector3& c,
                          const btVector3& p) -> btVector3
    {
        auto xmm3 = a - p;
        auto xmm4 = b - p;
        auto xmm5 = c - p;
        auto xmm0 = btCross(xmm3, xmm4).get128();
        auto xmm1 = btCross(xmm4, xmm5).get128();
        auto xmm2 = btCross(xmm5, xmm3).get128();
        xmm0 = _mm_dp_ps(xmm0, xmm0, 0x74);
        xmm1 = _mm_dp_ps(xmm1, xmm1, 0x71);
        xmm2 = _mm_dp_ps(xmm2, xmm2, 0x72);
        xmm0 = _mm_or_ps(xmm0, xmm1);
        xmm0 = _mm_or_ps(xmm0, xmm2);
        xmm0 = _mm_sqrt_ps(xmm0);
        xmm1 = _mm_set_ps1(1);
        xmm1 = _mm_dp_ps(xmm1, xmm0, 0x77);
        return _mm_div_ps(xmm0, xmm1);
    }
#endif
}
