#include "hdtCollisionAlgorithm.h"

namespace hdt
{
    CheckTriangle::CheckTriangle(const btVector3& p0, const btVector3& p1, const btVector3& p2, const float margin,
                                 float prenetration) :
        p0(p0), p1(p1), p2(p2), margin(margin), prenetration(prenetration)
    {
        edge0 = p1 - p0;
        edge1 = p2 - p0;
        edge2 = p2 - p1;
        normal = edge0.cross(edge1);

        const __m128 len2 = _mm_dp_ps(normal.get128(), normal.get128(), 0x71);
        if (_mm_cvtss_f32(len2) < FLT_EPSILON * FLT_EPSILON)
        {
            valid = false;
        }
        else
        {
            valid = true;
            normal.set128(_mm_div_ps(normal.get128(), setAll0(_mm_sqrt_ss(len2))));

            d00 = edge0.dot(edge0);
            d01 = edge0.dot(edge1);
            d11 = edge1.dot(edge1);

            if (fabs(d00 * d11 - d01 * d01) < FLT_EPSILON)
            {
                invDenom = std::numeric_limits<float>::infinity();
            }

            invDenom = 1.0f / (d00 * d11 - d01 * d01);

            if (prenetration > -FLT_EPSILON && prenetration < FLT_EPSILON)
            {
                prenetration = 0;
            }

            if (prenetration < 0)
            {
                // triangle facing the other way
                normal = -normal;
                prenetration = -prenetration;
            }
            this->prenetration = prenetration;
        }
    }

    auto closestPointOnTriangle(const btVector3& p, const btVector3& a, const btVector3& b,
                                const btVector3& c) -> btVector3
    {
        const btVector3 ab = b - a;
        const btVector3 ac = c - a;
        const btVector3 ap = p - a;

        const float d1 = ab.dot(ap);
        const float d2 = ac.dot(ap);
        if (d1 <= 0.0f && d2 <= 0.0f)
        {
            return a; // 1,0,0
        }

        const btVector3 bp = p - b;
        const float d3 = ab.dot(bp);
        const float d4 = ac.dot(bp);
        if (d3 >= 0.0f && d4 <= d3)
        {
            return b; // 0,1,0
        }

        const float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
        {
            const float v = d1 / (d1 - d3);
            return a + ab * v; // u,v,0
        }

        const btVector3 cp = p - c;
        const float d5 = ab.dot(cp);
        const float d6 = ac.dot(cp);
        if (d6 >= 0.0f && d5 <= d6)
        {
            return c; // 0,0,1
        }

        const float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
        {
            const float w = d2 / (d2 - d6);
            return a + ac * w; // u,0,w
        }

        const float va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
        {
            const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b + (c - b) * w; // 0,v,w
        }

        const float denom = 1.0f / (va + vb + vc);
        const float v = vb * denom;
        const float w = vc * denom;
        return a + ab * v + ac * w;
    }

    static auto pointInTriangle(const btVector3& p1, const btVector3& p2, const btVector3& p3,
                                [[maybe_unused]] const btVector3& normal, const btVector3& p) -> bool
    {
        const auto ab = p2 - p1;
        const auto ac = p3 - p1;

        const auto ap = p - p1;
        const auto d1 = ab.dot(ap);
        const auto d2 = ac.dot(ap);

        if (d1 <= 0 && d2 <= 0)
        {
            return false; // 1,0,0
        }

        const auto bp = p - p2;
        const auto d3 = ab.dot(bp);
        const auto d4 = ac.dot(bp);
        if (d3 >= 0 && d4 <= d3)
        {
            return false; // 0,1,0
        }

        const auto cp = p - p3;
        const auto d5 = ab.dot(cp);
        const auto d6 = ac.dot(cp);
        if (d6 >= 0 && d5 <= d6)
        {
            return false; // 0,0,1
        }

        const auto vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0)
        {
            return false; // u,v,0
        }

        const auto vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0)
        {
            return false; // u,0,w
        }

        const auto va = d3 * d6 - d5 * d4;
        if (va <= 0 && d4 >= d3 && d5 >= d6)
        {
            return false; // 0,v,w
        }

        return true;
    }

    auto checkSphereSphere(const btVector3& a, const btVector3& b, const float ra, const float rb,
                           CollisionResult& res) -> bool
    {
        const auto diff = a - b;
        const auto dist2 = diff.length2();
        if (const auto radiusSum = ra + rb; dist2 > radiusSum * radiusSum)
        {
            return false;
        }

        const float len = btSqrt(dist2);

        res.normOnB = btVector3(1, 0, 0);
        if (len > FLT_EPSILON)
        {
            res.normOnB = diff / len;
        }

        res.posA = a - res.normOnB * ra;
        res.posB = b + res.normOnB * rb;

        return true;
    }

    auto checkSphereTriangle(const btVector3& s, const float r, const CheckTriangle& tri, CollisionResult& res) -> bool
    {
        if (!tri.valid)
        {
            return false;
        }

        const auto radiusWithMargin = r + tri.margin;

        const auto closest = closestPointOnTriangle(s, tri.p0, tri.p1, tri.p2);

        const auto delta = s - closest;
        const auto dist2 = delta.length2();

        // Early out if sphere is too far from closest point
        if (dist2 > radiusWithMargin * radiusWithMargin)
        {
            return false;
        }

        const auto p1ToCentre = s - tri.p0;
        auto distanceFromPlane = p1ToCentre.dot(tri.normal);
        auto normal = tri.normal;

        if (tri.prenetration >= FLT_EPSILON)
        {
            if (distanceFromPlane < -tri.prenetration || distanceFromPlane > radiusWithMargin)
            {
                return false;
            }
        }

        if (dist2 < FLT_EPSILON * FLT_EPSILON)
        {
            res.normOnB = normal;
            res.depth = -radiusWithMargin;
            res.posA = s - normal * r;
            res.posB = closest;
            return true;
        }

        float dist = btSqrt(dist2);

        // use face normal for face contacts, delta direction for edge/vertex contacts
        btVector3 contactNormal;
        if (distanceFromPlane > 0 && dist2 <= (distanceFromPlane * distanceFromPlane + FLT_EPSILON))
        {
            contactNormal = normal;
        }
        else
        {
            contactNormal = delta / dist;
            if (contactNormal.dot(normal) < 0)
            {
                if (tri.prenetration < FLT_EPSILON)
                {
                    contactNormal = -contactNormal;
                }
                else
                {
                    return false;
                }
            }
        }

        res.normOnB = contactNormal;
        res.depth = dist - radiusWithMargin;
        res.posA = s - contactNormal * r;
        res.posB = closest;

        return res.depth < -FLT_EPSILON;
    }
} // namespace hdt
