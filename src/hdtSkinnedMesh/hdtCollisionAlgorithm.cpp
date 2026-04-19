#include "hdtCollisionAlgorithm.h"

namespace hdt
{

    auto checkSphereSphere(const btVector3& a, const btVector3& b, const float ra, const float rb, CollisionResult& res)
        -> bool
    {
        const btVector3 diff = a - b;
        const float dist2 = diff.length2();
        const float radiusSum = ra + rb;

        if (dist2 > radiusSum * radiusSum)
        {
            return false;
        }

        const float len = btSqrt(dist2);

        res.normOnB = btVector3(1, 0, 0);
        if (len > FLT_EPSILON)
        {
            res.normOnB = diff / len;
        }

        res.depth = len - radiusSum;
        res.posA = a - res.normOnB * ra;
        res.posB = b + res.normOnB * rb;

        return true;
    }
} // namespace hdt
