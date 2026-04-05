#include "hdtCollider.h"

#include <algorithm>

namespace hdt
{
    auto ColliderTree::insertCollider(const U32* keys, size_t keyCount, const Collider& c) -> void
    {
        auto p = this;
        for (size_t i = 0; i < keyCount && i < 4; ++i)
        {
            auto f = std::ranges::find_if(p->children, [=](const ColliderTree& n) { return n.key == keys[i]; });
            if (f == p->children.end())
            {
                p->children.emplace_back(keys[i]);
                p = &p->children.back();
            }
            else
            {
                p = &*f;
            }
        }
        p->colliders.push_back(c);
    }

    // finds overlapping pairs between two collider trees
    auto ColliderTree::checkCollisionL(ColliderTree* r, std::vector<std::pair<ColliderTree*, ColliderTree*>>& ret)
        -> void
    {
        enum struct Mode : uint8_t
        {
            L, // still splitting a, b is along for the ride
            R // a is a leaf, now drilling into b's subtree
        };
        struct Entry
        {
            ColliderTree* a;
            ColliderTree* b;
            Mode mode;
        };
        // thread_local so we don't re-alloc every call. This gets hammered
        thread_local std::vector<Entry> stack;
        stack.clear();
        stack.emplace_back(this, r, Mode::L);
        while (!stack.empty())
        {
            auto [a, b, mode] = stack.back();
            stack.pop_back();

            if (a->isKinematic && b->isKinematic)
            {
                continue;
            }

            if (mode == Mode::L)
            {
                if (!a->aabbAll.collideWith(b->aabbAll))
                {
                    continue;
                }

                // a is a leaf — switch to R-mode and walk down b
                if (a->numCollider && a->aabbMe.collideWith(b->aabbAll))
                {
                    if (a->aabbMe.collideWith(b->aabbMe))
                    {
                        ret.emplace_back(a, b);
                    }

                    const auto begin = b->children.data();
                    // skip b's kinematic children if a is kinematic (would get culled above anyway)
                    const auto end = begin + (a->isKinematic ? b->dynChild : b->children.size());
                    for (auto i = begin; i < end; ++i)
                    {
                        stack.emplace_back(a, i, Mode::R);
                    }
                }

                // keep splitting a — same kinematic shortcut
                const auto begin = a->children.data();
                const auto end = begin + (b->isKinematic ? a->dynChild : a->children.size());
                for (auto i = begin; i < end; ++i)
                {
                    stack.emplace_back(i, b, Mode::L);
                }
            }
            else
            {
                // a is always a leaf here (L only pushes R when numCollider is set)
                // numCollider check is technically redundant but whatever, it's cheap
                if (!a->numCollider)
                {
                    continue;
                }
                if (!a->aabbMe.collideWith(b->aabbAll))
                {
                    continue;
                }
                if (a->aabbMe.collideWith(b->aabbMe))
                {
                    ret.emplace_back(a, b);
                }

                const auto begin = b->children.data();
                const auto end = begin + (a->isKinematic ? b->dynChild : b->children.size());
                for (auto i = begin; i < end; ++i)
                {
                    stack.emplace_back(a, i, Mode::R);
                }
            }
        }
    }

    // dead code. checkCollisionL inlines R logic now. kept for API
    auto ColliderTree::checkCollisionR(ColliderTree* r, std::vector<std::pair<ColliderTree*, ColliderTree*>>& ret)
        -> void
    {
        if (isKinematic && r->isKinematic)
        {
            return;
        }

        if (numCollider)
        {
            if (!aabbMe.collideWith(r->aabbAll))
            {
                return;
            }

            if (aabbMe.collideWith(r->aabbMe))
            {
                ret.emplace_back(this, r);
            }

            const auto begin = r->children.data();
            const auto end = begin + (isKinematic ? r->dynChild : r->children.size());
            for (auto i = begin; i < end; ++i)
            {
                checkCollisionR(i, ret);
            }
        }
    }

    auto ColliderTree::clipCollider(const std::function<bool(const Collider&)>& func) -> void
    {
        for (auto& i : children)
        {
            i.clipCollider(func);
        }

        std::erase_if(colliders, func);
        std::erase_if(children, [](const ColliderTree& n) -> bool { return n.empty(); });
    }

    auto ColliderTree::updateKinematic(const std::function<float(const Collider*)>& func) -> void
    {
        U32 k = true;
        for (auto& i : colliders)
        {
            i.flexible = func(&i);
            k &= i.flexible < FLT_EPSILON;
        }

        for (auto& i : children)
        {
            i.updateKinematic(func);
            k &= i.isKinematic;
        }

        std::ranges::sort(colliders, [](const Collider& a, const Collider& b) { return a.flexible > b.flexible; });
        std::ranges::sort(children,
                          [](const ColliderTree& a, const ColliderTree& b) { return a.isKinematic < b.isKinematic; });

        isKinematic = k;

        if (k)
        {
            dynChild = dynCollider = 0;
        }
        else
        {
            for (dynChild = 0; dynChild < children.size(); ++dynChild)
            {
                if (children[dynChild].isKinematic)
                {
                    break;
                }
            }

            for (dynCollider = 0; dynCollider < colliders.size(); ++dynCollider)
            {
                if (colliders[dynCollider].flexible < FLT_EPSILON)
                {
                    break;
                }
            }
        }
    }

    auto ColliderTree::updateAabb() -> void
    {
        struct Frame
        {
            ColliderTree* node;
            size_t childIdx;
        };
        thread_local std::vector<Frame> stack;
        stack.clear();
        stack.emplace_back(this, 0);

        while (!stack.empty())
        {
            auto& f = stack.back();
            if (f.childIdx < f.node->children.size())
            {
                auto* child = &f.node->children[f.childIdx++];
                stack.emplace_back(child, 0);
                continue;
            }

            auto* node = f.node;
            stack.pop_back();

            // old code merged into aabb[0] directly which nuked collider 0's actual bbox,
            // making it always pass narrow-phase checks. accumulate in registers instead
            if (node->numCollider)
            {
                __m128 mn = node->aabb[0].m_min;
                __m128 mx = node->aabb[0].m_max;
                const auto* aabbPtr = node->aabb + 1;
                const auto* aabbEnd = node->aabb + node->numCollider;
                for (; aabbPtr < aabbEnd; ++aabbPtr)
                {
                    mn = _mm_min_ps(mn, aabbPtr->m_min);
                    mx = _mm_max_ps(mx, aabbPtr->m_max);
                }
                node->aabbMe.m_min = mn;
                node->aabbMe.m_max = mx;
            }

            __m128 allMin = node->aabbMe.m_min;
            __m128 allMax = node->aabbMe.m_max;
            for (const auto& child : node->children)
            {
                allMin = _mm_min_ps(allMin, child.aabbAll.m_min);
                allMax = _mm_max_ps(allMax, child.aabbAll.m_max);
            }
            node->aabbAll.m_min = allMin;
            node->aabbAll.m_max = allMax;
        }
    }

    auto ColliderTree::visitColliders(const std::function<void(Collider*)>& func) -> void
    {
        for (auto& i : colliders)
        {
            func(&i);
        }

        for (auto& i : children)
        {
            i.visitColliders(func);
        }
    }

    auto ColliderTree::optimize() -> void
    {
        for (auto& i : children)
        {
            i.optimize();
        }

        std::erase_if(children, [](const ColliderTree& n) { return n.empty(); });

        while (children.size() == 1 && children[0].colliders.empty())
        {
            vectorA16<ColliderTree> temp;
            temp.swap(children.front().children);
            children.swap(temp);
        }

        if (children.size() == 1 && colliders.empty())
        {
            colliders = children[0].colliders;

            vectorA16<ColliderTree> temp;
            temp.swap(children[0].children);
            children.swap(temp);
        }
    }

    // same deal as checkCollisionL - iterative, early return still works since we just bail out of the loop
    auto ColliderTree::collapseCollideL(ColliderTree* r) -> bool
    {
        enum struct Mode : uint8_t
        {
            L,
            R
        };
        struct Entry
        {
            ColliderTree* a;
            ColliderTree* b;
            Mode mode;
        };
        thread_local std::vector<Entry> stack;
        stack.clear();
        stack.emplace_back(this, r, Mode::L);

        while (!stack.empty())
        {
            const auto [a, b, mode] = stack.back();
            stack.pop_back();

            if (a->isKinematic && b->isKinematic)
            {
                continue;
            }

            if (mode == Mode::L)
            {
                if (!a->aabbAll.collideWith(b->aabbAll))
                {
                    continue;
                }

                if (a->numCollider && a->aabbMe.collideWith(b->aabbAll))
                {
                    if (a->aabbMe.collideWith(b->aabbMe))
                    {
                        return true;
                    }

                    const auto begin = b->children.data();
                    const auto end = begin + (a->isKinematic ? b->dynChild : b->children.size());
                    for (auto i = begin; i < end; ++i)
                    {
                        stack.emplace_back(a, i, Mode::R);
                    }
                }

                const auto begin = a->children.data();
                const auto end = begin + (b->isKinematic ? a->dynChild : a->children.size());
                for (auto i = begin; i < end; ++i)
                {
                    stack.emplace_back(i, b, Mode::L);
                }
            }
            else
            {
                if (!a->numCollider)
                {
                    continue;
                }
                if (!a->aabbMe.collideWith(b->aabbAll))
                {
                    continue;
                }

                if (a->aabbMe.collideWith(b->aabbMe))
                {
                    return true;
                }

                const auto begin = b->children.data();
                const auto end = begin + (a->isKinematic ? b->dynChild : b->children.size());
                for (auto i = begin; i < end; ++i)
                {
                    stack.emplace_back(a, i, Mode::R);
                }
            }
        }
        return false;
    }

    // dead code.. collapseCollideL inlines R logic now. kept for API
    auto ColliderTree::collapseCollideR(ColliderTree* r) -> bool
    {
        if (isKinematic && r->isKinematic)
        {
            return false;
        }

        if (numCollider)
        {
            if (!aabbMe.collideWith(r->aabbAll))
            {
                return false;
            }

            if (aabbMe.collideWith(r->aabbMe))
            {
                return true;
            }

            const auto begin = r->children.data();
            const auto end = begin + (isKinematic ? r->dynChild : r->children.size());
            for (auto i = begin; i < end; ++i)
            {
                if (collapseCollideR(i))
                {
                    return true;
                }
            }
        }

        return false;
    }

    auto ColliderTree::exportColliders(vectorA16<Collider>& exportTo) -> void
    {
        numCollider = static_cast<U32>(colliders.size());
        cbuf = reinterpret_cast<Collider*>(exportTo.size());
        for (auto& i : colliders)
        {
            exportTo.emplace_back(i);
        }

        for (auto& i : children)
        {
            i.exportColliders(exportTo);
        }
    }

    auto ColliderTree::remapColliders(Collider* start, Aabb* startAabb) -> void
    {
        vectorA16<Collider> tmp;
        colliders.swap(tmp);
        const auto offset = reinterpret_cast<size_t>(cbuf);
        cbuf = start + offset;
        aabb = startAabb + offset;

        for (auto& i : children)
        {
            i.remapColliders(start, startAabb);
        }
    }
} // namespace hdt
