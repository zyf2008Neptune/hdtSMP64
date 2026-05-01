#include "hdtPhysicsProfiler.h"

#include <LinearMath/btQuickprof.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace hdt::physicsprofiler
{
    using Clock = std::chrono::steady_clock;

    namespace
    {
        struct Node
        {
            std::string m_name;
            std::uint64_t m_totalNs{};
            std::uint64_t m_calls{};
            std::vector<std::unique_ptr<Node>> m_children;

            explicit Node(std::string a_name) : m_name(std::move(a_name)) {}

            auto child(std::string_view a_name) -> Node&
            {
                if (auto it = std::ranges::find_if(m_children,
                                                   [&](const auto& a_entry) { return a_entry->m_name == a_name; });
                    it != m_children.end())
                {
                    return **it;
                }

                m_children.emplace_back(std::make_unique<Node>(std::string(a_name)));
                return *m_children.back();
            }

            auto resetStats() -> void
            {
                m_totalNs = 0;
                m_calls = 0;

                for (auto& child : m_children)
                {
                    child->resetStats();
                }
            }
        };
    } // namespace

    namespace
    {
        struct Frame
        {
            Node* m_node{};
            Clock::time_point m_start{};
        };
    } // namespace

    namespace
    {
        struct ThreadState
        {
            std::uint32_t m_index{};
            Node m_root{"Root"};
            std::vector<Frame> m_stack;

            explicit ThreadState(std::uint32_t a_index) : m_index(a_index) {}
        };
    } // namespace

    namespace
    {
        struct AggregateNode
        {
            std::string m_name;
            std::uint64_t m_totalNs{};
            std::uint64_t m_calls{};
            std::vector<std::unique_ptr<AggregateNode>> m_children;

            explicit AggregateNode(std::string a_name) : m_name(std::move(a_name)) {}

            auto child(std::string_view a_name) -> AggregateNode&
            {
                if (auto it = std::ranges::find_if(m_children,
                                                   [&](const auto& a_entry) { return a_entry->m_name == a_name; });
                    it != m_children.end())
                {
                    return **it;
                }

                m_children.emplace_back(std::make_unique<AggregateNode>(std::string(a_name)));
                return *m_children.back();
            }
        };
    } // namespace

    namespace
    {
        struct FlatRow
        {
            std::string m_name;
            std::uint64_t m_totalNs{};
            std::uint64_t m_calls{};
        };
    } // namespace

    namespace
    {
        struct ThreadRow
        {
            std::uint32_t m_index{};
            std::uint64_t m_totalNs{};
            std::uint64_t m_calls{};
        };

    } // namespace

    namespace
    {
        std::mutex g_threadsLock;
        std::vector<std::unique_ptr<ThreadState>> g_threads;
        std::atomic_uint32_t g_nextThreadIndex{0};
        std::atomic_uint32_t g_activeScopes{0};
        std::atomic_uint64_t g_windowFrames{0};
        std::atomic_uint64_t g_totalFrames{0};
        std::atomic_uint64_t g_profileHistory{0};
        Clock::time_point g_windowStart = Clock::now();

        thread_local ThreadState* g_threadState = nullptr;
    } // namespace

    namespace
    {
        auto emptyEnter(const char*) noexcept -> void {}
    } // namespace

    namespace
    {
        auto emptyLeave() noexcept -> void {}
    } // namespace

    namespace
    {
        auto nsToMs(std::uint64_t a_ns) -> double { return static_cast<double>(a_ns) / 1'000'000.0; }
    } // namespace

    namespace
    {
        auto elapsedNs(Clock::time_point a_start, Clock::time_point a_end) -> std::uint64_t
        {
            return static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(a_end - a_start).count());
        }
    } // namespace

    namespace
    {
        auto getThreadState() -> ThreadState&
        {
            if (g_threadState)
            {
                return *g_threadState;
            }

            auto threadState = std::make_unique<ThreadState>(g_nextThreadIndex.fetch_add(1, std::memory_order_relaxed));
            g_threadState = threadState.get();

            std::scoped_lock lock(g_threadsLock);
            g_threads.emplace_back(std::move(threadState));

            return *g_threadState;
        }
    } // namespace

    namespace
    {
        auto getChildTotal(const Node& a_node) -> std::uint64_t
        {
            std::uint64_t total = 0;

            for (const auto& child : a_node.m_children)
            {
                total += child->m_totalNs;
            }

            return total;
        }
    } // namespace

    namespace
    {
        auto getChildCalls(const Node& a_node) -> std::uint64_t
        {
            std::uint64_t calls = 0;

            for (const auto& child : a_node.m_children)
            {
                calls += child->m_calls;
            }

            return calls;
        }
    } // namespace

    namespace
    {
        auto getChildTotal(const AggregateNode& a_node) -> std::uint64_t
        {
            std::uint64_t total = 0;

            for (const auto& child : a_node.m_children)
            {
                total += child->m_totalNs;
            }

            return total;
        }
    } // namespace

    namespace
    {
        auto mergeNode(AggregateNode& a_dst, const Node& a_src) -> void
        {
            a_dst.m_totalNs += a_src.m_totalNs;
            a_dst.m_calls += a_src.m_calls;

            for (const auto& child : a_src.m_children)
            {
                mergeNode(a_dst.child(child->m_name), *child);
            }
        }
    } // namespace

    namespace
    {
        auto mergeRoot(AggregateNode& a_dst, const Node& a_root) -> void
        {
            for (const auto& child : a_root.m_children)
            {
                mergeNode(a_dst.child(child->m_name), *child);
            }
        }
    } // namespace

    namespace
    {
        auto addFlatRows(std::unordered_map<std::string, FlatRow>& a_rows, const AggregateNode& a_node) -> void
        {
            if (a_node.m_name != "Recorded CPU")
            {
                auto& row = a_rows[a_node.m_name];
                row.m_name = a_node.m_name;
                row.m_totalNs += a_node.m_totalNs;
                row.m_calls += a_node.m_calls;
            }

            for (const auto& child : a_node.m_children)
            {
                addFlatRows(a_rows, *child);
            }
        }
    } // namespace

    namespace
    {
        auto logRow(std::string_view a_scope, std::uint64_t a_ns, std::uint64_t a_parentNs, std::uint64_t a_calls,
                    std::uint64_t a_frames) -> void
        {
            const auto totalMs = nsToMs(a_ns);
            const auto msPerFrame = a_frames > 0 ? totalMs / static_cast<double>(a_frames) : 0.0;
            const auto msPerCall = a_calls > 0 ? totalMs / static_cast<double>(a_calls) : 0.0;
            const auto pct = a_parentNs > 0 ? static_cast<double>(a_ns) * 100.0 / static_cast<double>(a_parentNs) : 0.0;

            if (a_calls > 0)
            {
                logger::info("{:<80} {:>12.3f} ms {:>10.3f} ms/frame {:>10.3f} ms/call {:>8.2f}% {:>8}", a_scope,
                             totalMs, msPerFrame, msPerCall, pct, a_calls);
            }
            else
            {
                logger::info("{:<80} {:>12.3f} ms {:>10.3f} ms/frame {:>17} {:>8.2f}% {:>8}", a_scope, totalMs,
                             msPerFrame, "-", pct, "-");
            }
        }
    } // namespace

    namespace
    {
        auto logTreeRecursive(const AggregateNode& a_node, std::string a_prefix, std::uint32_t a_depth,
                              std::uint64_t a_frames, const DumpOptions& a_options) -> void
        {
            if (a_depth >= a_options.m_maxDepth)
            {
                return;
            }

            std::vector<const AggregateNode*> children;
            children.reserve(a_node.m_children.size());

            for (const auto& child : a_node.m_children)
            {
                if (nsToMs(child->m_totalNs) >= a_options.m_minScopeMs)
                {
                    children.emplace_back(child.get());
                }
            }

            std::ranges::sort(children,
                              [](const auto* a_lhs, const auto* a_rhs) { return a_lhs->m_totalNs > a_rhs->m_totalNs; });

            for (std::size_t i = 0; i < children.size(); ++i)
            {
                const auto* child = children[i];
                const bool last = i + 1 == children.size();
                const auto scope = a_prefix + (last ? "`-- " : "|-- ") + child->m_name;

                logRow(scope, child->m_totalNs, a_node.m_totalNs, child->m_calls, a_frames);
                logTreeRecursive(*child, a_prefix + (last ? "    " : "|   "), a_depth + 1, a_frames, a_options);
            }

            if (a_node.m_name == "Recorded CPU")
            {
                return;
            }

            const auto accounted = getChildTotal(a_node);

            if (a_node.m_totalNs > accounted)
            {
                const auto self = a_node.m_totalNs - accounted;

                if (nsToMs(self) >= a_options.m_minScopeMs)
                {
                    logRow(a_prefix + "`-- [self]", self, a_node.m_totalNs, 0, a_frames);
                }
            }
        }
    } // namespace

    namespace
    {
        auto logTreeHeader(const AggregateNode& a_root, std::uint64_t a_frames, Clock::time_point a_windowStart) -> void
        {
            const auto wallNs = elapsedNs(a_windowStart, Clock::now());
            const auto wallMs = nsToMs(wallNs);
            const auto cpuMs = nsToMs(a_root.m_totalNs);
            const auto parallelism =
                wallNs > 0 ? static_cast<double>(a_root.m_totalNs) / static_cast<double>(wallNs) : 0.0;

            logger::info(
                "Physics profile: {} frames, {:.3f} ms wall, {:.3f} ms recorded CPU, {:.2f}x recorded parallelism",
                a_frames, wallMs, cpuMs, parallelism);
            logger::info("{:<80} {:>16} {:>18} {:>18} {:>9} {:>8}", "Scope", "Total CPU", "Avg/frame", "Avg/call",
                         "Parent%", "Calls");
            logger::info("{:-<154}", "");

            logRow("Recorded CPU", a_root.m_totalNs, a_root.m_totalNs, 0, a_frames);
        }
    } // namespace

    namespace
    {
        auto logFlatRows(const AggregateNode& a_root, std::uint64_t a_frames, const DumpOptions& a_options) -> void
        {
            std::unordered_map<std::string, FlatRow> map;
            addFlatRows(map, a_root);

            std::vector<FlatRow> rows;
            rows.reserve(map.size());

            for (auto& row : map | std::views::values)
            {
                if (nsToMs(row.m_totalNs) >= a_options.m_minScopeMs)
                {
                    rows.emplace_back(std::move(row));
                }
            }

            std::ranges::sort(rows,
                              [](const auto& a_lhs, const auto& a_rhs) { return a_lhs.m_totalNs > a_rhs.m_totalNs; });

            logger::info("Physics profile flat hotspots");
            logger::info("{:<60} {:>12} {:>18} {:>18} {:>8}", "Scope", "Total CPU", "Avg/frame", "Avg/call", "Calls");
            logger::info("{:-<122}", "");

            for (std::size_t i = 0; i < std::min<std::size_t>(rows.size(), a_options.m_flatLimit); ++i)
            {
                const auto& row = rows[i];
                const auto totalMs = nsToMs(row.m_totalNs);
                const auto msPerFrame = a_frames > 0 ? totalMs / static_cast<double>(a_frames) : 0.0;
                const auto msPerCall = row.m_calls > 0 ? totalMs / static_cast<double>(row.m_calls) : 0.0;

                logger::info("{:<60} {:>10.3f} ms {:>10.3f} ms/frame {:>10.3f} ms/call {:>8}", row.m_name, totalMs,
                             msPerFrame, msPerCall, row.m_calls);
            }
        }
    } // namespace

    namespace
    {
        auto logThreads(std::vector<ThreadRow>& a_rows, std::uint64_t a_frames) -> void
        {
            std::ranges::sort(a_rows,
                              [](const auto& a_lhs, const auto& a_rhs) { return a_lhs.m_totalNs > a_rhs.m_totalNs; });

            logger::info("Physics profile threads");
            logger::info("{:<12} {:>12} {:>18} {:>8}", "Thread", "Total CPU", "Avg/frame", "Calls");
            logger::info("{:-<62}", "");

            for (const auto& row : a_rows)
            {
                const auto totalMs = nsToMs(row.m_totalNs);
                const auto msPerFrame = a_frames > 0 ? totalMs / static_cast<double>(a_frames) : 0.0;

                logger::info("T{:<10} {:>10.3f} ms {:>10.3f} ms/frame {:>8}", row.m_index, totalMs, msPerFrame,
                             row.m_calls);
            }
        }
    } // namespace

    namespace
    {
        auto resetUnlocked(bool a_resetTotalFrames) -> void
        {
            for (auto& thread : g_threads)
            {
                thread->m_root.resetStats();
                thread->m_stack.clear();
            }

            g_windowFrames.store(0, std::memory_order_relaxed);

            if (a_resetTotalFrames)
            {
                g_totalFrames.store(0, std::memory_order_relaxed);
            }

            g_windowStart = Clock::now();
        }
    } // namespace

    auto install() -> void
    {
        {
            std::scoped_lock lock(g_threadsLock);
            resetUnlocked(true);
        }

        btSetCustomEnterProfileZoneFunc(&enter);
        btSetCustomLeaveProfileZoneFunc(&leave);
    }

    auto uninstall() -> void
    {
        btSetCustomEnterProfileZoneFunc(&emptyEnter);
        btSetCustomLeaveProfileZoneFunc(&emptyLeave);
    }

    auto enter(const char* a_name) noexcept -> void
    {
        if (!a_name)
        {
            return;
        }

        g_activeScopes.fetch_add(1, std::memory_order_acq_rel);

        try
        {
            auto& thread = getThreadState();
            auto& parent = thread.m_stack.empty() ? thread.m_root : *thread.m_stack.back().m_node;
            auto& node = parent.child(a_name);

            ++node.m_calls;
            thread.m_stack.emplace_back(&node, Clock::now());
        }
        catch (...)
        {
            g_activeScopes.fetch_sub(1, std::memory_order_acq_rel);
        }
    }

    auto leave() noexcept -> void
    {
        try
        {
            auto* thread = g_threadState;

            if (!thread || thread->m_stack.empty())
            {
                return;
            }

            const auto now = Clock::now();
            const auto frame = thread->m_stack.back();

            thread->m_stack.pop_back();
            frame.m_node->m_totalNs += elapsedNs(frame.m_start, now);

            g_activeScopes.fetch_sub(1, std::memory_order_acq_rel);
        }
        catch (...)
        {}
    }

    auto endFrame() -> void
    {
        g_windowFrames.fetch_add(1, std::memory_order_relaxed);
        g_totalFrames.fetch_add(1, std::memory_order_relaxed);
    }

    auto frameCountSinceReset() -> std::uint64_t { return g_windowFrames.load(std::memory_order_relaxed); }

    auto totalFrameCount() -> std::uint64_t { return g_totalFrames.load(std::memory_order_relaxed); }

    auto setProfileHistory(std::uint64_t a_frames) -> void
    {
        g_profileHistory.store(a_frames, std::memory_order_relaxed);
    }

    auto getProfileHistory() -> std::uint64_t { return g_profileHistory.load(std::memory_order_relaxed); }

    auto reset() -> bool
    {
        if (g_activeScopes.load(std::memory_order_acquire) != 0)
        {
            return false;
        }

        std::scoped_lock lock(g_threadsLock);
        resetUnlocked(true);

        return true;
    }

    auto dumpAndReset(const DumpOptions& a_options) -> bool
    {
        const auto activeScopes = g_activeScopes.load(std::memory_order_acquire);

        if (activeScopes != 0)
        { // Just debug in case we fucked up somewhere
            logger::warn("Physics profile dump skipped because {} profile scopes are still active", activeScopes);
            return false;
        }

        AggregateNode root{"Recorded CPU"};
        std::vector<ThreadRow> threads;
        std::uint64_t frames = 0;
        Clock::time_point windowStart;

        {
            std::scoped_lock lock(g_threadsLock);

            frames = std::max<std::uint64_t>(1, g_windowFrames.load(std::memory_order_relaxed));
            windowStart = g_windowStart;

            for (const auto& thread : g_threads)
            {
                const auto total = getChildTotal(thread->m_root);
                const auto calls = getChildCalls(thread->m_root);

                if (total == 0 && calls == 0)
                {
                    continue;
                }

                threads.emplace_back(thread->m_index, total, calls);
                mergeRoot(root, thread->m_root);
            }

            root.m_totalNs = getChildTotal(root);
            resetUnlocked(false);
        }

        if (root.m_totalNs == 0)
        {
            logger::info("Physics profile had no recorded scopes");
            return true;
        }

        if (a_options.m_logTree)
        {
            logTreeHeader(root, frames, windowStart);
            logTreeRecursive(root, "", 0, frames, a_options);
        }

        if (a_options.m_logFlat)
        {
            logFlatRows(root, frames, a_options);
        }

        if (a_options.m_logThreads)
        {
            logThreads(threads, frames);
        }

        return true;
    }

    auto dumpEvery(std::uint64_t a_frames, const DumpOptions& a_options) -> bool
    {
        if (a_frames == 0)
        {
            return false;
        }

        const auto totalFrames = totalFrameCount();

        if (totalFrames != 0 && totalFrames % a_frames == 0)
        {
            return dumpAndReset(a_options);
        }

        const auto profileHistory = getProfileHistory();

        if (profileHistory != 0 && frameCountSinceReset() >= profileHistory)
        {
            if (g_activeScopes.load(std::memory_order_acquire) == 0)
            {
                std::scoped_lock lock(g_threadsLock);
                resetUnlocked(false);
            }
        }

        return false;
    }
} // namespace hdt::physicsprofiler
