#ifndef SOLVER
#define SOLVER

#include <unordered_set>
#include <queue>
#include <stack>

#include "jug_state_space.h"

namespace jug
{
    template <class input>
    struct collection_wrap
    {
        using output_t = typename input::value_type;
        output_t peek(input&) { return {}; }
    };

    template <>
    struct collection_wrap<std::stack<jug_state_space::value_type>>
    {
        jug_state_space::value_type peek(std::stack<jug_state_space::value_type>& stack) { return stack.top(); }
    };

    template <>
    struct collection_wrap<std::queue<jug_state_space::value_type>>
    {
        jug_state_space::value_type peek(std::queue<jug_state_space::value_type>& queue) { return queue.front(); }
    };

    template <class fringe_collection>
    struct solver
    {
        using fringe_t = typename fringe_collection;
        jug_state_space::value_type resolve(const jug_state_space& state_space) const noexcept
        {
            auto start = state_space.start_state();
            //V, visited nodes
            std::unordered_set<jug_state::value_type, jug_state::value_hasher> closed = { start->value() };
            //S, frontier (fringe) nodes, queue for BFS, stack for DFS
            fringe_t opened{};
            opened.push(start);
            collection_wrap<fringe_collection> opened_wrap;

            while (!opened.empty())
            {
                auto v = opened_wrap.peek(opened);
                opened.pop();
                if (state_space.is_goal_state(v))
                {
                    //solution found, stop the algorithm
                    return  v;
                }

                for (auto u : state_space.get_moves_from(v))
                {
                    if (!closed.contains(u->value()))
                    {
                        closed.insert(u->value());
                        opened.push(u);
                    }
                }
            }

            // solution not found
            return {};
        }
        
    };

    struct depth_first_search_solver final : public solver<std::stack<jug_state_space::value_type>>
    {};

    struct breadth_first_search_solver final : public solver<std::queue<jug_state_space::value_type>>
    {};
}

#endif
