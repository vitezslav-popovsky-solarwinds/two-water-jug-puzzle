#include "jug_state_space.h"

namespace jug
{
    const jug_state_space::value_type jug_state_space::start_state() const noexcept
    {
        return std::make_shared<jug_state>(0, 0, nullptr);
    }

    bool jug_state_space::is_goal_state(const jug_state_space::value_type& state) const noexcept
    {
        return state->first_jug_value() == goal_volume_ || state->second_jug_value() == goal_volume_;
    }

    jug_state_space::move_generator jug_state_space::get_moves_from(const jug_state_space::value_type& parent) const noexcept
    {
        // empty first jug
        if (parent->first_jug_value() > 0)
        {
            co_yield std::make_shared<jug_state>(0, parent->second_jug_value(), parent);
        }

        // empty second jug
        if (parent->second_jug_value() > 0)
        {
            co_yield std::make_shared<jug_state>(parent->first_jug_value(), 0, parent);
        }

        // fill first jug
        if (parent->first_jug_value() < first_jug_capacity_)
        {
            co_yield std::make_shared<jug_state>(first_jug_capacity_, parent->second_jug_value(), parent);
        }

        // fill second jug
        if (parent->second_jug_value() < second_jug_capacity_)
        {
            co_yield std::make_shared<jug_state>(parent->first_jug_value(), second_jug_capacity_, parent);
        }

        uint_fast32_t delta;
        // pour from first into second
        if ((delta = std::min(parent->first_jug_value(), second_jug_capacity_ - parent->second_jug_value())) > 0)
        {
            co_yield std::make_shared<jug_state>(parent->first_jug_value() - delta, parent->second_jug_value() + delta, parent);
        }

        // pour from second into first
        if ((delta = std::min(parent->second_jug_value(), first_jug_capacity_ - parent->first_jug_value())) > 0)
        {
            co_yield std::make_shared<jug_state>(parent->first_jug_value() + delta, parent->second_jug_value() - delta, parent);
        }

        co_return;
    }
}

