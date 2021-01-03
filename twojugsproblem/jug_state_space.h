#ifndef JUG_STATE_SPACE
#define JUG_STATE_SPACE

#include <coroutine>
#include <memory>

namespace jug
{
    struct problem_input final
    {
        const uint_fast32_t first_jug_capacity;
        const uint_fast32_t second_jug_capacity;
        const uint_fast32_t goal_volume;
    };

    class jug_state final
    {
    public:
        using value_type = std::pair<uint_fast32_t, uint_fast32_t>;

        struct value_hasher final {
            size_t operator()(const value_type& p) const noexcept {
                size_t hash_value = 1009;
                hash_value = hash_value * 9199 + p.first;
                hash_value = hash_value * 9199 + p.second;
                return hash_value;
            }
        };

        constexpr uint_fast32_t first_jug_value() const noexcept { return value_.first; }
        constexpr uint_fast32_t second_jug_value() const noexcept { return value_.second; }

        constexpr uint_fast32_t depth() const noexcept { return depth_; }

        constexpr value_type value() const noexcept { return value_; }

        jug_state(uint_fast32_t first_jug_value, uint_fast32_t second_jug_value, const std::shared_ptr<const jug_state> parent) :
            value_{ std::make_pair(first_jug_value, second_jug_value) }, parent_{ parent }, depth_{ get_depth(parent) + 1 }
        {}

        jug_state(const jug_state& other) = default;

        ~jug_state() noexcept = default;

        const std::shared_ptr<const jug_state> parent() const { return parent_; }

        const static uint_fast32_t get_depth(const std::shared_ptr<const jug_state> state) noexcept
        {
            return state ? state->depth_ : 0;
        }

    private:
        const std::shared_ptr<const jug_state> parent_;
        const value_type value_;
        const uint_fast32_t depth_;
    };

    class jug_state_space;

    template<typename solver>
    concept is_solver = requires(solver t, std::shared_ptr<const  jug_state> s, jug_state_space ss) {
        s = t.resolve(ss);
    };

    class jug_state_space final
    {
    private:
        const uint_fast32_t goal_volume_;
        const uint_fast32_t first_jug_capacity_;
        const uint_fast32_t second_jug_capacity_;
    public:

        class move_generator;
        class move_generator_promise;
        class move_generator_iterator;
        using value_type = std::shared_ptr<const jug_state>;

        struct result final
        {
            constexpr jug_state::value_type* begin() const noexcept
            {
                return first;
            }

            constexpr jug_state::value_type* end() const noexcept
            {
                return sentinel;
            }

            ~result()
            {
                delete[] first;
                first = sentinel = nullptr;
            }

            jug_state::value_type* first;
            jug_state::value_type* sentinel;
        };

        explicit jug_state_space(const problem_input& input) noexcept :
            goal_volume_{ input.goal_volume }, first_jug_capacity_{ input.first_jug_capacity }, second_jug_capacity_{ input.second_jug_capacity }
        {}

        const value_type start_state() const noexcept;

        bool is_goal_state(const value_type&) const noexcept;

        move_generator get_moves_from(const value_type&) const noexcept;

        template<typename S>
        requires is_solver<S>
            const result resolve() const noexcept
        {
            const value_type& goal = S{}.resolve(*this);

            auto depth = jug_state::get_depth(goal);
            auto first = new jug_state::value_type[depth + 1];

            // traverse from goal to root vertex
            auto i = depth;
            value_type next = goal;
            while (next)
            {
                first[--i] = next->value();
                next = next->parent();
            }

            return result{
                .first = first,
                .sentinel = &first[depth]
            };
        }

        class move_generator_promise final
        {
        public:
            using value_type = jug_state_space::value_type;
            move_generator_promise() = default;
            // result from coroutine
            move_generator get_return_object() noexcept
            {
                return move_generator{ std::coroutine_handle<move_generator_promise>::from_promise(*this) };
            }
            constexpr std::suspend_always initial_suspend() const { return {}; }
            constexpr std::suspend_always final_suspend() const noexcept { return {}; }
            // input from coroutine co_yield
            std::suspend_always yield_value(const value_type& value)
            {
                value_ = const_cast<value_type*>(std::addressof(value));
                return {};
            }
            void return_void() {}
            void unhandled_exception()
            {
                std::rethrow_exception(std::current_exception());
            }

            value_type& value() noexcept
            {
                return *value_;
            }
        private:
            value_type* value_;
        };

        class move_generator_iterator final
        {
            using coroutine_handle = std::coroutine_handle<move_generator_promise>;
            coroutine_handle coroutine_;

        public:
            using iterator_category = std::input_iterator_tag;
            using difference_type = std::ptrdiff_t;
            using value_type = jug_state_space::value_type;
            using reference = value_type&;
            using pointer = value_type*;

            move_generator_iterator() = default;

            explicit move_generator_iterator(std::nullptr_t) noexcept
                : coroutine_(nullptr)
            {}

            explicit move_generator_iterator(coroutine_handle coroutine) noexcept
                : coroutine_(coroutine)
            {}

            bool operator==(const move_generator_iterator& other) const noexcept
            {
                return (!coroutine_ && !other.coroutine_);
            }

            bool operator!=(const move_generator_iterator& other) const noexcept
            {
                return !(*this == other);
            }

            move_generator_iterator& operator++()
            {
                coroutine_.resume();
                if (coroutine_.done())
                {
                    coroutine_ = nullptr;
                }

                return *this;
            }

            move_generator_iterator operator++(int) = delete;

            reference operator*() const noexcept
            {
                return coroutine_.promise().value();
            }

            pointer operator->() const noexcept
            {
                return std::addressof(operator*());
            }
        };

        class move_generator final
        {
        public:

            using promise_type = move_generator_promise;
            using iterator = move_generator_iterator;

            move_generator() noexcept : coroutine_{ nullptr }
            {}

            move_generator(const move_generator&) = delete;
            move_generator& operator=(const move_generator&) = delete;

            move_generator(move_generator&& other) noexcept : coroutine_{ other.coroutine_ } {
                other.coroutine_ = nullptr;
            }

            move_generator& operator=(move_generator&& other) noexcept {
                if (this != std::addressof(other)) {
                    coroutine_ = other.coroutine_;
                    other.coroutine_ = nullptr;
                }
                return *this;
            }

            ~move_generator()
            {
                if (coroutine_)
                {
                    coroutine_.destroy();
                }
            }

            iterator begin() noexcept
            {
                if (coroutine_)
                {
                    coroutine_.resume();
                    if (!coroutine_.done())
                    {
                        return iterator{ coroutine_ };
                    }
                }

                return iterator{ nullptr };
            }

            iterator end() noexcept
            {
                return iterator{ nullptr };
            }

        private:

            friend class move_generator_promise;

            explicit move_generator(std::coroutine_handle<promise_type> coroutine) noexcept
                : coroutine_{ coroutine }
            {}

            std::coroutine_handle<promise_type> coroutine_;

        };
    };
}

#endif
