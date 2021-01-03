#include "jug_state_space.h"
#include "solver.h"

#include <iostream>

void print(const jug::jug_state_space::result& result, const char* s)
{

    std::cout << std::endl << s << std::endl;
    bool first_taken = false;
    for (auto s : result)
    {
        if (first_taken)
        {
            std::cout << " -> ";
        }
        first_taken = true;
        std::cout << "[" << s.first << ", " << s.second << "]";
    }

    if (!first_taken)
    {
        std::cout << "Solution not found!";
    }

    std::cout << std::endl;
}

void print(const jug::problem_input& input)
{
    std::cout << "Water Jug Puzzle" << std::endl
        << "Goal volume: " << input.goal_volume << std::endl
        << "Jug capacities " << input.first_jug_capacity << " and " << input.second_jug_capacity << std::endl;
}

int main()
{
    jug::problem_input input{
     .first_jug_capacity = 5,
     .second_jug_capacity = 3,
     .goal_volume = 4
    };

    print(input);


    jug::jug_state_space space(input);

    print(space.resolve<jug::depth_first_search_solver>(), "Depth first search result:");
    print(space.resolve<jug::breadth_first_search_solver>(), "Breadth first search result");
}

