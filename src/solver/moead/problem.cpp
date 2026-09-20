#include "solver/moead/problem.hpp"
#include "solution/solution.hpp"

namespace mocvrp {

Problem::Problem(const Instance * instance) : instance(instance) {}

Problem::Problem() : instance(nullptr) {}

pagmo::vector_double Problem::fitness(const pagmo::vector_double & dv) const {
    Solution solution(*(this->instance), dv);

    Solution::negate_maximized(solution.value, this->instance->senses);

    return solution.value;
}

std::pair<pagmo::vector_double, pagmo::vector_double> Problem::get_bounds()
    const {
    return std::make_pair(
            pagmo::vector_double(2 * this->instance->num_customers, 0.0),
            pagmo::vector_double(2 * this->instance->num_customers, 1.0));
}

pagmo::vector_double::size_type Problem::get_nobj() const {
    return this->instance->num_objectives;
}

}
