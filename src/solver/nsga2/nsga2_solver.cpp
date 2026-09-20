#include "solver/nsga2/nsga2_solver.hpp"
#include "solver/nsga2/problem.hpp"
#include <pagmo/algorithms/nsga2.hpp>

namespace mocvrp {

NSGA2_Solver::NSGA2_Solver(const Instance & instance)
    : Solver::Solver(instance) {}

NSGA2_Solver::NSGA2_Solver() = default;

void NSGA2_Solver::solve() {
    this->start_time = std::chrono::steady_clock::now();

    pagmo::problem prob{Problem(&this->instance)};

    // A single generation per evolution, so that the termination criteria and
    // the snapshots remain under the control of the solver.
    pagmo::algorithm algo{pagmo::nsga2(1,
                                       this->crossover_probability,
                                       this->crossover_distribution,
                                       this->mutation_probability,
                                       this->mutation_distribution,
                                       this->seed)};

    std::size_t num_random_individuals = 0;

    if (this->initial_individuals.size() < this->population_size) {
        num_random_individuals = this->population_size -
                                 this->initial_individuals.size();
    }

    pagmo::population pop{prob, num_random_individuals, this->seed};

    // The population stores the decision vector first and its value second,
    // the other way around from the initial individuals.
    for (const auto & individual : this->initial_individuals) {
        std::vector<double> value(individual.first);

        Solution::negate_maximized(value, this->instance.senses);

        pop.push_back(individual.second, value);
    }

    this->update_best_individuals(pop);

    if(this->max_num_snapshots > this->num_snapshots + 1) {
        this->capture_snapshot(pop);

        if (this->time_limit < std::numeric_limits<double>::max()) {
            this->time_snapshot_factor = std::pow(this->time_limit / this->time_last_snapshot, 1.0 / (this->max_num_snapshots - this->num_snapshots));
            this->time_next_snapshot = this->time_last_snapshot * this->time_snapshot_factor;
        } else {
            this->time_next_snapshot = std::numeric_limits<double>::max();
            this->time_snapshot_factor = 1.0;
        }

        if (this->iterations_limit < std::numeric_limits<unsigned>::max()) {
            this->iteration_snapshot_factor = std::pow(this->iterations_limit / (this->iteration_last_snapshot + 1.0), 1.0 / (this->max_num_snapshots - this->num_snapshots));
            this->iteration_next_snapshot = unsigned(std::round(double(this->iteration_last_snapshot) * this->iteration_snapshot_factor));
        } else {
            this->iteration_next_snapshot = std::numeric_limits<unsigned>::max();
            this->iteration_snapshot_factor = 1.0;
        }
    } else {
        this->time_next_snapshot = 0.0;
        this->iteration_next_snapshot = 0;
        this->time_snapshot_factor = 1.0;
        this->iteration_snapshot_factor = 1.0;
    }

    while (!this->are_termination_criteria_met()) {
        this->num_iterations++;

        pop = algo.evolve(pop);

        this->update_best_individuals(pop);

        if(this->max_num_snapshots > this->num_snapshots + 1) {
            if (this->num_iterations >= this->iteration_next_snapshot) {
                this->capture_snapshot(pop);

                if (this->time_limit < std::numeric_limits<double>::max()) {
                    this->time_next_snapshot = this->time_last_snapshot * this->time_snapshot_factor;
                    this->time_snapshot_factor = std::pow(this->time_limit / this->time_last_snapshot, 1.0 / (this->max_num_snapshots - this->num_snapshots));
                }

                if (this->iterations_limit < std::numeric_limits<unsigned>::max()) {
                    this->iteration_next_snapshot = unsigned(std::round(double(this->iteration_last_snapshot) * this->iteration_snapshot_factor));
                    this->iteration_snapshot_factor = std::pow(this->iterations_limit / this->iteration_last_snapshot, 1.0 / (this->max_num_snapshots - this->num_snapshots));
                }
            } else if (this->elapsed_time() >= this->time_next_snapshot) {
                this->capture_snapshot(pop);

                if (this->time_limit < std::numeric_limits<double>::max()) {
                    this->time_next_snapshot = this->time_last_snapshot * this->time_snapshot_factor;
                    this->time_snapshot_factor = std::pow(this->time_limit / this->time_last_snapshot, 1.0 / (this->max_num_snapshots - this->num_snapshots));
                }

                if (this->iterations_limit < std::numeric_limits<unsigned>::max()) {
                    this->iteration_next_snapshot = unsigned(std::round(double(this->iteration_last_snapshot) * this->iteration_snapshot_factor));
                    this->iteration_snapshot_factor = std::pow(this->iterations_limit / this->iteration_last_snapshot, 1.0 / (this->max_num_snapshots - this->num_snapshots));
                }
            }
        }
    }

    if (this->max_num_snapshots > 0) {
        this->capture_snapshot(pop);
    }

    this->best_solutions.clear();

    for (const auto & best_individual : this->best_individuals) {
        this->best_solutions.push_back(Solution(this->instance,
                                                best_individual.second));
    }

    this->solving_time = this->elapsed_time();
}

std::ostream & operator <<(std::ostream & os, const NSGA2_Solver & solver) {
    os << static_cast<const Solver &>(solver)
       << "Population size: " << solver.population_size << std::endl
       << "Crossover probability: " << solver.crossover_probability << std::endl
       << "Distribution index for crossover: " << solver.crossover_distribution
       << std::endl
       << "Mutation probability: " << solver.mutation_probability << std::endl
       << "Distribution index for mutation: " << solver.mutation_distribution
       << std::endl;
    return os;
}

}
