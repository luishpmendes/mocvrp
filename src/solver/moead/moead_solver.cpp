#include "solver/moead/moead_solver.hpp"
#include "solver/moead/problem.hpp"
#include <pagmo/algorithms/moead.hpp>

namespace mocvrp {

MOEAD_Solver::MOEAD_Solver(const Instance & instance)
    : Solver::Solver(instance) {}

MOEAD_Solver::MOEAD_Solver() = default;

void MOEAD_Solver::solve() {
    this->start_time = std::chrono::steady_clock::now();

    pagmo::problem prob{Problem(&this->instance)};

    // A single generation per evolution, so that the termination criteria and
    // the snapshots remain under the control of the solver. The weights of the
    // decomposed problems are rebuilt at every evolution, so the random and the
    // low discrepancy generations behave differently here: the former draws a
    // new set of weights, and therefore a new neighbourhood, at every
    // generation, while the latter keeps the same ones throughout.
    pagmo::algorithm algo{pagmo::moead(1,
                                       this->weight_generation,
                                       this->decomposition,
                                       this->neighbours,
                                       this->cr,
                                       this->f,
                                       this->eta_m,
                                       this->realb,
                                       this->limit,
                                       this->preserve_diversity,
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

std::ostream & operator <<(std::ostream & os, const MOEAD_Solver & solver) {
    os << static_cast<const Solver &>(solver)
       << "Population size: " << solver.population_size << std::endl
       << "Method used to generate the weights: " << solver.weight_generation
       << std::endl
       << "Decomposition method: " << solver.decomposition << std::endl
       << "Size of the weight's neighborhood: " << solver.neighbours
       << std::endl
       << "Crossover parameter in the Differential Evolution operator: "
       << solver.cr << std::endl
       << "Parameter for the Differential Evolution operator: " << solver.f
       << std::endl
       << "Distribution index used by the polynomial mutation: " << solver.eta_m
       << std::endl
       << "Chance that the neighbourhood is considered at each generation, "
       << "rather than the whole population: " << solver.realb << std::endl
       << "Maximum number of copies reinserted in the population: "
       << solver.limit << std::endl
       << "Diversity preservation mechanisms activated: "
       << solver.preserve_diversity << std::endl;
    return os;
}

}
