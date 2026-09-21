#include "solver/nsga3/nsga3_solver.hpp"
#include <pagmo/algorithms/nsga3.hpp>
#include <pagmo/detail/reference_point.hpp>
#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>

int main() {
    std::ifstream ifs;
    mocvrp::Instance instance;
    mocvrp::NSGA3_Solver solver;

    for (const std::string filename : {"instances/X/X-n101-k25.vrp",
                                       "instances/X/X-n106-k14.vrp",
                                       "instances/X/X-n110-k13.vrp",
                                       "instances/X/X-n115-k10.vrp",
                                       "instances/X/X-n120-k6.vrp"}) {
        std::cout << filename << std::endl;

        ifs.open(filename);

        assert(ifs.is_open());

        ifs >> instance;

        ifs.close();

        solver = mocvrp::NSGA3_Solver(instance);

        solver.set_seed(2351389233);
        solver.time_limit = 5.0;
        solver.iterations_limit = 1000;
        solver.max_num_solutions = 128;
        solver.population_size = 32;
        solver.max_num_snapshots = 16;
        solver.divisions = 2;

        assert((solver.seed = 2351389233));
        assert(fabs(solver.time_limit - 5.0) <
            std::numeric_limits<double>::epsilon());
        assert(solver.iterations_limit == 1000);
        assert(solver.max_num_solutions == 128);
        assert(solver.population_size == 32);
        assert(solver.max_num_snapshots == 16);
        assert(fabs(solver.crossover_probability - 0.95) <
            std::numeric_limits<double>::epsilon());
        assert(fabs(solver.crossover_distribution - 10.0) <
            std::numeric_limits<double>::epsilon());
        assert(fabs(solver.mutation_probability - 0.01) <
            std::numeric_limits<double>::epsilon());
        assert(fabs(solver.mutation_distribution - 50.0) <
            std::numeric_limits<double>::epsilon());
        assert(solver.divisions == 2);
        assert(solver.divisions_inner == 0);
        assert(solver.random_mating);
        assert(solver.memory);

        solver.solve();

        assert(solver.solving_time > 0);

        assert(solver.num_iterations > 0);
        assert(solver.num_iterations <= solver.iterations_limit);

        assert(solver.best_solutions.size() > 0);
        assert(solver.best_solutions.size() <= solver.max_num_solutions);

        assert(solver.best_solutions.front().value.size() ==
                instance.num_objectives);

        assert(solver.num_snapshots == solver.max_num_snapshots);

        assert(solver.best_solutions_snapshots.size() == solver.num_snapshots);
        assert(solver.num_non_dominated_snapshots.size() == solver.num_snapshots);
        assert(solver.num_fronts_snapshots.size() == solver.num_snapshots);
        assert(solver.populations_snapshots.size() == solver.num_snapshots);

        for (const auto & s1 : solver.best_solutions) {
            assert(s1.is_feasible());

            for (const auto & s2 : solver.best_solutions) {
                assert(!s1.dominates(s2));
                assert(!s2.dominates(s1));
            }
        }

        // The values are negated on the way into pagmo and negated back on
        // the way out, so the best individuals must hold the values of the
        // solutions their chromosomes decode into.
        for (const auto & best_individual : solver.best_individuals) {
            assert(best_individual.first ==
                    mocvrp::Solution(instance, best_individual.second).value);
        }

        for (const auto & snapshot : solver.best_solutions_snapshots) {
            assert(std::get<0>(snapshot) >= 0);
            assert(std::get<0>(snapshot) <= solver.num_iterations);
            assert(std::get<1>(snapshot) >= 0.0);
            assert(std::get<1>(snapshot) <= solver.solving_time);
            assert(std::get<2>(snapshot).size() > 0);
            assert(std::get<2>(snapshot).size() <= solver.max_num_solutions);

            for (const auto & s : std::get<2>(snapshot)) {
                assert(s.size() == instance.num_objectives);
            }
        }

        for (const auto & snapshot : solver.num_non_dominated_snapshots) {
            assert(std::get<0>(snapshot) >= 0);
            assert(std::get<0>(snapshot) <= solver.num_iterations);
            assert(std::get<1>(snapshot) >= 0.0);
            assert(std::get<1>(snapshot) <= solver.solving_time);
            // Every pagmo algorithm evolves a single population.
            assert(std::get<2>(snapshot).size() == 1);

            for (const unsigned & num_non_dominated : std::get<2>(snapshot)) {
                assert(num_non_dominated > 0);
                assert(num_non_dominated <= solver.population_size);
            }
        }

        for (const auto & snapshot : solver.num_fronts_snapshots) {
            assert(std::get<0>(snapshot) >= 0);
            assert(std::get<0>(snapshot) <= solver.num_iterations);
            assert(std::get<1>(snapshot) >= 0.0);
            assert(std::get<1>(snapshot) <= solver.solving_time);
            assert(std::get<2>(snapshot).size() == 1);

            for (const unsigned & num_fronts : std::get<2>(snapshot)) {
                assert(num_fronts > 0);
                assert(num_fronts <= solver.population_size);
            }
        }

        for (const auto & snapshot : solver.populations_snapshots) {
            assert(std::get<0>(snapshot) >= 0);
            assert(std::get<0>(snapshot) <= solver.num_iterations);
            assert(std::get<1>(snapshot) >= 0.0);
            assert(std::get<1>(snapshot) <= solver.solving_time);
            assert(std::get<2>(snapshot).size() == 1);

            for (const auto & population : std::get<2>(snapshot)) {
                assert(population.size() == solver.population_size);

                for (const auto & s : population) {
                    assert(s.size() == instance.num_objectives);
                }
            }
        }

        std::cout << solver << std::endl;

        std::cout << "Num non dominated snapshots: ";
        for(unsigned i = 0;
            i < solver.num_non_dominated_snapshots.size() - 1;
            i++) {
            std::cout << "("
                    << std::get<0>(solver.num_non_dominated_snapshots[i])
                    << ", "
                    << std::get<1>(solver.num_non_dominated_snapshots[i])
                    << ", "
                    << std::accumulate(
                std::get<2>(solver.num_non_dominated_snapshots[i]).begin(),
                std::get<2>(solver.num_non_dominated_snapshots[i]).end(),
                0) / std::get<2>(solver.num_non_dominated_snapshots[i]).size()
                    << "), ";
        }
        std::cout << "("
                << std::get<0>(solver.num_non_dominated_snapshots.back())
                << ", "
                << std::get<1>(solver.num_non_dominated_snapshots.back())
                << ", "
                << std::accumulate(
            std::get<2>(solver.num_non_dominated_snapshots.back()).begin(),
            std::get<2>(solver.num_non_dominated_snapshots.back()).end(),
            0) / std::get<2>(solver.num_non_dominated_snapshots.back()).size()
                << ")" << std::endl;

        std::cout << "Num fronts snapshots: ";
        for(unsigned i = 0; i < solver.num_fronts_snapshots.size() - 1; i++) {
            std::cout << "("
                    << std::get<0>(solver.num_fronts_snapshots[i])
                    << ", "
                    << std::get<1>(solver.num_fronts_snapshots[i])
                    << ", "
                    << std::accumulate(
                std::get<2>(solver.num_fronts_snapshots[i]).begin(),
                std::get<2>(solver.num_fronts_snapshots[i]).end(),
                0) / std::get<2>(solver.num_fronts_snapshots[i]).size()
                    << "), ";
        }
        std::cout << "("
                << std::get<0>(solver.num_fronts_snapshots.back())
                << ", "
                << std::get<1>(solver.num_fronts_snapshots.back())
                << ", "
                << std::accumulate(
            std::get<2>(solver.num_fronts_snapshots.back()).begin(),
            std::get<2>(solver.num_fronts_snapshots.back()).end(),
            0) / std::get<2>(solver.num_fronts_snapshots.back()).size()
                << ")" << std::endl;
    }


    /**********************************************************************
     * Settings introduced by the migration to the finished NSGA-III
     * implementation of luishpmendes/pagmo2, branch nsga3-finish.
     **********************************************************************/
    std::cout << std::endl << "NSGA-III settings" << std::endl;

    /*  The ten argument constructor. Neither the seed nor the memory flag
     *  may end up on divisions_inner or random_mating, which is what the
     *  old eight argument call silently did against this pagmo.
     */
    pagmo::nsga3 algo(1, 0.95, 10.00, 0.01, 50.00, 2, 0, true, 2351389233u,
                      true);

    assert(algo.get_seed() == 2351389233u);

    std::string extra_info = algo.get_extra_info();

    assert(extra_info.find("Reference direction divisions: 2") !=
            std::string::npos);
    assert(extra_info.find("Reference direction inner divisions: 0") !=
            std::string::npos);
    assert(extra_info.find("Random mating: true") != std::string::npos);
    assert(extra_info.find("Inter-generational memory: true") !=
            std::string::npos);
    assert(extra_info.find("Seed: 2351389233") != std::string::npos);

    ifs.open("instances/X/X-n101-k25.vrp");
    assert(ifs.is_open());
    ifs >> instance;
    ifs.close();

    /*  Terminates on the iteration limit alone, leaving the time limit at
     *  its default, so that the wall clock cannot influence the result.
     */
    auto solve_and_collect = [&instance](unsigned divisions,
                                         unsigned divisions_inner,
                                         bool random_mating,
                                         unsigned population_size) {
        mocvrp::NSGA3_Solver solver(instance);

        solver.set_seed(2351389233);
        solver.iterations_limit = 20;
        solver.max_num_solutions = 128;
        solver.max_num_snapshots = 0;
        solver.population_size = population_size;
        solver.divisions = divisions;
        solver.divisions_inner = divisions_inner;
        solver.random_mating = random_mating;

        solver.solve();

        std::vector<std::vector<double>> values;

        for (const auto & solution : solver.best_solutions) {
            values.push_back(solution.value);
        }

        std::sort(values.begin(), values.end());

        return values;
    };

    // The same seed must give the same result, under either mating scheme
    for (bool random_mating : {true, false}) {
        std::vector<std::vector<double>> first =
            solve_and_collect(2, 0, random_mating, 32);
        std::vector<std::vector<double>> second =
            solve_and_collect(2, 0, random_mating, 32);

        assert(!first.empty());
        assert(first == second);

        std::cout << "Random mating " << random_mating << ": "
                  << first.size() << " solutions, reproducible" << std::endl;
    }

    // An inner layer finer than the outer one is rejected
    bool inner_layer_rejected = false;

    try {
        mocvrp::NSGA3_Solver rejecting_solver(instance);

        rejecting_solver.set_seed(2351389233);
        rejecting_solver.iterations_limit = 1;
        rejecting_solver.max_num_snapshots = 0;
        rejecting_solver.population_size = 32;
        rejecting_solver.divisions = 2;
        rejecting_solver.divisions_inner = 3;

        rejecting_solver.solve();
    } catch (const std::invalid_argument &) {
        inner_layer_rejected = true;
    }

    assert(inner_layer_rejected);

    /*  Five objectives with two outer divisions generate
     *  C(5 + 2 - 1, 2) = 15 reference directions, and an inner layer of a
     *  single division contributes C(5 + 1 - 1, 1) = 5 further ones, none
     *  of which coincides with an outer one here, for a total of 20. The
     *  finished implementation accepts a population of exactly that size.
     *  No outer layer alone can be matched exactly at five objectives: 5,
     *  15, 35, 70, 126 and 210 are none of them a multiple of four.
     */
    assert(pagmo::detail::generate_reference_directions(
                instance.num_objectives, 2, 0).size() == 15);
    assert(pagmo::detail::generate_reference_directions(
                instance.num_objectives, 2, 1).size() == 20);

    std::vector<std::vector<double>> equal_values =
        solve_and_collect(2, 1, true, 20);

    assert(!equal_values.empty());

    std::cout << "Population equal to the reference direction count: "
              << equal_values.size() << " solutions" << std::endl;

    /*  A second inner division contributes C(5 + 2 - 1, 2) = 15 directions
     *  instead of 5, for a total of 30, which a population of 32 holds.
     */
    assert(pagmo::detail::generate_reference_directions(
                instance.num_objectives, 2, 2).size() == 30);

    std::vector<std::vector<double>> two_layer_values =
        solve_and_collect(2, 2, true, 32);

    assert(!two_layer_values.empty());

    std::cout << "Two layer reference directions: "
              << two_layer_values.size() << " solutions" << std::endl;

    /*  A population which would have been large enough for the outer layer
     *  alone is rejected once the inner layer is added. Sixteen individuals
     *  satisfy both the minimum of 8 and the multiple of 4, and hold the 15
     *  directions of the outer layer, so it is the reference direction rule
     *  which rejects this, not the size rule.
     */
    bool combined_set_rejected = false;

    try {
        solve_and_collect(2, 1, true, 16);
    } catch (const std::invalid_argument &) {
        combined_set_rejected = true;
    }

    assert(combined_set_rejected);

    std::cout << std::endl << "NSGA3 Solver Test PASSED" << std::endl;

    return 0;
}
