#include "solution/solution.hpp"
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <numeric>
#include <random>
#include <sstream>

int main() {
    // Every objective of every solution is normalized to [0,1].
    auto assert_normalized = [](const mocvrp::Solution & solution) {
        for (const double & v : solution.value) {
            assert(v >= 0.0 && v <= 1.0);
        }
    };

    // The levelling down example of the formulation document: perfect balance
    // is achieved by driving two extra kilometres for no reason.
    {
        std::vector<std::pair<double, double>> coord = {{0.0, 0.0},
                                                       {1.0, 0.0},
                                                       {2.0, 0.0},
                                                       {3.0, 0.0},
                                                       {0.0, 2.0},
                                                       {0.0, 3.0},
                                                       {0.0, 4.0}};
        std::vector<unsigned> demand = {0, 5, 5, 5, 5, 5, 5};
        mocvrp::Instance instance(coord, demand, 15);

        assert(instance.is_valid());
        assert(instance.num_vertices == 7);
        assert(instance.num_customers == 6);

        // The customers 3 and 6 are the farthest apart, and no arc is longer
        // than theirs, so the total distance is bounded by 2 * 6 * 5.
        assert(instance.total_orders == 6);
        assert(instance.max_num_routes == 6);
        assert(instance.diameter == 5.0);
        assert(instance.max_total_distance == 60.0);

        std::vector<std::vector<unsigned>> routes = {{1, 2, 3}, {4, 5, 6}},
                                           levelled_routes = {{2, 1, 3},
                                                              {4, 5, 6}},
                                           partial_routes = {{1, 2, 3}};
        mocvrp::Solution solution(instance, routes);
        mocvrp::Solution levelled(instance, levelled_routes);

        assert(solution.is_feasible());
        assert(levelled.is_feasible());
        assert_normalized(solution);
        assert_normalized(levelled);

        assert(solution.length[0] == 6.0);
        assert(solution.length[1] == 8.0);
        assert(levelled.length[0] == 8.0);
        assert(levelled.length[1] == 8.0);

        assert(solution.value[0] == 1.0);
        assert(fabs(solution.value[1] - 2.0 / 6.0) <
                std::numeric_limits<double>::epsilon());
        assert(fabs(solution.value[2] - 2.0 / 5.0) <
                std::numeric_limits<double>::epsilon());
        assert(fabs(solution.value[3] - 0.75) <
                std::numeric_limits<double>::epsilon());
        assert(fabs(solution.value[4] - 14.0 / 60.0) <
                std::numeric_limits<double>::epsilon());

        assert(levelled.value[0] == 1.0);
        assert(fabs(levelled.value[1] - 2.0 / 6.0) <
                std::numeric_limits<double>::epsilon());
        assert(fabs(levelled.value[2] - 2.0 / 5.0) <
                std::numeric_limits<double>::epsilon());
        assert(fabs(levelled.value[3] - 1.0) <
                std::numeric_limits<double>::epsilon());
        assert(fabs(levelled.value[4] - 16.0 / 60.0) <
                std::numeric_limits<double>::epsilon());

        // Over the five objectives the two solutions are incomparable: the
        // levelled one is better balanced but travels further.
        assert(!solution.dominates(levelled));
        assert(!levelled.dominates(solution));

        // Serving one customer less is dominated: it delivers fewer orders
        // and is not better anywhere else.
        mocvrp::Solution partial(instance, partial_routes);

        assert(partial.is_feasible());
        assert_normalized(partial);
        assert(fabs(partial.value[0] - 3.0 / 6.0) <
                std::numeric_limits<double>::epsilon());
        assert(fabs(partial.value[1] - 1.0 / 6.0) <
                std::numeric_limits<double>::epsilon());
        assert(fabs(partial.value[4] - 6.0 / 60.0) <
                std::numeric_limits<double>::epsilon());
        assert(!partial.dominates(solution));

        // A single route is perfectly balanced by definition.
        assert(fabs(partial.value[3] - 1.0) <
                std::numeric_limits<double>::epsilon());

        // The same customers in a worse visiting order are dominated: only
        // the total travelled distance changes, and it grows.
        std::vector<std::vector<unsigned>> detour_routes = {{2, 1, 3}};
        mocvrp::Solution detour(instance, detour_routes);

        assert(detour.is_feasible());
        assert_normalized(detour);
        assert(detour.value[0] == partial.value[0]);
        assert(detour.value[1] == partial.value[1]);
        assert(detour.value[2] == partial.value[2]);
        assert(detour.value[3] == partial.value[3]);
        assert(fabs(detour.value[4] - 8.0 / 60.0) <
                std::numeric_limits<double>::epsilon());
        assert(partial.dominates(detour));
        assert(!detour.dominates(partial));
    }

    // Two customers at the same location: the instance diameter is zero, and
    // so is every route diameter, which is normalized to zero instead of
    // being divided by zero.
    {
        std::vector<std::pair<double, double>> coord = {{0.0, 0.0},
                                                       {1.0, 0.0},
                                                       {1.0, 0.0}};
        std::vector<unsigned> demand = {0, 1, 1};
        mocvrp::Instance instance(coord, demand, 2);

        assert(instance.is_valid());
        assert(instance.diameter == 0.0);
        assert(instance.max_total_distance == 4.0);
        assert(instance.primal_bound[2] == 0.0);

        std::vector<std::vector<unsigned>> shared_routes = {{1, 2}},
                                           separate_routes = {{1}, {2}};
        mocvrp::Solution shared(instance, shared_routes);
        mocvrp::Solution separate(instance, separate_routes);

        assert(shared.is_feasible());
        assert(separate.is_feasible());
        assert_normalized(shared);
        assert_normalized(separate);

        // A single route of length 1 + 0 + 1.
        assert(shared.value[0] == 1.0);
        assert(fabs(shared.value[1] - 0.5) <
                std::numeric_limits<double>::epsilon());
        assert(shared.value[2] == 0.0);
        assert(shared.value[3] == 1.0);
        assert(fabs(shared.value[4] - 0.5) <
                std::numeric_limits<double>::epsilon());

        // A route per customer attains the bounds on the number of routes and
        // on the total distance.
        assert(separate.value[0] == 1.0);
        assert(separate.value[1] == 1.0);
        assert(separate.value[2] == 0.0);
        assert(separate.value[3] == 1.0);
        assert(separate.value[4] == 1.0);

        assert(shared.dominates(separate));
    }

    std::ifstream ifs;
    mocvrp::Instance instance;
    std::vector<double> key;
    std::mt19937 rng(2351389233);
    std::uniform_real_distribution<double> distribution(0.0, 1.0);

    for (const std::string filename : {"instances/X/X-n101-k25",
                                       "instances/X/X-n1001-k43",
                                       "instances/DIMACS/ORTEC-n242-k12",
                                       "instances/DIMACS/Loggi-n401-k23",
                                       "instances/AGS/Leuven1"}) {
        std::cout << filename << std::endl;

        ifs.open(filename + ".vrp");

        assert(ifs.is_open());

        ifs >> instance;

        ifs.close();

        // The empty solution serves nobody and is perfectly balanced.
        {
            mocvrp::Solution solution(instance);

            assert(solution.is_feasible());
            assert_normalized(solution);
            assert(solution.routes.empty());
            assert(solution.value[0] == 0.0);
            assert(solution.value[1] == 0.0);
            assert(solution.value[2] == 0.0);
            assert(solution.value[3] == 1.0);
            assert(solution.value[4] == 0.0);
        }

        // The best known solution shipped with the instance: its cost must
        // match the total travelled distance computed here.
        {
            mocvrp::Solution solution(instance);
            std::ifstream sol_ifs(filename + ".sol");

            assert(sol_ifs.is_open());

            sol_ifs >> solution;

            sol_ifs.close();

            double cost = 0.0;
            std::ifstream cost_ifs(filename + ".sol");
            std::string line;

            while (std::getline(cost_ifs, line)) {
                if (line.rfind("Cost", 0) == 0) {
                    cost = std::stod(line.substr(4));
                }
            }

            cost_ifs.close();

            assert(solution.is_feasible());
            assert_normalized(solution);
            // The best known solutions serve every customer.
            assert(solution.value[0] == 1.0);
            assert(solution.value[1] ==
                    double(solution.routes.size()) / instance.max_num_routes);
            assert(std::accumulate(solution.length.begin(),
                                   solution.length.end(),
                                   0.0) == cost);
            assert(solution.value[4] == cost / instance.max_total_distance);

            // The solution is written with its raw cost, not the normalized
            // one, as CVRPLIB prescribes.
            std::ostringstream oss;

            oss << solution;

            std::istringstream written(oss.str());
            double written_cost = -1.0;

            while (std::getline(written, line)) {
                if (line.rfind("Cost", 0) == 0) {
                    written_cost = std::stod(line.substr(4));
                }
            }

            assert(written_cost == cost);
        }

        // A decoded random key.
        {
            key.resize(2 * instance.num_customers);

            for (double & k : key) {
                k = distribution(rng);
            }

            mocvrp::Solution solution(instance, key);

            assert(solution.is_feasible());
            assert_normalized(solution);
            assert(solution.value[0] > 0.0);
            assert(solution.value[0] <= 1.0);
            assert(solution.value[1] > 0.0);
            assert(solution.value[1] <= instance.primal_bound[1]);
            assert(solution.value[2] >= 0.0);
            assert(solution.value[2] <= instance.primal_bound[2]);
            assert(solution.value[3] > 0.0);
            assert(solution.value[3] <= 1.0);
            assert(solution.value[4] > 0.0);
            assert(solution.value[4] <= instance.primal_bound[4]);

            // Serving strictly more orders with no other loss dominates.
            mocvrp::Solution empty_solution(instance);

            assert(!empty_solution.dominates(solution));
        }
    }

    std::cout << std::endl << "Solution Test PASSED" << std::endl;

    return 0;
}
