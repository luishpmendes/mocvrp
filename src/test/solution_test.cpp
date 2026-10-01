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

    // The decoded routes follow the random key encoding: the served customers
    // are those the second half of the key assigns to some route, every route
    // holds customers assigned to a single route in the visiting order of the
    // first half, the routes are sorted by assigned route, and an assigned
    // route is only split when the next customer would exceed the capacity.
    auto assert_encodes = [](const mocvrp::Instance & instance,
                             const std::vector<double> & key,
                             const mocvrp::Solution & solution) {
        auto route_of = [&](const unsigned customer) {
            return mocvrp::Solution::route_of_key(
                    key[instance.num_customers + customer - 1],
                    instance.max_num_routes);
        };
        std::vector<bool> is_served(instance.num_vertices, false);

        for (std::size_t r = 0; r < solution.routes.size(); r++) {
            const std::vector<unsigned> & route = solution.routes[r];

            assert(!route.empty());
            assert(solution.load[r] <= instance.capacity);

            for (std::size_t h = 0; h < route.size(); h++) {
                is_served[route[h]] = true;

                assert(route_of(route[h]) == route_of(route.front()));

                if (h > 0) {
                    assert(key[route[h - 1] - 1] <= key[route[h] - 1]);
                }
            }

            if (r > 0) {
                const std::vector<unsigned> & previous = solution.routes[r - 1];

                assert(route_of(previous.front()) <= route_of(route.front()));

                if (route_of(previous.front()) == route_of(route.front())) {
                    assert(key[previous.back() - 1] <= key[route.front() - 1]);
                    assert(solution.load[r - 1] +
                           instance.demand[route.front()] >
                               instance.capacity);
                }
            }
        }

        for (unsigned customer = 1;
             customer < instance.num_vertices;
             customer++) {
            assert(is_served[customer] == (route_of(customer) != 0));
        }
    };

    // The interval [0,1) is split into one subinterval per route plus one
    // for not serving, each closed on the left, and the upper bound 1.0 lies
    // in the last one. With three routes the boundaries are exact.
    {
        assert(mocvrp::Solution::route_of_key(0.0, 3) == 0);
        assert(mocvrp::Solution::route_of_key(std::nextafter(0.25, 0.0), 3) ==
                0);
        assert(mocvrp::Solution::route_of_key(0.25, 3) == 1);
        assert(mocvrp::Solution::route_of_key(std::nextafter(0.5, 0.0), 3) ==
                1);
        assert(mocvrp::Solution::route_of_key(0.5, 3) == 2);
        assert(mocvrp::Solution::route_of_key(std::nextafter(0.75, 0.0), 3) ==
                2);
        assert(mocvrp::Solution::route_of_key(0.75, 3) == 3);
        assert(mocvrp::Solution::route_of_key(std::nextafter(1.0, 0.0), 3) ==
                3);
        assert(mocvrp::Solution::route_of_key(1.0, 3) == 3);

        // The middle of every subinterval lies in it, whatever the number of
        // routes.
        for (unsigned num_routes = 1; num_routes <= 1000; num_routes++) {
            for (unsigned route = 0; route <= num_routes; route++) {
                assert(mocvrp::Solution::route_of_key(
                        (route + 0.5) / (num_routes + 1.0), num_routes) ==
                        route);
            }
        }
    }

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

    // The random key encoding on the levelling down instance: the second half
    // of the key assigns each customer to one of six routes, or to none, by
    // splitting [0,1) into seven equal subintervals, and the first half
    // defines the visiting order within each route.
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

        assert(instance.max_num_routes == 6);

        // Builds the key with the specified visiting order keys and assigned
        // routes, zero being not served, of the customers 1 to 6. Each route
        // key is the middle of the subinterval of its route.
        auto make_key = [&](const std::vector<double> & order,
                            const std::vector<unsigned> & assignment) {
            std::vector<double> key(order);

            for (const unsigned & route : assignment) {
                key.push_back((route + 0.5) / (instance.max_num_routes + 1.0));
            }

            return key;
        };

        // The key decodes into the specified routes, and into the value of
        // the solution made of them.
        auto assert_decodes = [&](
                const std::vector<double> & key,
                const std::vector<std::vector<unsigned>> & routes) {
            mocvrp::Solution solution(instance, key),
                             expected(instance, routes);

            assert(solution.is_feasible());
            assert_normalized(solution);
            assert_encodes(instance, key, solution);
            assert(solution.routes == routes);
            assert(solution.value == expected.value);

            return solution;
        };

        const std::vector<double> order = {0.2, 0.1, 0.3, 0.4, 0.5, 0.6};

        // The customers 1 to 3 share the second route and the customers 4 to
        // 6 share the fifth one, so the other four routes are left out. The
        // first half of the key orders each route.
        assert_decodes(make_key(order, {2, 2, 2, 5, 5, 5}),
                       {{2, 1, 3}, {4, 5, 6}});

        // The routes are sorted by assigned route, not by visiting order.
        assert_decodes(make_key(order, {5, 5, 5, 2, 2, 2}),
                       {{4, 5, 6}, {2, 1, 3}});

        // The customer 3 is not served.
        mocvrp::Solution partial = assert_decodes(
                make_key(order, {2, 2, 0, 5, 5, 5}),
                {{2, 1}, {4, 5, 6}});

        assert(fabs(partial.value[0] - 5.0 / 6.0) <
                std::numeric_limits<double>::epsilon());

        // Every customer has a route of its own, which attains the largest
        // number of routes.
        mocvrp::Solution separate = assert_decodes(
                make_key(order, {6, 5, 4, 3, 2, 1}),
                {{6}, {5}, {4}, {3}, {2}, {1}});

        assert(separate.value[0] == 1.0);
        assert(separate.value[1] == 1.0);

        // All six customers assigned to the third route weigh twice the
        // capacity, so that route is split in visiting order.
        assert_decodes(make_key({0.1, 0.4, 0.2, 0.5, 0.3, 0.6},
                                {3, 3, 3, 3, 3, 3}),
                       {{1, 3, 5}, {2, 4, 6}});

        // The overload of the first route is split off into a route of its
        // own, rather than into the second route, which would have room.
        assert_decodes(make_key(order, {1, 1, 1, 1, 2, 2}),
                       {{2, 1, 3}, {4}, {5, 6}});

        // The upper bound 1.0 assigns to the last route.
        std::vector<double> key(order);

        key.resize(2 * instance.num_customers, 1.0);

        assert_decodes(key, {{2, 1, 3}, {4, 5, 6}});

        // Nobody is served when every customer is assigned to no route.
        mocvrp::Solution empty = assert_decodes(
                make_key(order, {0, 0, 0, 0, 0, 0}), {});

        assert(empty.value == mocvrp::Solution(instance).value);
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

        // Decoded random keys.
        for (unsigned i = 0; i < 10; i++) {
            key.resize(2 * instance.num_customers);

            for (double & k : key) {
                k = distribution(rng);
            }

            mocvrp::Solution solution(instance, key);

            assert(solution.is_feasible());
            assert_normalized(solution);
            assert_encodes(instance, key, solution);
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

        // Decoded random keys that assign every customer to one of the first
        // few routes, or to none, so that those routes are overloaded and
        // must be split.
        for (unsigned num_routes = 1; num_routes <= 10; num_routes++) {
            std::uniform_int_distribution<unsigned> route_distribution(
                    0, num_routes);

            key.resize(2 * instance.num_customers);

            for (unsigned i = 0; i < instance.num_customers; i++) {
                key[i] = distribution(rng);
                key[instance.num_customers + i] =
                    (route_distribution(rng) + 0.5) /
                    (instance.max_num_routes + 1.0);
            }

            mocvrp::Solution solution(instance, key);

            assert(solution.is_feasible());
            assert_normalized(solution);
            assert_encodes(instance, key, solution);
        }
    }

    std::cout << std::endl << "Solution Test PASSED" << std::endl;

    return 0;
}
