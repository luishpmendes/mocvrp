#include "instance/instance.hpp"
#include <algorithm>
#include <cassert>
#include <fstream>
#include <iostream>

int main() {
    // The empty instance has no bound at all: its primal bound is zero, and
    // it is not left undefined by a division by zero.
    {
        mocvrp::Instance instance;

        assert(instance.total_orders == 0);
        assert(instance.max_num_routes == 0);
        assert(instance.diameter == 0.0);
        assert(instance.max_total_distance == 0.0);
        assert(instance.primal_bound.size() == instance.num_objectives);

        for (const double & bound : instance.primal_bound) {
            assert(bound == 0.0);
        }
    }

    std::ifstream ifs;
    mocvrp::Instance instance;

    for (const std::string filename : {"instances/X/X-n101-k25.vrp",
                                       "instances/X/X-n214-k11.vrp",
                                       "instances/X/X-n331-k15.vrp",
                                       "instances/X/X-n573-k30.vrp",
                                       "instances/X/X-n1001-k43.vrp",
                                       "instances/DIMACS/ORTEC-n242-k12.vrp",
                                       "instances/DIMACS/ORTEC-n455-k41.vrp",
                                       "instances/DIMACS/ORTEC-n701-k64.vrp",
                                       "instances/DIMACS/Loggi-n401-k23.vrp",
                                       "instances/DIMACS/Loggi-n601-k19.vrp",
                                       "instances/DIMACS/Loggi-n1001-k31.vrp",
                                       "instances/XL/XL-n1048-k237.vrp",
                                       "instances/XL/XL-n1701-k562.vrp",
                                       "instances/XL/XL-n2634-k17.vrp",
                                       "instances/AGS/Leuven1.vrp",
                                       "instances/AGS/Leuven2.vrp"}) {
        std::cout << filename << std::endl;

        ifs.open(filename);

        assert(ifs.is_open());

        ifs >> instance;

        ifs.close();

        assert(instance.num_objectives == 5);
        assert(instance.num_vertices >= 101);
        assert(instance.num_vertices <= 4001);
        assert(instance.num_customers + 1 == instance.num_vertices);
        assert(instance.capacity > 0);
        assert(instance.demand[0] == 0);
        assert(instance.orders[0] == 0);
        assert(instance.senses[0] == NSBRKGA::Sense::MAXIMIZE);
        assert(instance.senses[1] == NSBRKGA::Sense::MINIMIZE);
        assert(instance.senses[2] == NSBRKGA::Sense::MINIMIZE);
        assert(instance.senses[3] == NSBRKGA::Sense::MAXIMIZE);
        assert(instance.senses[4] == NSBRKGA::Sense::MINIMIZE);

        double max_dist = 0.0,
               max_customer_dist = 0.0;

        for (unsigned u = 0; u < instance.num_vertices; u++) {
            for (unsigned v = u + 1; v < instance.num_vertices; v++) {
                max_dist = std::max(max_dist, instance.adj[u][v]);

                if (u > 0) {
                    max_customer_dist = std::max(max_customer_dist,
                                                 instance.adj[u][v]);
                }
            }
        }

        // Every customer carries a single order by default.
        assert(instance.total_orders == instance.num_customers);
        assert(instance.max_num_routes == instance.num_customers);
        assert(instance.diameter == max_customer_dist);
        assert(instance.diameter > 0.0);
        assert(instance.max_total_distance ==
                2.0 * instance.num_customers * max_dist);

        // The primal bound is normalized: the worst value of each objective
        // is either the bottom or the top of [0,1].
        assert(instance.primal_bound[0] == 0.0);
        assert(instance.primal_bound[1] == 1.0);
        assert(instance.primal_bound[2] == 1.0);
        assert(instance.primal_bound[3] == 0.0);
        assert(instance.primal_bound[4] == 1.0);
        assert(instance.is_valid());
    }

    std::cout << std::endl << "Instance Test PASSED" << std::endl;

    return 0;
}
