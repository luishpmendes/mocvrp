#include "instance/instance.hpp"
#include <cassert>
#include <fstream>
#include <iostream>

int main() {
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
        assert(instance.primal_bound[1] == instance.num_customers);
        assert(instance.is_valid());
    }

    std::cout << std::endl << "Instance Test PASSED" << std::endl;

    return 0;
}
