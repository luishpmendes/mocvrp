#include "solver/nsbrkga/decoder.hpp"
#include <algorithm>

namespace mocvrp {

Decoder::Decoder(const Instance & instance,
                 unsigned num_threads)
    : instance(instance),
      permutation_of_thread(num_threads),
      route_begin_of_thread(num_threads),
      value_of_thread(num_threads,
                      std::vector<double>(instance.num_objectives)) {
    for (unsigned thread = 0; thread < num_threads; thread++) {
        this->permutation_of_thread[thread].reserve(
                this->instance.num_customers);
        this->route_begin_of_thread[thread].reserve(
                this->instance.num_customers);
    }
}

std::vector<double> Decoder::decode(NSBRKGA::Chromosome & chromosome,
                                    bool /* not used */) {
#   ifdef _OPENMP
        std::vector<std::tuple<unsigned, double, unsigned>> & permutation =
            this->permutation_of_thread[omp_get_thread_num()];
        std::vector<unsigned> & route_begin =
            this->route_begin_of_thread[omp_get_thread_num()];
        std::vector<double> & value =
            this->value_of_thread[omp_get_thread_num()];
#   else
        std::vector<std::tuple<unsigned, double, unsigned>> & permutation =
            this->permutation_of_thread.front();
        std::vector<unsigned> & route_begin =
            this->route_begin_of_thread.front();
        std::vector<double> & value = this->value_of_thread.front();
#   endif

    // The routes are those of the Solution random key constructor, as
    // contiguous segments of the sorted permutation.
    Solution::decode_key(this->instance, chromosome, permutation, route_begin);

    value.assign(this->instance.num_objectives, 0.0);

    // The empty solution delivers nothing, uses no route and travels no
    // distance, and it is perfectly balanced by convention.
    if (route_begin.empty()) {
        value[3] = 1.0;
        return value;
    }

    double total_orders = 0.0,
           max_diameter = 0.0,
           total_length = 0.0,
           min_length = 0.0,
           max_length = 0.0,
           min_load = 0.0,
           max_load = 0.0,
           min_orders = 0.0,
           max_orders = 0.0;

    for (std::size_t r = 0; r < route_begin.size(); r++) {
        const unsigned begin = route_begin[r],
                       end = r + 1 < route_begin.size() ?
                             route_begin[r + 1] :
                             unsigned(permutation.size()),
                       first = std::get<2>(permutation[begin]),
                       last = std::get<2>(permutation[end - 1]);

        // The route leaves the depot, visits its customers and returns.
        double length = this->instance.adj[0][first] +
                        this->instance.adj[last][0],
               load = 0.0,
               num_orders = 0.0,
               diameter = 0.0;

        for (unsigned h = begin; h + 1 < end; h++) {
            length += this->instance.adj[std::get<2>(permutation[h])]
                                        [std::get<2>(permutation[h + 1])];
        }

        for (unsigned h = begin; h < end; h++) {
            const unsigned customer = std::get<2>(permutation[h]);

            load += this->instance.demand[customer];
            num_orders += this->instance.orders[customer];
        }

        // The diameter does not take the depot into account, so a route with
        // a single customer has diameter zero.
        for (unsigned h = begin; h + 1 < end; h++) {
            const unsigned u = std::get<2>(permutation[h]);

            for (unsigned k = h + 1; k < end; k++) {
                const unsigned v = std::get<2>(permutation[k]);

                if (diameter < this->instance.adj[u][v]) {
                    diameter = this->instance.adj[u][v];
                }
            }
        }

        if (r == 0) {
            min_length = max_length = length;
            min_load = max_load = load;
            min_orders = max_orders = num_orders;
        }

        total_orders += num_orders;
        total_length += length;

        if (max_diameter < diameter) {
            max_diameter = diameter;
        }

        if (min_length > length) {
            min_length = length;
        }

        if (max_length < length) {
            max_length = length;
        }

        if (min_load > load) {
            min_load = load;
        }

        if (max_load < load) {
            max_load = load;
        }

        if (min_orders > num_orders) {
            min_orders = num_orders;
        }

        if (max_orders < num_orders) {
            max_orders = num_orders;
        }
    }

    // The denominators are positive for every non empty solution.
    value[0] = total_orders;
    value[1] = route_begin.size();
    value[2] = max_diameter;
    value[3] = std::min({min_length / max_length,
                         min_load / max_load,
                         min_orders / max_orders});
    value[4] = total_length;

    this->instance.normalize(value);

    return value;
}

}
