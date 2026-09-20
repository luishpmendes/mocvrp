#include "solution/solution.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace mocvrp {

bool Solution::dominates(const std::vector<double> & valueA,
                         const std::vector<double> & valueB,
                         const std::vector<NSBRKGA::Sense> & senses) {
    if (valueA.size() != valueB.size() || valueA.size() != senses.size()) {
        return false;
    }

    bool at_least_as_good = true, better = false;

    for (std::size_t i = 0; i < valueA.size() && at_least_as_good; i++) {
        if (senses[i] == NSBRKGA::Sense::MINIMIZE) {
            if (valueA[i] > valueB[i] + std::numeric_limits<double>::epsilon()) {
                at_least_as_good = false;
            } else if (valueA[i] <
                       valueB[i] - std::numeric_limits<double>::epsilon()) {
                better = true;
            }
        } else {
            if (valueA[i] < valueB[i] - std::numeric_limits<double>::epsilon()) {
                at_least_as_good = false;
            } else if (valueA[i] >
                       valueB[i] + std::numeric_limits<double>::epsilon()) {
                better = true;
            }
        }
    }

    return at_least_as_good && better;
}

void Solution::compute_value() {
    this->value.resize(this->instance.num_objectives, 0.0);
    this->value.assign(this->instance.num_objectives, 0.0);

    // The empty solution delivers nothing, uses no route and travels no
    // distance, and it is perfectly balanced by convention.
    if (this->routes.empty()) {
        this->value[3] = 1.0;
        return;
    }

    double total_orders = 0.0,
           max_diameter = 0.0,
           total_length = 0.0,
           min_length = this->length.front(),
           max_length = this->length.front(),
           min_load = this->load.front(),
           max_load = this->load.front(),
           min_orders = this->num_orders.front(),
           max_orders = this->num_orders.front();

    for (std::size_t r = 0; r < this->routes.size(); r++) {
        total_orders += this->num_orders[r];
        total_length += this->length[r];

        if (max_diameter < this->diameter[r]) {
            max_diameter = this->diameter[r];
        }

        if (min_length > this->length[r]) {
            min_length = this->length[r];
        }

        if (max_length < this->length[r]) {
            max_length = this->length[r];
        }

        if (min_load > this->load[r]) {
            min_load = this->load[r];
        }

        if (max_load < this->load[r]) {
            max_load = this->load[r];
        }

        if (min_orders > this->num_orders[r]) {
            min_orders = this->num_orders[r];
        }

        if (max_orders < this->num_orders[r]) {
            max_orders = this->num_orders[r];
        }
    }

    // The denominators are positive for every non empty solution.
    this->value[0] = total_orders;
    this->value[1] = this->routes.size();
    this->value[2] = max_diameter;
    this->value[3] = std::min({min_length / max_length,
                               min_load / max_load,
                               min_orders / max_orders});
    this->value[4] = total_length;
}

void Solution::init() {
    this->length.resize(this->routes.size(), 0.0);
    this->length.assign(this->routes.size(), 0.0);
    this->load.resize(this->routes.size(), 0.0);
    this->load.assign(this->routes.size(), 0.0);
    this->num_orders.resize(this->routes.size(), 0.0);
    this->num_orders.assign(this->routes.size(), 0.0);
    this->diameter.resize(this->routes.size(), 0.0);
    this->diameter.assign(this->routes.size(), 0.0);

    for (std::size_t r = 0; r < this->routes.size(); r++) {
        const std::vector<unsigned> & route = this->routes[r];

        if (route.empty()) {
            continue;
        }

        // The route leaves the depot, visits its customers and returns.
        this->length[r] = this->instance.adj[0][route.front()] +
                          this->instance.adj[route.back()][0];

        for (std::size_t h = 0; h + 1 < route.size(); h++) {
            this->length[r] += this->instance.adj[route[h]][route[h + 1]];
        }

        for (const unsigned & customer : route) {
            this->load[r] += this->instance.demand[customer];
            this->num_orders[r] += this->instance.orders[customer];
        }

        // The diameter does not take the depot into account, so a route with
        // a single customer has diameter zero.
        for (std::size_t h = 0; h + 1 < route.size(); h++) {
            for (std::size_t k = h + 1; k < route.size(); k++) {
                if (this->diameter[r] <
                        this->instance.adj[route[h]][route[k]]) {
                    this->diameter[r] = this->instance.adj[route[h]][route[k]];
                }
            }
        }
    }

    this->compute_value();
}

Solution::Solution(const Instance & instance,
                   const std::vector<std::vector<unsigned>> & routes) :
        Solution(instance) {
    this->routes = routes;
    this->init();
}

Solution::Solution(const Instance & instance,
                   const std::vector<double> & key) : Solution(instance) {
    std::vector<std::pair<double, unsigned>> permutation;

    permutation.reserve(this->instance.num_customers);

    // The first half of the key defines the visiting order, and the second
    // half defines which customers are served.
    for (unsigned customer = 1;
         customer < this->instance.num_vertices;
         customer++) {
        if (key[this->instance.num_customers + customer - 1] >= 0.5) {
            permutation.push_back(std::make_pair(key[customer - 1], customer));
        }
    }

    std::sort(permutation.begin(), permutation.end());

    double current_load = 0.0;

    // Sweeps the served customers, closing the current route whenever the
    // next customer would exceed the vehicles capacity.
    for (const auto & [k, customer] : permutation) {
        if (this->routes.empty() ||
            current_load + this->instance.demand[customer] >
                this->instance.capacity) {
            this->routes.push_back(std::vector<unsigned>());
            current_load = 0.0;
        }

        this->routes.back().push_back(customer);
        current_load += this->instance.demand[customer];
    }

    this->init();
}

Solution::Solution(const Instance & instance) :
        instance(instance),
        routes(),
        length(),
        load(),
        num_orders(),
        diameter(),
        value() {
    this->compute_value();
}

bool Solution::is_feasible() const {
    if (!this->instance.is_valid()) {
        return false;
    }

    if (this->length.size() != this->routes.size()) {
        return false;
    }

    if (this->load.size() != this->routes.size()) {
        return false;
    }

    if (this->num_orders.size() != this->routes.size()) {
        return false;
    }

    if (this->diameter.size() != this->routes.size()) {
        return false;
    }

    if (this->value.size() != this->instance.num_objectives) {
        return false;
    }

    // A solution uses at most one route per customer.
    if (this->routes.size() > this->instance.num_customers) {
        return false;
    }

    std::vector<bool> is_served(this->instance.num_vertices, false);

    for (const std::vector<unsigned> & route : this->routes) {
        // An empty route is not a route but the absence of one.
        if (route.empty()) {
            return false;
        }

        for (const unsigned & customer : route) {
            if (customer < 1 || customer >= this->instance.num_vertices) {
                return false;
            }

            // No customer is visited twice. The customers are not required
            // to be covered, though: the service is optional.
            if (is_served[customer]) {
                return false;
            }

            is_served[customer] = true;
        }
    }

    for (std::size_t r = 0; r < this->routes.size(); r++) {
        if (this->load[r] > this->instance.capacity) {
            return false;
        }
    }

    // The cached attributes must agree with a fresh computation.
    Solution solution(this->instance, this->routes);

    for (std::size_t r = 0; r < this->routes.size(); r++) {
        if (fabs(this->length[r] - solution.length[r]) >
                std::numeric_limits<double>::epsilon()) {
            return false;
        }

        if (fabs(this->load[r] - solution.load[r]) >
                std::numeric_limits<double>::epsilon()) {
            return false;
        }

        if (fabs(this->num_orders[r] - solution.num_orders[r]) >
                std::numeric_limits<double>::epsilon()) {
            return false;
        }

        if (fabs(this->diameter[r] - solution.diameter[r]) >
                std::numeric_limits<double>::epsilon()) {
            return false;
        }
    }

    for (unsigned i = 0; i < this->instance.num_objectives; i++) {
        if (fabs(this->value[i] - solution.value[i]) >
                std::numeric_limits<double>::epsilon()) {
            return false;
        }

        // The primal bound holds the worst value of each objective, in the
        // sense of that objective.
        if (this->instance.senses[i] == NSBRKGA::Sense::MINIMIZE) {
            if (this->value[i] > this->instance.primal_bound[i]) {
                return false;
            }
        } else {
            if (this->value[i] < this->instance.primal_bound[i]) {
                return false;
            }
        }
    }

    return true;
}

bool Solution::dominates(const Solution & solution) const {
    return Solution::dominates(this->value,
                               solution.value,
                               this->instance.senses);
}

std::istream & operator >>(std::istream & is, Solution & solution) {
    std::string line;

    solution.routes.clear();

    while (std::getline(is, line)) {
        std::string::size_type pos = line.find(':');

        if (pos == std::string::npos) {
            // The cost line closes the solution: the value is recomputed.
            break;
        }

        std::istringstream iss(line.substr(pos + 1));
        std::vector<unsigned> route;
        unsigned customer;

        while (iss >> customer) {
            route.push_back(customer);
        }

        solution.routes.push_back(route);
    }

    solution.init();

    return is;
}

std::ostream & operator <<(std::ostream & os, const Solution & solution) {
    std::streamsize precision = os.precision(17);

    for (std::size_t r = 0; r < solution.routes.size(); r++) {
        os << "Route #" << r + 1 << ':';

        for (const unsigned & customer : solution.routes[r]) {
            os << ' ' << customer;
        }

        os << std::endl;
    }

    os << "Cost " << solution.value[4] << std::endl;

    os.precision(precision);

    return os;
}

}
