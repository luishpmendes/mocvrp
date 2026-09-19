#include "instance/instance.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace mocvrp {

/*****************************************
 * The sections of a CVRPLIB instance file.
 *****************************************/
enum class Section {
    NONE,
    NODE_COORD,
    EDGE_WEIGHT,
    DEMAND,
    DEPOT
};

/********************************************************
 * Removes the leading and trailing whitespaces of a string.
 *
 * @param str the string to be trimmed.
 *
 * @return the trimmed string.
 ********************************************************/
static std::string trim(const std::string & str) {
    static const std::string whitespaces = " \t\n\v\f\r";
    std::string::size_type first = str.find_first_not_of(whitespaces);

    if (first == std::string::npos) {
        return std::string();
    }

    return str.substr(first, str.find_last_not_of(whitespaces) - first + 1);
}

/***************************************
 * Converts a string to upper case.
 *
 * @param str the string to be converted.
 *
 * @return the converted string.
 ***************************************/
static std::string upper(const std::string & str) {
    std::string result(str);

    std::transform(result.begin(), result.end(), result.begin(),
                   [](unsigned char c) { return std::toupper(c); });

    return result;
}

void Instance::init() {
    this->num_objectives = 5;
    this->num_customers = this->num_vertices > 0 ? this->num_vertices - 1 : 0;

    this->senses.resize(this->num_objectives, NSBRKGA::Sense::MINIMIZE);
    this->senses.assign(this->num_objectives, NSBRKGA::Sense::MINIMIZE);

    // The delivered orders and the route balance are maximized.
    this->senses[0] = NSBRKGA::Sense::MAXIMIZE;
    this->senses[3] = NSBRKGA::Sense::MAXIMIZE;

    this->orders.resize(this->num_vertices, 1);
    this->orders.assign(this->num_vertices, 1);

    if (this->num_vertices > 0) {
        this->orders[0] = 0;
    }

    // When the distances are not given explicitly, they are the Euclidean
    // distances rounded to the nearest integer, as TSPLIB95 prescribes.
    if (this->adj.empty()) {
        this->adj.resize(this->num_vertices,
                         std::vector<double>(this->num_vertices, 0.0));
        this->adj.assign(this->num_vertices,
                         std::vector<double>(this->num_vertices, 0.0));

        for (unsigned u = 0; u < this->num_vertices; u++) {
            for (unsigned v = u + 1; v < this->num_vertices; v++) {
                double dx = this->coord[u].first - this->coord[v].first,
                       dy = this->coord[u].second - this->coord[v].second,
                       dist = round(sqrt(dx * dx + dy * dy));

                this->adj[u][v] = dist;
                this->adj[v][u] = dist;
            }
        }
    }

    double max_dist = 0.0,
           max_customer_dist = 0.0;

    for (unsigned u = 0; u < this->num_vertices; u++) {
        for (unsigned v = u + 1; v < this->num_vertices; v++) {
            if (max_dist < this->adj[u][v]) {
                max_dist = this->adj[u][v];
            }

            // The route diameter does not take the depot into account.
            if (u > 0 && max_customer_dist < this->adj[u][v]) {
                max_customer_dist = this->adj[u][v];
            }
        }
    }

    this->primal_bound.resize(this->num_objectives, 0.0);
    this->primal_bound.assign(this->num_objectives, 0.0);

    // No order is delivered.
    this->primal_bound[0] = 0.0;
    // Every customer is served by a route of its own.
    this->primal_bound[1] = this->num_customers;
    // The largest distance between two customers.
    this->primal_bound[2] = max_customer_dist;
    // The route balance is greater than zero, but it gets arbitrarily close.
    this->primal_bound[3] = 0.0;
    // A solution traverses at most two arcs per customer.
    this->primal_bound[4] = 2.0 * this->num_customers * max_dist;
}

Instance::Instance(const std::vector<std::pair<double, double>> & coord,
                   const std::vector<unsigned> & demand,
                   const unsigned capacity) :
    num_objectives(5),
    num_vertices(coord.size()),
    num_customers(coord.size() > 0 ? coord.size() - 1 : 0),
    capacity(capacity),
    coord(coord),
    demand(demand),
    has_explicit_weights(false) {
    this->init();
}

Instance::Instance(const std::vector<std::vector<double>> & adj,
                   const std::vector<unsigned> & demand,
                   const unsigned capacity) :
    num_objectives(5),
    num_vertices(adj.size()),
    num_customers(adj.size() > 0 ? adj.size() - 1 : 0),
    capacity(capacity),
    coord(adj.size(), std::make_pair(0.0, 0.0)),
    adj(adj),
    demand(demand),
    has_explicit_weights(true) {
    this->init();
}

Instance::Instance() :
    num_objectives(5),
    num_vertices(0),
    num_customers(0),
    capacity(0),
    has_explicit_weights(false) {
    this->init();
}

bool Instance::is_valid() const {
    if (this->num_objectives != 5) {
        return false;
    }

    if (this->senses.size() != this->num_objectives) {
        return false;
    }

    if (this->primal_bound.size() != this->num_objectives) {
        return false;
    }

    for (const double & bound : this->primal_bound) {
        if (bound < 0.0) {
            return false;
        }
    }

    if (this->num_vertices < 2) {
        return false;
    }

    if (this->num_customers + 1 != this->num_vertices) {
        return false;
    }

    if (this->capacity == 0) {
        return false;
    }

    if (this->coord.size() != this->num_vertices) {
        return false;
    }

    if (this->adj.size() != this->num_vertices) {
        return false;
    }

    if (this->demand.size() != this->num_vertices) {
        return false;
    }

    if (this->orders.size() != this->num_vertices) {
        return false;
    }

    // The depot neither demands anything nor carries any order.
    if (this->demand[0] != 0) {
        return false;
    }

    if (this->orders[0] != 0) {
        return false;
    }

    // The distances are non negative and symmetric, and the diagonal is null.
    // The triangle inequality is not required: the explicit instances are real
    // road distances, and they violate it.
    for (unsigned u = 0; u < this->num_vertices; u++) {
        if (this->adj[u].size() != this->num_vertices) {
            return false;
        }

        if (this->adj[u][u] != 0.0) {
            return false;
        }

        for (unsigned v = u + 1; v < this->num_vertices; v++) {
            if (this->adj[u][v] < 0.0) {
                return false;
            }

            if (this->adj[u][v] != this->adj[v][u]) {
                return false;
            }
        }
    }

    for (unsigned u = 1; u < this->num_vertices; u++) {
        // No customer demands more than a vehicle can carry.
        if (this->demand[u] < 1 || this->demand[u] > this->capacity) {
            return false;
        }

        if (this->orders[u] < 1) {
            return false;
        }

        // No customer lies at the depot.
        if (this->adj[0][u] < 1.0) {
            return false;
        }
    }

    return true;
}

std::istream & operator >>(std::istream & is, Instance & instance) {
    std::string line,
                edge_weight_type,
                edge_weight_format;
    Section section = Section::NONE;
    bool has_dimension = false,
         has_depot = false;
    unsigned row = 1,
             col = 0;

    instance.name.clear();
    instance.comment.clear();
    instance.num_objectives = 5;
    instance.num_vertices = 0;
    instance.num_customers = 0;
    instance.capacity = 0;
    instance.coord.clear();
    instance.adj.clear();
    instance.demand.clear();
    instance.orders.clear();
    instance.has_explicit_weights = false;
    instance.senses.clear();
    instance.primal_bound.clear();

    while (std::getline(is, line)) {
        line = trim(line);

        if (line.empty()) {
            continue;
        }

        std::string::size_type pos = line.find(':');

        if (pos != std::string::npos) {
            std::string key = upper(trim(line.substr(0, pos))),
                        value = trim(line.substr(pos + 1));

            if (key == "NAME") {
                instance.name = value;
            } else if (key == "COMMENT") {
                instance.comment = value;
            } else if (key == "TYPE") {
                if (upper(value) != "CVRP") {
                    throw std::runtime_error("unsupported TYPE: " + value);
                }
            } else if (key == "DIMENSION") {
                instance.num_vertices = std::stoul(value);
                instance.num_customers = instance.num_vertices > 0 ?
                                         instance.num_vertices - 1 : 0;

                instance.coord.resize(instance.num_vertices,
                                      std::make_pair(0.0, 0.0));
                instance.coord.assign(instance.num_vertices,
                                      std::make_pair(0.0, 0.0));
                instance.demand.resize(instance.num_vertices, 0);
                instance.demand.assign(instance.num_vertices, 0);

                has_dimension = true;
            } else if (key == "CAPACITY") {
                instance.capacity = std::stoul(value);
            } else if (key == "EDGE_WEIGHT_TYPE") {
                edge_weight_type = upper(value);

                if (edge_weight_type != "EUC_2D" &&
                    edge_weight_type != "EXPLICIT") {
                    throw std::runtime_error("unsupported EDGE_WEIGHT_TYPE: " +
                                             value);
                }

                instance.has_explicit_weights = edge_weight_type == "EXPLICIT";
            } else if (key == "EDGE_WEIGHT_FORMAT") {
                edge_weight_format = upper(value);

                if (edge_weight_format != "LOWER_ROW") {
                    throw std::runtime_error("unsupported EDGE_WEIGHT_FORMAT: " +
                                             value);
                }
            } else if (key == "NODE_COORD_TYPE") {
                if (upper(value) != "TWOD_COORDS") {
                    throw std::runtime_error("unsupported NODE_COORD_TYPE: " +
                                             value);
                }
            }

            continue;
        }

        std::string keyword = upper(line);

        if (keyword == "EOF") {
            break;
        }

        if (keyword == "NODE_COORD_SECTION" ||
            keyword == "EDGE_WEIGHT_SECTION" ||
            keyword == "DEMAND_SECTION" ||
            keyword == "DEPOT_SECTION") {
            if (!has_dimension) {
                throw std::runtime_error("DIMENSION is missing before " +
                                         keyword);
            }

            if (keyword == "NODE_COORD_SECTION") {
                section = Section::NODE_COORD;
            } else if (keyword == "EDGE_WEIGHT_SECTION") {
                if (!instance.has_explicit_weights) {
                    throw std::runtime_error("EDGE_WEIGHT_SECTION on a "
                                             "non explicit instance");
                }

                instance.adj.resize(instance.num_vertices,
                                    std::vector<double>(instance.num_vertices,
                                                        0.0));
                instance.adj.assign(instance.num_vertices,
                                    std::vector<double>(instance.num_vertices,
                                                        0.0));

                section = Section::EDGE_WEIGHT;
            } else if (keyword == "DEMAND_SECTION") {
                section = Section::DEMAND;
            } else {
                section = Section::DEPOT;
            }

            continue;
        }

        std::istringstream iss(line);

        if (section == Section::NODE_COORD) {
            unsigned index;
            double x, y;

            while (iss >> index >> x >> y) {
                if (index < 1 || index > instance.num_vertices) {
                    throw std::runtime_error("vertex out of range in "
                                             "NODE_COORD_SECTION");
                }

                instance.coord[index - 1] = std::make_pair(x, y);
            }
        } else if (section == Section::EDGE_WEIGHT) {
            double weight;

            // The lower row format lists, below the diagonal, the row of each
            // vertex but the first one.
            while (iss >> weight) {
                if (row >= instance.num_vertices) {
                    throw std::runtime_error("too many weights in "
                                             "EDGE_WEIGHT_SECTION");
                }

                instance.adj[row][col] = weight;
                instance.adj[col][row] = weight;

                col++;

                if (col == row) {
                    row++;
                    col = 0;
                }
            }
        } else if (section == Section::DEMAND) {
            unsigned index, value;

            while (iss >> index >> value) {
                if (index < 1 || index > instance.num_vertices) {
                    throw std::runtime_error("vertex out of range in "
                                             "DEMAND_SECTION");
                }

                instance.demand[index - 1] = value;
            }
        } else if (section == Section::DEPOT) {
            long long value;

            while (iss >> value) {
                if (value < 0) {
                    section = Section::NONE;
                    break;
                }

                if (value != 1) {
                    throw std::runtime_error("the depot is expected to be the "
                                             "first vertex");
                }

                if (has_depot) {
                    throw std::runtime_error("more than one depot");
                }

                has_depot = true;
            }
        }
    }

    if (!has_dimension) {
        throw std::runtime_error("DIMENSION is missing");
    }

    if (!has_depot) {
        throw std::runtime_error("DEPOT_SECTION is missing");
    }

    if (instance.has_explicit_weights) {
        if (edge_weight_format.empty()) {
            throw std::runtime_error("EDGE_WEIGHT_FORMAT is missing");
        }

        if (row != instance.num_vertices || col != 0) {
            throw std::runtime_error("too few weights in EDGE_WEIGHT_SECTION");
        }
    }

    instance.init();

    return is;
}

std::ostream & operator <<(std::ostream & os, const Instance & instance) {
    std::streamsize precision = os.precision(17);

    os << "NAME : " << instance.name << std::endl
       << "COMMENT : " << instance.comment << std::endl
       << "TYPE : CVRP" << std::endl
       << "DIMENSION : " << instance.num_vertices << std::endl
       << "EDGE_WEIGHT_TYPE : "
       << (instance.has_explicit_weights ? "EXPLICIT" : "EUC_2D") << std::endl;

    if (instance.has_explicit_weights) {
        os << "EDGE_WEIGHT_FORMAT : LOWER_ROW" << std::endl
           << "NODE_COORD_TYPE : TWOD_COORDS" << std::endl;
    }

    os << "CAPACITY : " << instance.capacity << std::endl;

    if (instance.has_explicit_weights) {
        os << "EDGE_WEIGHT_SECTION" << std::endl;

        for (unsigned u = 1; u < instance.num_vertices; u++) {
            for (unsigned v = 0; v < u; v++) {
                if (v > 0) {
                    os << ' ';
                }

                os << instance.adj[u][v];
            }

            os << std::endl;
        }
    }

    os << "NODE_COORD_SECTION" << std::endl;

    for (unsigned u = 0; u < instance.num_vertices; u++) {
        os << u + 1 << ' ' << instance.coord[u].first << ' '
           << instance.coord[u].second << std::endl;
    }

    os << "DEMAND_SECTION" << std::endl;

    for (unsigned u = 0; u < instance.num_vertices; u++) {
        os << u + 1 << ' ' << instance.demand[u] << std::endl;
    }

    os << "DEPOT_SECTION" << std::endl
       << 1 << std::endl
       << -1 << std::endl
       << "EOF" << std::endl;

    os.precision(precision);

    return os;
}

}
