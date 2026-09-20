#pragma once

#include "instance/instance.hpp"
#include <pagmo/types.hpp>

namespace mocvrp {
/***********************************************************************
 * The Problem class represents the Multi-Objective Capacitated Vehicle
 * Routing Problem with Optional Service as a pagmo problem.
 *
 * The decision vector has two keys per customer: the first half defines
 * the visiting order and the second half defines which customers are
 * served, exactly as the Solution random key constructor prescribes.
 ***********************************************************************/
class Problem {
    public:
    /****************************
     * The instance been solved.
     ****************************/
    const Instance * instance;

    /***********************************************
     * Constructs a new problem.
     *
     * @param instance the instance been solved.
     ***********************************************/
    Problem(const Instance * instance);

    /*******************************
     * Constructs an empty problem.
     *******************************/
    Problem();

    /********************************************************************
     * Returns the value of the solution the specified decision vector
     * represents, with the maximized objectives negated, since every
     * objective is minimized inside pagmo.
     *
     * @param dv the decision vector to be decoded.
     *
     * @return the value of the solution the decision vector represents.
     ********************************************************************/
    pagmo::vector_double fitness(const pagmo::vector_double & dv) const;

    /*****************************************************
     * Returns the bounds of the decision vector.
     *
     * @return the lower and the upper bounds.
     *****************************************************/
    std::pair<pagmo::vector_double, pagmo::vector_double> get_bounds() const;

    /****************************************
     * Returns the number of objectives.
     *
     * @return the number of objectives.
     ****************************************/
    pagmo::vector_double::size_type get_nobj() const;
};

}
