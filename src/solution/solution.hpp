#pragma once

#include "instance/instance.hpp"

namespace mocvrp {
/*********************************************************
 * The Solution class represents a solution for the
 * Multi-Objective Capacitated Vehicle Routing Problem
 * with Optional Service.
 *********************************************************/
class Solution {
    public:
    /*************************************************************
     * Returns true if valueA dominates valueB; false otherwise.
     *
     * @param valueA the first value been compared.
     * @param valueB the second value been compared.
     * @param senses the optimization senses.
     *
     * @return true if valueA dominates valueB; false otherwise.
     *************************************************************/
    static bool dominates(const std::vector<double> & valueA,
                          const std::vector<double> & valueB,
                          const std::vector<NSBRKGA::Sense> & senses);

    /*************************************************************
     * Negates, in place, the entries of the specified value whose
     * sense is to maximize, converting between the optimization
     * senses of the instance and the convention of minimizing
     * every objective.
     *
     * This conversion is its own inverse, so the same call takes
     * a value in either direction.
     *
     * @param value  the value to be converted.
     * @param senses the optimization senses.
     *************************************************************/
    static void negate_maximized(std::vector<double> & value,
                                 const std::vector<NSBRKGA::Sense> & senses);

    /****************************
     * The instance been solved.
     ****************************/
    const Instance & instance;

    /***********************************************************
     * The customers visited by each route, in order.
     * The depot is implicit at the start and at the end.
     ***********************************************************/
    std::vector<std::vector<unsigned>> routes;

    /**************************
     * The length of each route.
     **************************/
    std::vector<double> length;

    /************************
     * The load of each route.
     ************************/
    std::vector<double> load;

    /********************************
     * The orders of each route.
     ********************************/
    std::vector<double> num_orders;

    /*****************************
     * The diameter of each route.
     *****************************/
    std::vector<double> diameter;

    /******************************************************************
     * The value of the solution, normalized to [0,1], that consists of:
     * - the number of delivered orders over the total number of
     *   orders, to be maximized
     * - the number of routes over the largest number of routes, to
     *   be minimized
     * - the largest route diameter over the instance diameter, to be
     *   minimized
     * - the route balance, to be maximized
     * - the total travelled distance over its upper bound, to be
     *   minimized
     ******************************************************************/
    std::vector<double> value;

    private:
    /**************************************
     * Computes the value of the solution.
     **************************************/
    void compute_value();

    /******************************
     * Initializes a new solution.
     ******************************/
    void init();

    public:
    /**********************************************************
     * Constructs a new solution.
     *
     * @param instance the instance been solved.
     * @param routes   the customers visited by each route.
     **********************************************************/
    Solution(const Instance & instance,
             const std::vector<std::vector<unsigned>> & routes);

    /*********************************************************
     * Constructs a new solution.
     *
     * @param instance the instance been solved.
     * @param key      the key representing the customers
     *                 been served and their visiting order.
     *********************************************************/
    Solution(const Instance & instance,
             const std::vector<double> & key);

    /********************************************
     * Constructs an empty solution.
     *
     * @param instance the instance been solved.
     ********************************************/
    Solution(const Instance & instance);

    /*********************************************
     * Verifies whether this solution is
     * feasible for the instance been solved.
     *
     * @return true if this solution is feasible;
     *         false otherwise.
     *********************************************/
    bool is_feasible() const;

    /*******************************************************************
     * Verifies whether this solution dominates the specified one.
     *
     * @param solution the solution whose domination is to be verified.
     *
     * @return true if this solution dominated the specified one;
     *         false otherwise.
     *******************************************************************/
    bool dominates(const Solution & solution) const;

    /*******************************************************
     * Standard input operator.
     *
     * @param is       standard input stream object.
     * @param solution the solution.
     *
     * @return the stream object.
     *******************************************************/
    friend std::istream & operator >>(std::istream & is,
                                      Solution & solution);

    /*************************************************************
     * Standard output operator.
     *
     * @param os       standard output stream object.
     * @param solution the solution.
     *
     * @return the stream object.
     *************************************************************/
    friend std::ostream & operator <<(std::ostream & os,
                                      const Solution & solution);
};

}
