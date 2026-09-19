#pragma once

#define NSBRKGA_MULTIPLE_INCLUSIONS

#include "nsbrkga.hpp"
#include <istream>
#include <ostream>
#include <string>
#include <utility>
#include <vector>

namespace mocvrp {
/*********************************************************
 * The Instance class represents an instance of the
 * Multi-Objective Capacitated Vehicle Routing Problem
 * with Optional Service.
 *********************************************************/
class Instance {
    public:
    /*********************
     * The instance name.
     *********************/
    std::string name;

    /************************
     * The instance comment.
     ************************/
    std::string comment;

    /****************************
     * The number of objectives.
     ****************************/
    unsigned num_objectives;

    /*****************************************************
     * The number of vertices, the depot 0 included.
     *****************************************************/
    unsigned num_vertices;

    /*****************************************
     * The number of customers.
     *****************************************/
    unsigned num_customers;

    /*****************************
     * The vehicles capacity.
     *****************************/
    unsigned capacity;

    /***************************
     * The vertices coordinates.
     ***************************/
    std::vector<std::pair<double, double>> coord;

    /************************
     * The adjacency matrix.
     ************************/
    std::vector<std::vector<double>> adj;

    /*********************************************************
     * The demand of each vertex, zero for the depot.
     *********************************************************/
    std::vector<unsigned> demand;

    /*********************************************************
     * The number of orders of each vertex, zero for the depot.
     *********************************************************/
    std::vector<unsigned> orders;

    /*************************************************************
     * Whether the distances come from an explicit weight matrix.
     *************************************************************/
    bool has_explicit_weights;

    /*********************************
     * The optimization senses.
     *********************************/
    std::vector<NSBRKGA::Sense> senses;

    /********************************
     * This instance primal bounds.
     ********************************/
    std::vector<double> primal_bound;

    private:
    /******************************
     * Initializes a new instance.
     ******************************/
    void init();

    public:
    /***********************************************************
     * Constructs a new instance.
     *
     * @param coord    the coordinates of the vertices.
     * @param demand   the demand of each vertex.
     * @param capacity the vehicles capacity.
     ***********************************************************/
    Instance(const std::vector<std::pair<double, double>> & coord,
             const std::vector<unsigned> & demand,
             const unsigned capacity);

    /***********************************************************
     * Constructs a new instance.
     *
     * @param adj      the adjacency matrix.
     * @param demand   the demand of each vertex.
     * @param capacity the vehicles capacity.
     ***********************************************************/
    Instance(const std::vector<std::vector<double>> & adj,
             const std::vector<unsigned> & demand,
             const unsigned capacity);

    /********************************
     * Constructs an empty instance.
     ********************************/
    Instance();

    /***********************************************************
     * Verifies whether this instance is valid.
     *
     * @return true if this instance is valid; false otherwise.
     ***********************************************************/
    bool is_valid() const;

    /**************************************************************************
     * Standard input operator.
     *
     * @param is       standard input stream object.
     * @param instance the instance.
     *
     * @return the stream object.
     **************************************************************************/
    friend std::istream & operator >>(std::istream & is, Instance & instance);

    /*************************************************************
     * Standard output operator.
     *
     * @param os       standard output stream object.
     * @param instance the instance.
     *
     * @return the stream object.
     *************************************************************/
    friend std::ostream & operator <<(std::ostream & os,
                                      const Instance & instance);
};

}
