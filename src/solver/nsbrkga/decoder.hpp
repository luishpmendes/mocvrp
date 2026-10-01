#pragma once

#include "chromosome.hpp"
#include "solution/solution.hpp"

namespace mocvrp {
/***********************************************************************
 * The Decoder class decodes a chromosome into a solution for the
 * Multi-Objective Capacitated Vehicle Routing Problem with Optional
 * Service.
 *
 * The chromosome has two keys per customer: the second half assigns
 * each customer to a route, or to none, and the first half defines the
 * visiting order within each route, exactly as Solution::decode_key
 * prescribes.
 ***********************************************************************/
class Decoder {
    public:
    /****************************
     * The instance been solved.
     ****************************/
    const Instance & instance;

    /**********************************************************
     * The served customers, with their routes and keys, sorted,
     * of each thread.
     **********************************************************/
    std::vector<std::vector<std::tuple<unsigned, double, unsigned>>>
        permutation_of_thread;

    /*********************************************************************
     * The index, within the permutation, at which each route starts, of
     * each thread.
     *********************************************************************/
    std::vector<std::vector<unsigned>> route_begin_of_thread;

    /**************************************************
     * The value of the decoded solution of each thread.
     **************************************************/
    std::vector<std::vector<double>> value_of_thread;

    /***************************************************************
     * Constructs a new decoder.
     *
     * @param instance    the instance been solved.
     * @param num_threads the number of parallel decoding threads.
     ***************************************************************/
    Decoder(const Instance & instance, unsigned num_threads);

    /********************************************************************
     * Decodes the specified chromosome.
     *
     * @param chromosome the chromosome to be decoded.
     * @param rewrite    whether the chromosome must be rewritten.
     *
     * @return the value of the solution the chromosome represents.
     ********************************************************************/
    std::vector<double> decode(NSBRKGA::Chromosome & chromosome, bool rewrite);
};

}
