#pragma once

#include "chromosome.hpp"
#include "instance/instance.hpp"

namespace mocvrp {
/***********************************************************************
 * The Decoder class decodes a chromosome into a solution for the
 * Multi-Objective Capacitated Vehicle Routing Problem with Optional
 * Service.
 *
 * The chromosome has two keys per customer: the first half defines the
 * visiting order and the second half defines which customers are
 * served, exactly as the Solution random key constructor prescribes.
 ***********************************************************************/
class Decoder {
    public:
    /****************************
     * The instance been solved.
     ****************************/
    const Instance & instance;

    /**********************************************************
     * The served customers and their keys, sorted, of each
     * thread.
     **********************************************************/
    std::vector<std::vector<std::pair<double, unsigned>>> permutation_of_thread;

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
