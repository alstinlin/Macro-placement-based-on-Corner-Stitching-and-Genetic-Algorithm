#include "Corner_stitching.h"
#include "parser_file.h"

#include <bits/stdc++.h>
#include <sys/stat.h>
#include <errno.h>
#include <vector>
#include <string>

using namespace std;

#ifndef GENETIC_ALGORITHM_H
#define GENETIC_ALGORITHM_H

class Gene_algorithm{
public:
    struct Individual {
        vector<pair<string, int>> order;
        double cost;

        bool operator<(const Individual &other) const {
            return cost < other.cost;
        }
    };

    unordered_map<string, bool> unfilled;

    const int GENERATIONS = 50; 
    const int POPULATION_SIZE = 40;       
    const double MUTATION_RATE = 0.1;  
    const double CROSSOVER_RATE = 0.75; 

    vector<Individual> initialize_population(const vector<pair<string, int>> &initial_order, class Corner_stitching& CS, class Parser_file& P);
    Individual         tournament_selection(const vector<Individual> &population);
    vector<pair<string, int>>     crossover(const vector<pair<string, int>> &parent1, const vector<pair<string, int>> &parent2);
    void                mutate(vector<pair<string, int>> &individual);
    vector<pair<string, int>>     genetic_algorithm(vector<pair<string, int>> &initial_order, class Corner_stitching& CS, class Parser_file& P);
};

void log_debug(const std::string &message);

#endif 
