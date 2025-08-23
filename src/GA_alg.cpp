#include "GA_alg.h"
#include "datatype.h"
#include "parser_file.h"
#include "WL_optimizer.h"
#include "Corner_stitching.h"

#include <iostream>
#include <vector>
#include <string>  
#include <algorithm>
#include <random>
#include <chrono>

using namespace std;

vector<Gene_algorithm::Individual> Gene_algorithm::initialize_population(const vector<pair<string, int>> &initial_order, class Corner_stitching& CS, class Parser_file& P) {
    vector<Individual> population;
    for (int i = 0; i < POPULATION_SIZE; ++i) {
        Individual individual;
        individual.order = initial_order;
        shuffle(individual.order.begin(), individual.order.end(), default_random_engine(chrono::system_clock::now().time_since_epoch().count()));
        individual.cost = CS.pad_insertion_ordered(individual.order, P, true);
        population.push_back(individual);
    }
    return population;
}

Gene_algorithm::Individual Gene_algorithm::tournament_selection(const vector<Individual> &population) {
    int tournament_size = 5;
    vector<Individual> tournament;
    for (int i = 0; i < tournament_size; ++i) {
        tournament.push_back(population[rand() % POPULATION_SIZE]);
    }
    return *min_element(tournament.begin(), tournament.end());
}

vector<pair<string, int>> Gene_algorithm::crossover(const vector<pair<string, int>> &parent1, const vector<pair<string, int>> &parent2) {
    int size = parent1.size();
    vector<pair<string, int>> child(size, {"", 0});
    map<string, bool> unfilled;

    for (const auto &pair : unfilled) {
        if (pair.first.empty()) {
            throw std::logic_error("Unfilled map contains an empty or invalid key.");
        }
    }

    for (const auto &p : parent1) {
        if (p.first.empty()) {
            throw std::logic_error("Parent1 contains an empty or invalid string.");
        }
        unfilled[p.first] = true;
    }

    int start = rand() % size;
    int end = start + (rand() % (size - start));

    for (int i = start; i <= end; ++i) {
        child[i] = parent1[i];
        unfilled[parent1[i].first] = false;
    }

    int parent2_index = -1;

    for (int child_index = 0; child_index < child.size(); ++child_index) {
        if (child[child_index].first.empty()) { 
            while (true) {
                parent2_index++;
                if (parent2_index >= parent2.size()) {
                    throw std::logic_error("Parent2 index out of bounds.");
                }
                string gene = parent2[parent2_index].first;
                if (unfilled[gene]) {
                    child[child_index] = parent2[parent2_index];
                    unfilled[gene] = false;
                    break;
                }
            }
        }
    }

    for (auto it = unfilled.begin(); it != unfilled.end(); it++) {
        it->second = true;
    }

    return child;
}

void Gene_algorithm::mutate(vector<pair<string, int>> &individual) {
    int size = individual.size();
    if ((double)rand() / RAND_MAX < MUTATION_RATE) {
        int index1 = rand() % size;
        int index2 = rand() % size;
        if(index1 == index2)
            return;
        swap(individual[index1], individual[index2]);
    }
    if ((double)rand() / RAND_MAX < MUTATION_RATE * 0.5) {
        int index1 = rand() % size;
        int index2 = rand() % size;
        if(index1 > index2)
            swap(index1, index2);
        if(index1 == index2)
            return;
        reverse(individual.begin()+index1, individual.begin()+index2);
    }
    if ((double)rand() / RAND_MAX < MUTATION_RATE * 3) {
        int index1 = rand() % size;
        individual[index1].second = rand() % 8;
    }
    return;
}

void log_debug(const std::string &message) {
    std::ofstream log_file("debug_log.txt", std::ios::app);
    log_file << message << std::endl;
    log_file.close();
}

vector<pair<string, int>> Gene_algorithm::genetic_algorithm(vector<pair<string, int>> &initial_order, class Corner_stitching& CS, class Parser_file& P) {
    
    for(int i=0; i<initial_order.size(); i++){
        unfilled[initial_order[i].first] = true;
    }

    vector<Individual> population = initialize_population(initial_order, CS, P);
    vector<Individual> new_population;

    for (int gen = 0; gen < GENERATIONS; ++gen) {
        new_population.clear();

        sort(population.begin(), population.end());

        new_population.push_back(population[0]);
        new_population.push_back(population[1]); 
        new_population.push_back(population[2]);
        new_population.push_back(population[4]);  

        cout<<gen<<": "<<population[0].cost<<endl;

        while (new_population.size() < POPULATION_SIZE) {
            Individual parent1 = tournament_selection(population);
            Individual parent2 = tournament_selection(population);

            Individual child;
            if ((double)rand() / RAND_MAX < CROSSOVER_RATE) {
                child.order = crossover(parent1.order, parent2.order);
            } else {
                child.order = parent1.order;
            }

            mutate(child.order);

            child.cost = CS.pad_insertion_ordered(child.order, P, true);
            new_population.push_back(child);
        }
        population = new_population;
    }
    sort(population.begin(), population.end());
    cout<<population[0].cost<<endl;
    return population[0].order;
}
