#include <bits/stdc++.h>

#include "GA_alg.h"
#include "datatype.h"
#include "parser_file.h"
#include "WL_optimizer.h"
#include "Corner_stitching.h"


using namespace std;

int main(int argc, char* argv[]){

    ofstream ofs("debug_log.txt", ios::out);
    ofs.clear();

    class Parser_file P;
    P.parser_aux(argv[1]);

    class Corner_stitching CS(P.die_lb, P.die_ur);
    CS.pad_insertion(P.fixed_pad_vec);

    vector<pair<string, int>> best_order;
    vector<pair<string, int>> movable_pad;
    vector<float>             pad_area;
    float                     best_result;

    for(int i=0; i<P.movable_pad_vec.size(); i++){
        movable_pad.push_back({P.movable_pad_vec[i], 0});
    }

    class Gene_algorithm GA;
    best_order  = GA.genetic_algorithm(movable_pad, CS, P);
    best_result = CS.pad_insertion_ordered(best_order, P, false);

    cout<<"GA best result: "<<best_result<<endl;

    if(best_result == (float)1e30){
        class wl_opt opt;
        opt.CG_optimize(P, CS, true);
        cout<<"CG optimizing result: "<<opt.cal_hpwl(P, CS)<<endl;
        for(int i = 0; i < best_order.size(); i++){
            best_order[i].second = 0;
        }
    }

    P.parser_pl_out(argv[1], CS, best_order);
    P.out_matlab(CS, "result.m");

    return 0;
}