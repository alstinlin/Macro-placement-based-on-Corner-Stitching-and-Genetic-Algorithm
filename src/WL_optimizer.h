#include "Corner_stitching.h"
#include "datatype.h"
#include "parser_file.h"

#include <bits/stdc++.h>
#include <sys/stat.h>
#include <errno.h>
using namespace std;

#ifndef WL_optimizer_H
#define WL_optimizer_H

class wl_opt{
public:

    unordered_map<string, bool> net_checked;

    struct ComparePair {
        bool operator()(const pair<string, float>& a, const pair<string, float>& b) {
            return a.second < b.second;  
        }
    };
    priority_queue<pair<string, float>, vector<pair<string, float>>, ComparePair> opt_pq;

    void  fill_pq(class Parser_file &P);
    float cal_hpwl(class Parser_file &P, class Corner_stitching &CS);
    void  CG_optimize(class Parser_file &P, class Corner_stitching &CS, bool refresh);
    int   pad_refinement(class Parser_file &P, class Corner_stitching &A, float move_para, int cnt);
        
private:
};

void log_debug_opt(const std::string &message);


#endif