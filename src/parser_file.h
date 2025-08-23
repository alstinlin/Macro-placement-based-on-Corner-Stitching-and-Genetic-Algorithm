#ifndef parser_file_H
#define parser_file_H

#include <bits/stdc++.h>

#include "datatype.h"
#include "Corner_stitching.h"

using namespace std;

#define KE(E) E.first
#define VL(E) E.second

class Parser_file{
private:
public:
    string File_name;
    float hpwl=0.0, lse=0.0;
    int cell_num=0, pad_num=0, net_num=0; 
    FPOS die_lb, die_ur;
    float test=0;

    unordered_map<string, bool>    cell_pad;
    unordered_map<string, MODULE*> cell_vec;
    unordered_map<string, PAD*>    pad_vec;
    unordered_map<string, NET*>    net_vec;

    unordered_map<string, PAD*>    fixed_pad_vec;
    vector<string>                 movable_pad_vec;

    void parser_aux(string file_name);
    void parser_nodes(string file_name);
    void parser_nets(string file_name);
    void parser_pl(string file_name);

    void parser_pl_out(string file_name, class Corner_stitching &CS, vector<pair<string, int>>& best_order);

    void out_pad_matlab(string file_name);
    void out_pad_dpx(string file_name);
    void out_matlab(class Corner_stitching &CS, string file_name);
    void out_matlab_overlap(class Corner_stitching &CS, string file_name);
    void out_matlab_detection(class Corner_stitching &CS, string file_name, string tile);
};

#endif

























