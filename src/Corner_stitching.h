/*
 * File:    main.cpp
 * Author:  Alstin Lin
 * Date:    2025-10-12
 * Brief:   Implementation of the Corner Stitching data structure.
 */

#include "datatype.h"
#include "parser_file.h"

#include <unordered_map>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>
#include <list>
using namespace std;

#define COR(A) A->cor
#define DIM(A) A->dim
#define NAM(A) A->name
#define P_NAM(A) A->pad_name
#define LB_P(A)  A->lb_ptr
#define BL_P(A)  A->bl_ptr
#define RT_P(A)  A->rt_ptr
#define TR_P(A)  A->tr_ptr

#ifndef Corner_stitching_H
#define Corner_stitching_H

class Corner_stitching{
private:

public:
    struct TILE{
        string name;
        string pad_name=".";
        TILE *lb_ptr, *bl_ptr, *rt_ptr, *tr_ptr;
        struct FPOS cor, dim; 
        TILE(string n, struct FPOS c, struct FPOS d) : name(n), cor(c), dim(d), 
                                          lb_ptr(nullptr), bl_ptr(nullptr), rt_ptr(nullptr), tr_ptr(nullptr) {}
    };

    int b_num=1, 
        s_num=1;

    FPOS die_lb, 
         die_ur;

    TILE* rb_ptr=nullptr;

    TILE* process_TILE=nullptr;
    
    queue<TILE*> TILE_intersection;
    queue<TILE*> o2s_TR;
    queue<TILE*> o2s_RT;
    queue<TILE*> o2s_BL;
    queue<TILE*> o2s_LB;

    unordered_map<string, TILE*> TILE_map;
    unordered_map<string, TILE*> SPACE_map;

    Corner_stitching(FPOS cor_lb,FPOS cor_rt) : die_lb(cor_lb), die_ur(cor_rt) {}

    TILE* find_TILE(FPOS cor, TILE* rb_ptr);

    bool  TILE_check_overlap(FPOS lb, FPOS rt);
    void  TILE_check_overlap_op(TILE *ptr, FPOS cor, FPOS dim, float lb_boudary, bool &can_insert);
    
    void  TILE_enumerate(FPOS lb, FPOS rt);
    void  TILE_enumerate_op(TILE *ptr, FPOS cor, FPOS dim, float lb_boudary);

    void  other2self_find(TILE* blk_ptr, int cas, int cnt);
    void  other2self_modify(TILE* blk_ptr, int cnt);

    TILE* split_vertical_up(TILE *blk, float y, int cnt);
    TILE* split_vertical_down(TILE *blk, float y, int cnt);
    TILE* split_horizontal_left(TILE *blk, float x, int cnt);
    TILE* split_horizontal_right(TILE *blk, float x, int cnt);

    void  merge_vertical(TILE *blk, int cnt);
    void  merge_horizontal(TILE *blk, int cnt);

    void  TILE_creation(FPOS lb, FPOS rt, string pad_name, int cnt);

    void  TILE_deletion(TILE *tile, int cnt);

    void  pad_insertion(unordered_map<string, PAD*>  &pad_vec);

    float pad_insertion_ordered(vector<pair<string, int>> &order_vec, class Parser_file &P, bool init);
};

#endif
