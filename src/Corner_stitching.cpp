#include "datatype.h"
#include "parser_file.h"
#include "Corner_stitching.h"

#include <unordered_map>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <queue>

using namespace std;      

//*    The TILE located at (X, Y) with a width of W and a height of H,                                  *//
//*    will occupy the region bounded by [X, X + W] along the X-axis and [Y, Y + H] along the Y-axis.    *//

//*    if (X', Y) lies on the intersection edge between two horizontally adjacent TILEs                               *//
//*    the find_TILE function will return a pointer to the TILE on the right-hand side of the intersection.         *//

//*    if (X', Y) lies on the intersection edge between two vertically adjacent TILEs                                 *//
//*    the find_TILE function will return a pointer to the TILE located below the intersection.                     *//

Corner_stitching::TILE* Corner_stitching::find_TILE(FPOS cor, TILE* ptr){
    if(Y(cor) >  Y(die_ur))   return nullptr;
    if(Y(cor) <= Y(die_lb))   return nullptr;
    if(X(cor) <  X(die_lb))   return nullptr;
    if(X(cor) >= X(die_ur)+1) return nullptr;

    if(Y(COR(ptr))+H(DIM(ptr)) < Y(cor)){
        if(RT_P(ptr)!=nullptr)
            return find_TILE(cor, RT_P(ptr));
        else
            return nullptr;
    } else{
        if(X(COR(ptr)) > X(cor)){
            if(BL_P(ptr)!=nullptr)
                return find_TILE(cor, BL_P(ptr));
            else
                return nullptr;
        } else
            return ptr;
    }
}

bool Corner_stitching::TILE_check_overlap(FPOS lb, FPOS rt){
    FPOS cur_cor={X(lb), Y(rt)};
    bool can_insert=true;
    do{
        TILE *cur_TILE=find_TILE(cur_cor, rb_ptr);
        if(cur_TILE==nullptr) return can_insert;
        TILE_check_overlap_op(cur_TILE, lb, rt, Y(COR(cur_TILE)), can_insert);
        cur_cor={X(lb), Y(COR(cur_TILE))};
    } while(Y(cur_cor) > Y(lb) && can_insert==true);
    return can_insert;
}
void Corner_stitching::TILE_check_overlap_op(TILE *ptr, FPOS lb, FPOS rt, float lb_bound, bool &can_insert){
    if(ptr == nullptr || can_insert == false) return;
    if(NAM(ptr)[0] != 's' && ptr != process_TILE) can_insert=false;

    if(TR_P(ptr)!=nullptr)
      if((X(COR(ptr))+W(DIM(ptr)) < X(rt)))
          TILE_check_overlap_op(TR_P(ptr), lb, rt, Y(COR(ptr)), can_insert);

    if(LB_P(ptr)!=nullptr)
      if(Y(COR(ptr)) > lb_bound)
          TILE_check_overlap_op(LB_P(ptr), lb, rt, lb_bound, can_insert);

    return;
}

void Corner_stitching::TILE_enumerate(FPOS lb, FPOS rt){
    FPOS cur_cor={X(lb), Y(rt)}; 
    do{
        TILE *cur_TILE=find_TILE(cur_cor, rb_ptr);
        if(cur_TILE==nullptr) return;
        TILE_enumerate_op(cur_TILE, lb, rt, Y(COR(cur_TILE)));
        cur_cor={X(lb), Y(COR(cur_TILE))};
    } while(Y(cur_cor) > Y(lb));
    return;
}
void Corner_stitching::TILE_enumerate_op(TILE *ptr, FPOS lb, FPOS rt, float lb_bound){

    if(Y(COR(ptr)) < Y(rt) && X(COR(ptr))+W(DIM(ptr)) > X(lb))
      if((Y(COR(ptr)) >= lb_bound) || (lb_bound <= Y(lb) && Y(COR(ptr)) <= Y(lb)))
            TILE_intersection.push(ptr);

    if(TR_P(ptr)!=nullptr)
      if((X(COR(ptr))+W(DIM(ptr)) < X(rt)))
          TILE_enumerate_op(TR_P(ptr), lb, rt, Y(COR(ptr)));

    if(LB_P(ptr)!=nullptr)
      if(Y(COR(ptr)) > lb_bound)
          TILE_enumerate_op(LB_P(ptr), lb, rt, lb_bound);

    return;
}

void Corner_stitching::other2self_find(TILE *self, int cas, int cnt){
    if(cas==1){
        TILE *other=find_TILE({X(COR(self))-1, Y(COR(self))+1}, rb_ptr);
        if(other==nullptr) return;
        while(Y(COR(other))+H(DIM(other)) <= Y(COR(self))+H(DIM(self))){
            o2s_TR.push(other);
            if(RT_P(other)!=nullptr) other=RT_P(other);
            else break;
        }
    } else if(cas==2){
        TILE *other=find_TILE({X(COR(self)), Y(COR(self))}, rb_ptr);
        if(other==nullptr) return;
        while(X(COR(other))+W(DIM(other)) <= X(COR(self))+W(DIM(self))){
            o2s_RT.push(other);
            if(TR_P(other)!=nullptr) other=TR_P(other);
            else break;
        }
    } else if(cas==3){
        TILE *other=find_TILE({X(COR(self))+W(DIM(self)), Y(COR(self))+H(DIM(self))}, rb_ptr);
        if(other==nullptr) return;
        while(Y(COR(other)) >= Y(COR(self))){
            o2s_BL.push(other);
            if(LB_P(other)!=nullptr) other=LB_P(other);
            else break;
        }
    } else if(cas==4){
        TILE *other=find_TILE({X(COR(self))+W(DIM(self))-1, Y(COR(self))+H(DIM(self))+1}, rb_ptr);
        if(other==nullptr) return;
        while(X(COR(other)) >= X(COR(self))){
            o2s_LB.push(other);
            if(BL_P(other)!=nullptr) other=BL_P(other);
            else break;
        }
    }
    return;
}

void Corner_stitching::other2self_modify(TILE* self, int cnt){
    TILE *other=nullptr;
    while(o2s_TR.size()!=0){
        other=o2s_TR.front();
        TR_P(other)=self;
        o2s_TR.pop();
    }
    while(o2s_RT.size()!=0){
        other=o2s_RT.front();
        RT_P(other)=self;
        o2s_RT.pop();
    }
    while(o2s_BL.size()!=0){
        other=o2s_BL.front();
        BL_P(other)=self;
        o2s_BL.pop();
    }
    while(o2s_LB.size()!=0){
        other=o2s_LB.front();
        LB_P(other)=self;
        o2s_LB.pop();
    }
    return;
}

//*         The original TILE is split into two vertically adjacent TILEs.                                        *//
//*         The upper TILE retains the original TILE's properties, while a new TILE is created beneath it.        *//

Corner_stitching::TILE* Corner_stitching::split_vertical_up(TILE *blk, float y, int cnt){
    string TILE_up_name="s"+to_string(s_num++);
    TILE *TILE_up=new TILE(TILE_up_name,
                               {X(COR(blk)), y},
                               {W(DIM(blk)), Y(COR(blk))+Y(DIM(blk))-y});
    TILE_map[NAM(TILE_up)] = TILE_up;
    SPACE_map[NAM(TILE_up)] = TILE_up;
    

    other2self_find(TILE_up, 1, cnt),
    other2self_find(TILE_up, 3, cnt),
    other2self_find(TILE_up, 4, cnt);

    LB_P(TILE_up)=blk,
    BL_P(TILE_up)=find_TILE({X(COR(TILE_up))-1, Y(COR(TILE_up))+1}, rb_ptr),
    RT_P(TILE_up)=RT_P(blk),
    TR_P(TILE_up)=TR_P(blk);

    RT_P(blk)=TILE_up;
    TR_P(blk)=find_TILE({X(COR(blk))+X(DIM(blk)), y}, rb_ptr),
    H(DIM(blk))-=H(DIM(TILE_up));

    other2self_modify(TILE_up, cnt);

    return TILE_up;
}

Corner_stitching::TILE* Corner_stitching::split_vertical_down(TILE *blk, float y, int cnt){
    string TILE_down_name="s"+to_string(s_num++);
    TILE *TILE_down=new TILE(TILE_down_name,
                               {X(COR(blk)), Y(COR(blk))},
                               {W(DIM(blk)), y-Y(COR(blk))});
    TILE_map[NAM(TILE_down)] = TILE_down;
    SPACE_map[NAM(TILE_down)] = TILE_down;

    other2self_find(TILE_down, 1, cnt),
    other2self_find(TILE_down, 2, cnt),
    other2self_find(TILE_down, 3, cnt);

    LB_P(TILE_down)=LB_P(blk),
    BL_P(TILE_down)=BL_P(blk),
    RT_P(TILE_down)=blk,
    TR_P(TILE_down)=find_TILE({X(COR(TILE_down))+X(DIM(TILE_down)), Y(COR(TILE_down))+Y(DIM(TILE_down))}, rb_ptr);

    LB_P(blk)=TILE_down;
    BL_P(blk)=find_TILE({X(COR(blk))-1, y+1}, rb_ptr);
    Y(COR(blk))=y,
    H(DIM(blk))-=H(DIM(TILE_down));

    other2self_modify(TILE_down, cnt);

    return TILE_down;
}

//*         The original TILE is split into two horizontally adjacent TILEs.                                     *//
//*         The left TILE retains the original properties, while a new TILE is created to its right.             *//

Corner_stitching::TILE* Corner_stitching::split_horizontal_left(TILE *blk, float x, int cnt){
    string TILE_left_name="s"+to_string(s_num++);
    TILE *TILE_left=new TILE(TILE_left_name,
                                {X(COR(blk)), Y(COR(blk))},
                                {x-X(COR(blk)), H(DIM(blk))});
    TILE_map[NAM(TILE_left)] = TILE_left;
    SPACE_map[NAM(TILE_left)] = TILE_left;

    other2self_find(TILE_left, 1, cnt),
    other2self_find(TILE_left, 2, cnt),
    other2self_find(TILE_left, 4, cnt);

    LB_P(TILE_left)=LB_P(blk),
    BL_P(TILE_left)=BL_P(blk),
    RT_P(TILE_left)=find_TILE({x-1, Y(COR(blk))+H(DIM(blk))+1}, rb_ptr),
    TR_P(TILE_left)=blk;

    BL_P(blk)=TILE_left,
    LB_P(blk)=find_TILE({x, Y(COR(blk))}, rb_ptr);
    X(COR(blk))=x;
    W(DIM(blk))-=W(DIM(TILE_left));

    other2self_modify(TILE_left, cnt);

    return TILE_left;
}

Corner_stitching::TILE* Corner_stitching::split_horizontal_right(TILE *blk, float x, int cnt){
    string TILE_right_name="s"+to_string(s_num++);
    TILE *TILE_right=new TILE(TILE_right_name,
                                {x, Y(COR(blk))},
                                {X(COR(blk))+W(DIM(blk))-x, H(DIM(blk))});
    TILE_map[NAM(TILE_right)] = TILE_right;
    SPACE_map[NAM(TILE_right)] = TILE_right;

    other2self_find(TILE_right, 2, cnt),
    other2self_find(TILE_right, 3, cnt),
    other2self_find(TILE_right, 4, cnt);

    LB_P(TILE_right)=find_TILE({x,Y(COR(blk))}, rb_ptr),
    BL_P(TILE_right)=blk,
    RT_P(TILE_right)=RT_P(blk),
    TR_P(TILE_right)=TR_P(blk);

    TR_P(blk)=TILE_right,
    RT_P(blk)=find_TILE({x-1, Y(COR(blk))+H(DIM(blk))+1}, rb_ptr);
    W(DIM(blk))-=W(DIM(TILE_right));

    other2self_modify(TILE_right, cnt);

    return TILE_right;
}

void Corner_stitching::merge_vertical(TILE *blk, int cnt){
    if(RT_P(blk)!=nullptr){
        TILE *TILE_up=RT_P(blk);
        if(X(COR(blk))==X(COR(TILE_up)) && W(DIM(blk))==W(DIM(TILE_up)) && P_NAM(blk)==P_NAM(TILE_up)){
            other2self_find(TILE_up, 1, cnt),
            other2self_find(TILE_up, 3, cnt),
            other2self_find(TILE_up, 4, cnt);

            H(DIM(blk))+=H(DIM(TILE_up));
            RT_P(blk)=RT_P(TILE_up),
            TR_P(blk)=TR_P(TILE_up);

            other2self_modify(blk, cnt);

            if(TILE_map.find(NAM(TILE_up))!=TILE_map.end())
                TILE_map.erase(NAM(TILE_up));
            if(SPACE_map.find(NAM(TILE_up))!=SPACE_map.end())
                SPACE_map.erase(NAM(TILE_up));

            delete TILE_up;
        } 
    }

    if(LB_P(blk)!=nullptr && cnt==1){
        TILE *TILE_down=LB_P(blk);
        if(X(COR(blk))==X(COR(TILE_down)) && W(DIM(blk))==W(DIM(TILE_down)) && P_NAM(blk)==P_NAM(TILE_down)){
            other2self_find(TILE_down, 1, cnt),
            other2self_find(TILE_down, 2, cnt),
            other2self_find(TILE_down, 3, cnt);

            Y(COR(blk))=Y(COR(TILE_down)),
            H(DIM(blk))+=H(DIM(TILE_down));
            LB_P(blk)=LB_P(TILE_down),
            BL_P(blk)=BL_P(TILE_down);

            other2self_modify(blk, cnt);

            if(TILE_map.find(NAM(TILE_down))!=TILE_map.end())
                TILE_map.erase(NAM(TILE_down));
            if(SPACE_map.find(NAM(TILE_down))!=SPACE_map.end())
                SPACE_map.erase(NAM(TILE_down));

            delete TILE_down;
        }
    }
    return;
}

void Corner_stitching::merge_horizontal(TILE *blk, int cnt){
    if(TR_P(blk)!=nullptr){
        TILE *TILE_right=TR_P(blk);
        if(Y(COR(blk))==Y(COR(TILE_right)) && H(DIM(blk))==H(DIM(TILE_right)) && P_NAM(blk)==P_NAM(TILE_right)){

            other2self_find(TILE_right, 2, cnt);
            other2self_find(TILE_right, 3, cnt);
            other2self_find(TILE_right, 4, cnt);

            TR_P(blk)=TR_P(TILE_right);
            RT_P(blk)=RT_P(TILE_right);
            W(DIM(blk))+=W(DIM(TILE_right));

            other2self_modify(blk, cnt);

            if(TILE_map.find(NAM(TILE_right))!=TILE_map.end())
                TILE_map.erase(NAM(TILE_right));
            if(SPACE_map.find(NAM(TILE_right))!=SPACE_map.end())
                SPACE_map.erase(NAM(TILE_right));

            delete TILE_right;
        }
    }

    if(BL_P(blk)!=nullptr && cnt>=1){
        TILE *TILE_left=BL_P(blk);
        if(Y(COR(blk))==Y(COR(TILE_left)) && H(DIM(blk))==H(DIM(TILE_left)) && P_NAM(blk)==P_NAM(TILE_left)){

            other2self_find(TILE_left, 1, cnt);
            other2self_find(TILE_left, 2, cnt);
            other2self_find(TILE_left, 4, cnt);

            BL_P(blk)=BL_P(TILE_left);
            LB_P(blk)=LB_P(TILE_left);
            X(COR(blk))=X(COR(TILE_left));
            W(DIM(blk))+=W(DIM(TILE_left));

            other2self_modify(blk, cnt);

            if(TILE_map.find(NAM(TILE_left))!=TILE_map.end())
                TILE_map.erase(NAM(TILE_left));
            if(SPACE_map.find(NAM(TILE_left))!=SPACE_map.end())
                SPACE_map.erase(NAM(TILE_left));

            delete TILE_left;
        }
    }
    return;
}

void Corner_stitching::TILE_creation(FPOS lb, FPOS rt, string pad_name, int cnt){
    TILE_enumerate(lb, rt);
    
    do{
        TILE *intersect_TILE=TILE_intersection.front();
        TILE_intersection.pop();

        TILE *new_TILE=nullptr;

        TILE_map.erase(NAM(intersect_TILE));
        SPACE_map.erase(NAM(intersect_TILE));
        NAM(intersect_TILE)="b"+to_string(b_num++);
        P_NAM(intersect_TILE)=pad_name;
        
        if(TILE_intersection.size()==0){
            TILE_map[P_NAM(intersect_TILE)]=intersect_TILE;
        }

        if(Y(COR(intersect_TILE))+H(DIM(intersect_TILE)) > Y(rt)){
            new_TILE=split_vertical_up(intersect_TILE, Y(rt), cnt);
        }   

        if(Y(COR(intersect_TILE)) < Y(lb)){
            new_TILE=split_vertical_down(intersect_TILE, Y(lb), cnt);
        }
        
        if(X(COR(intersect_TILE))+W(DIM(intersect_TILE)) > X(rt)){
            new_TILE=split_horizontal_right(intersect_TILE, X(rt), cnt);
            merge_vertical(new_TILE, 1);
        }
        
        if(X(COR(intersect_TILE)) < X(lb)){
            new_TILE=split_horizontal_left(intersect_TILE, X(lb), cnt);
            merge_vertical(new_TILE, 1);
        }
         
        merge_vertical(intersect_TILE, cnt); 
       
    } while(TILE_intersection.size()!=0);
}

void Corner_stitching::TILE_deletion(TILE *tile, int cnt){
    stack<TILE*> left_TILE_sequence;
    queue<TILE*> right_TILE_sequence;
    queue<TILE*> mid_TILE_sequence;

    TILE_map.erase(P_NAM(tile));
    NAM(tile)="s"+to_string(s_num++);
    P_NAM(tile)=".";
    TILE_map[NAM(tile)] = tile;

    TILE *right_tile = (X(COR(tile))+X(DIM(tile))==X(die_ur)) ?
                     nullptr : find_TILE({X(COR(tile))+X(DIM(tile)), Y(COR(tile))+Y(DIM(tile))}, rb_ptr);

    while(right_tile!=nullptr && Y(COR(right_tile)) + Y(DIM(right_tile)) > Y(COR(tile))){
        right_TILE_sequence.push(right_tile);
        if(LB_P(right_tile)!=nullptr) right_tile=LB_P(right_tile);
        else break;
    }

    TILE *left_tile=find_TILE({X(COR(tile))-1, Y(COR(tile))+1}, rb_ptr);
    while(left_tile!=nullptr && Y(COR(left_tile)) < Y(COR(tile)) + Y(DIM(tile))){
        left_TILE_sequence.push(left_tile);
        if(RT_P(left_tile)!=nullptr) left_tile=RT_P(left_tile);
        else break;
    }

    TILE *new_tile=nullptr;
    bool finish=false;

    if(right_TILE_sequence.size()){
        right_tile=right_TILE_sequence.front();
        right_TILE_sequence.pop();
    } else{
        mid_TILE_sequence.push(tile);
        finish=true;
    }

    if(!finish && Y(COR(right_tile)) + Y(DIM(right_tile)) > Y(COR(tile)) + Y(DIM(tile)) 
        && P_NAM(right_tile)=="."){
        new_tile=split_vertical_up(right_tile, Y(COR(tile))+Y(DIM(tile)), cnt);
    }
    
    while(!finish){

        if(Y(COR(right_tile)) < Y(COR(tile)) && P_NAM(right_tile)=="."){
            new_tile=split_vertical_down(right_tile, Y(COR(tile)), cnt);
        }

        if(Y(COR(right_tile)) > Y(COR(tile))){
            new_tile=split_vertical_up(tile, Y(COR(right_tile)), cnt);
        }

        if(right_TILE_sequence.size()>0){
            merge_horizontal(new_tile, 0);
            mid_TILE_sequence.push(new_tile);
            right_tile=right_TILE_sequence.front();
            right_TILE_sequence.pop();
        } else{
            merge_horizontal(tile, 0);
            mid_TILE_sequence.push(tile);
            finish=true;
        }
    }

    finish=false;

    if(left_TILE_sequence.size()){
        left_tile=left_TILE_sequence.top();
        left_TILE_sequence.pop();
    } else{
        finish=true;
    }

    tile=mid_TILE_sequence.front();
    mid_TILE_sequence.pop();

    if(!finish &&  Y(COR(tile)) + Y(DIM(tile)) < Y(COR(left_tile)) + Y(DIM(left_tile))
       && P_NAM(left_tile)=="."){
        new_tile=split_vertical_up(left_tile, Y(COR(tile))+Y(DIM(tile)), cnt);
    }

    while(!finish){
        if(Y(COR(tile)) > Y(COR(left_tile))){
            if(P_NAM(left_tile)=="."){
                new_tile=split_vertical_up(left_tile, Y(COR(tile)), cnt);
            }
            merge_horizontal(tile, 1);
            merge_vertical(tile, 0);
            if(mid_TILE_sequence.size()){
                tile=mid_TILE_sequence.front(),
                mid_TILE_sequence.pop();
            } else{
                finish=true;
            }
        } else if(Y(COR(tile)) < Y(COR(left_tile))){
            new_tile=split_vertical_up(tile, Y(COR(left_tile)), cnt);
            merge_horizontal(new_tile, 1);
            merge_vertical(new_tile, 0);
            if(left_TILE_sequence.size()){
                left_tile=left_TILE_sequence.top(),
                left_TILE_sequence.pop();
            } else{
                finish=true;
            }
        } else{
            merge_horizontal(left_tile, 0);
            if(!mid_TILE_sequence.size()&&!left_TILE_sequence.size())
                merge_vertical(left_tile, 1);
            else
                merge_vertical(left_tile, 0);
            if(mid_TILE_sequence.size()&&left_TILE_sequence.size()){
                tile=mid_TILE_sequence.front(),
                mid_TILE_sequence.pop();
                left_tile=left_TILE_sequence.top(),
                left_TILE_sequence.pop();
            } else{
                finish=true;
            }
        }
    }
    return;
}

void Corner_stitching::pad_insertion(unordered_map<string, PAD*>  &vec_in){
    TILE *die = new TILE("s0", die_lb, {X(die_ur)-X(die_lb), Y(die_ur)-Y(die_lb)});
    rb_ptr    = new TILE("s", {X(die_ur), Y(die_lb)}, {10.0, Y(die_ur)-Y(die_lb)});
    BL_P(rb_ptr) = die;
    TR_P(die)    = rb_ptr;
    TILE_map[NAM(die)]     = die;
    TILE_map[NAM(rb_ptr)]  = rb_ptr;
    SPACE_map[NAM(die)]    = die;
    SPACE_map[NAM(rb_ptr)] = rb_ptr;
    for(pair<string, PAD*> elem : vec_in){
        TILE_creation(LB(VL(elem)), UR(VL(elem)), KE(elem), 0);
    }
    return;
}

float Corner_stitching::pad_insertion_ordered(vector<pair<string, int>> &order_vec, class Parser_file &P, bool init){

    function<void(float &, float &)> swap_wh = [](float &w, float &h){
        float tempt=w;
        w=h;
        h=tempt;
    };

    function<void(float &, float &, int)> cal_offset = [](float &off_x, float &off_y, int ort){
        float lay_off_x=off_y,
              lay_off_y=off_x;

        if(ort == 0){
            off_x = off_x,
            off_y = off_y;
        } else if(ort == 5){
            off_x = lay_off_x,
            off_y = lay_off_y;
        } else if(ort == 2){
            off_x = -off_x,
            off_y = -off_y;
        } else if(ort == 7){
            off_x = -lay_off_x,
            off_y = -lay_off_y;
        } else if(ort == 4){
            off_x = -off_x,
            off_y = off_y;
        } else if(ort == 1){
            off_x = -lay_off_x,
            off_y = lay_off_y;
        } else if(ort == 6){
            off_x = off_x,
            off_y = -off_y;
        } else if(ort == 3){
            off_x = lay_off_x,
            off_y = -lay_off_y;
        }
    };

    function<bool(FPOS, FPOS, FPOS, FPOS)> inside_boundary = [](FPOS lb_cor, FPOS ur_cor,FPOS die_lb, FPOS die_ur)->bool{
        if(Y(ur_cor) <= Y(die_ur) && X(ur_cor) <= X(die_ur) && Y(lb_cor) >= Y(die_lb) && X(lb_cor) >= X(die_lb))
            return true;
        else
            return false;
    };

    function<FPOS(FPOS, FPOS, FPOS, int)> center_loc = [](FPOS space_cor, FPOS space_dim, FPOS pad_half_dim, int corner)->FPOS{
        FPOS center_cor;
        if(corner==0)
            center_cor = {X(space_cor)+X(pad_half_dim), Y(space_cor)+Y(pad_half_dim)};
        if(corner==1)
            center_cor = {X(space_cor)+X(pad_half_dim), Y(space_cor)+Y(space_dim)-Y(pad_half_dim)};
        if(corner==2)
            center_cor = {X(space_cor)+X(space_dim)-X(pad_half_dim), Y(space_cor)+Y(pad_half_dim)};
        if(corner==3)
            center_cor = {X(space_cor)+X(space_dim)-X(pad_half_dim), Y(space_cor)+Y(space_dim)-Y(pad_half_dim)};
        return center_cor;
    };

    function<void(float&, FPOS, FPOS, FPOS, float)> update_score = [](float &score, FPOS cor_pin, FPOS lb_net, FPOS ur_net, float weight) {
        if(X(cor_pin) < X(lb_net) && X(lb_net) != 1e9)
            score -= (X(lb_net) - X(cor_pin))*weight;
        if(Y(cor_pin) < Y(lb_net) && Y(lb_net) != 1e9)
            score -= (Y(lb_net) - Y(cor_pin))*weight;
        if(X(cor_pin) > X(ur_net) && X(ur_net) != 0.0)
            score -= (X(cor_pin) - X(ur_net))*weight;
        if(Y(cor_pin) > Y(ur_net) && Y(ur_net) != 0.0)
            score -= (Y(cor_pin) - Y(ur_net))*weight;
        return;
    };

    float hpwl = 0.0;

    for(pair<string, int> elem : order_vec){
        string pad_name    = elem.first;
        int    pad_orient  = elem.second; 
        float  best_score  = 0.0;
        TILE*  best_tile   = nullptr;
        int    best_corner = 0;

        PAD* pad = P.pad_vec[pad_name];
        FPOS pad_half_dim = WH(pad)/2;

        if(pad_orient == 1 || pad_orient == 3 || pad_orient == 5 || pad_orient == 7)
            swap_wh(X(pad_half_dim), Y(pad_half_dim));

        float base_score = ((X(P.die_ur) + Y(P.die_ur))) * PS(pad).size() + (X(P.die_ur) + Y(P.die_ur));

        for(pair<string, TILE*> space : SPACE_map){

            if(X(DIM(VL(space))) < 2 * X(pad_half_dim))
                continue;

            FPOS  lb_center = center_loc(COR(VL(space)), DIM(VL(space)), pad_half_dim, 0),
                  lu_center = center_loc(COR(VL(space)), DIM(VL(space)), pad_half_dim, 1),
                  rb_center = center_loc(COR(VL(space)), DIM(VL(space)), pad_half_dim, 2),
                  ru_center = center_loc(COR(VL(space)), DIM(VL(space)), pad_half_dim, 3);

            FPOS  lb_lb = lb_center - pad_half_dim,
                  lu_lb = lu_center - pad_half_dim,
                  rb_lb = rb_center - pad_half_dim,
                  ru_lb = ru_center - pad_half_dim;

            FPOS  lb_ur = lb_center + pad_half_dim,
                  lu_ur = lu_center + pad_half_dim,
                  rb_ur = rb_center + pad_half_dim,
                  ru_ur = ru_center + pad_half_dim;
            
            vector<float> score(4, 0.0); // 0: lb, 1: lu, 2: rb, 3: ru 

            if(inside_boundary(lb_lb, lb_ur, P.die_lb, P.die_ur)){
                if(TILE_check_overlap(lb_lb, lb_ur)){
                    score[0] = base_score;
                }    
            }

            if(inside_boundary(lu_lb, lu_ur, P.die_lb, P.die_ur)){
                if(TILE_check_overlap(lu_lb, lu_ur)){
                    score[1] = base_score;
                }   
            }

            if(inside_boundary(rb_lb, rb_ur, P.die_lb, P.die_ur)){
                if(TILE_check_overlap(rb_lb, rb_ur)){
                    score[2] = base_score;
                }       
            }

            if(inside_boundary(ru_lb, ru_ur, P.die_lb, P.die_ur)){
                if(TILE_check_overlap(ru_lb, ru_ur)){
                    score[3] = base_score;
                }       
            } 

            for(PIN* pin : PS(pad)){
                NET* net=N(pin);

                FPOS pin_off = OF(pin);

                cal_offset(X(pin_off), Y(pin_off), pad_orient);

                FPOS lb_pin = lb_center + pin_off,
                     lu_pin = lu_center + pin_off,
                     rb_pin = rb_center + pin_off,
                     ru_pin = ru_center + pin_off;

                FPOS lb_net=PLB(net),
                     ur_net=PRU(net);

                update_score(score[0], lb_pin, lb_net, ur_net, net->net_weight);
                update_score(score[1], lu_pin, lb_net, ur_net, net->net_weight);
                update_score(score[2], rb_pin, lb_net, ur_net, net->net_weight);
                update_score(score[3], ru_pin, lb_net, ur_net, net->net_weight);  

            }

            for(int i=0; i<4; i++){
                if(best_score < score[i]){
                    best_corner = i;
                    best_score  = score[i];
                    best_tile   = VL(space);
                }
            }

            if(best_score == base_score)
                break;
        }

        if(best_score <= 0){
            hpwl = 1e30;
            break;
        }
        
        FPOS  space_cor = COR(best_tile),
              space_dim = DIM(best_tile);

        FPOS  place_center,
              place_lb,
              place_ur;

        place_center = center_loc(space_cor, space_dim, pad_half_dim, best_corner);
        place_lb     = place_center - pad_half_dim;
        place_ur     = place_center + pad_half_dim;


        TILE_creation(place_lb, place_ur, NAM(pad), 0);

        for(PIN* pin : PS(pad)){
            NET* net=N(pin);

            FPOS pin_off = OF(pin);

            cal_offset(X(pin_off), Y(pin_off), pad_orient);

            FPOS place_pin = place_center + pin_off;
            
            FPOS& lb_net=PLB(net);
            FPOS& ur_net=PRU(net);

            if(X(lb_net) > X(place_pin)){
                X(lb_net) = X(place_pin);
            }
            if(Y(lb_net) > Y(place_pin)){
                Y(lb_net) = Y(place_pin);
            }
            if(X(ur_net) < X(place_pin)){
                X(ur_net) = X(place_pin);
            }
            if(Y(ur_net) < Y(place_pin)){
                Y(ur_net) = Y(place_pin);
            }
        }
    }

    for(pair<string, NET *> elem : P.net_vec){
        NET* net = VL(elem);

        if(hpwl <= 1e30){
            hpwl += (X(PRU(net)) - X(PLB(net))),
            hpwl += (Y(PRU(net)) - Y(PLB(net)));
        }

        if(init == false)
            continue;

        PLB(net) = {X(net->pin_lb_ori), Y(net->pin_lb_ori)};
        PRU(net) = {X(net->pin_ur_ori), Y(net->pin_ur_ori)};
    }

    if(init == false)
        return hpwl;

    for(auto it=SPACE_map.begin(); it!=SPACE_map.end(); ){
        TILE* tile = it->second;
        string tile_name = it->first;
        TILE_map.erase(tile_name),
        it = SPACE_map.erase(it);

        delete tile;
    }

    for(auto it=TILE_map.begin(); it!=TILE_map.end(); ){
        TILE* tile = it->second;
        string tile_name = it->first;
        it = TILE_map.erase(it);

        delete tile;
    }

    b_num = 1, s_num = 1;

    pad_insertion(P.fixed_pad_vec);

    return hpwl;
}