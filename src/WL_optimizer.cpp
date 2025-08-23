#include "Corner_stitching.h"
#include "datatype.h"
#include "parser_file.h"
#include "WL_optimizer.h"

#include <bits/stdc++.h>
#include <sys/stat.h>
#include <math.h>
#include <errno.h>
using namespace std;

void wl_opt::fill_pq(class Parser_file &P){
    for(pair<string, PAD*> elem : P.pad_vec){
        float pad_priority=0;
        if(PS(VL(elem)).size()==0) 
            continue;
        if(F(VL(elem)))
            continue;
        for(int i=0; i<PS(VL(elem)).size(); i++){
            PIN *pin=PS(VL(elem))[i];
            if(N(pin)->same_pad || N(pin)->fix_net)
                continue;
            if(net_checked.find(NAM(N(pin)))!=net_checked.end())
                continue;
            net_checked[NAM(N(pin))]=true;
            pad_priority+=N(pin)->net_weight;
        }
        net_checked.clear();
        opt_pq.push({NAM(VL(elem)), pad_priority});
    }
    return;
}

int wl_opt::pad_refinement(class Parser_file &P, class Corner_stitching &CS, float move_para, int cnt){
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

    priority_queue<pair<string, float>, vector<pair<string, float>>, ComparePair> pq_copy;
    pq_copy=opt_pq;
    int reject=0;
    int cnt2=0;
    while(pq_copy.size()){

        //log_debug_opt(KE(pq_copy.top())+"\n");

        PAD *pad=P.pad_vec[KE(pq_copy.top())];
        pq_copy.pop();

        float grad_x=0.0, grad_y=0.0;
        vector<pair<string, FPOS>> pin_change_before(PS(pad).size());

        for(int j=0; j<PS(pad).size(); j++){
            PIN *pin=PS(pad)[j];
            float off_x=X(OF(pin)),
                  off_y=Y(OF(pin));

            cal_offset(off_x, off_y, ORT(pad));

            float pin_x = X(CT(pad)) + off_x,
                  pin_y = Y(CT(pad)) + off_y;

            float exp_x = exp(pin_x*0.002),
                  exp_y = exp(pin_y*0.002),
                  exp_x_n = exp(-pin_x*0.002),
                  exp_y_n = exp(-pin_y*0.002);

            grad_x += exp_x/X(N(pin)->ExpSum_P),
            grad_x -= exp_x_n/X(N(pin)->ExpSum_N);

            grad_y += exp_y/Y(N(pin)->ExpSum_P),
            grad_y -= exp_y_n/Y(N(pin)->ExpSum_N);

            pin_change_before[j]={NAM(N(pin)), {pin_x, pin_y}};
        }

        grad_x=-grad_x,
        grad_y=-grad_y; //movement orientation is opposite to gradient

        grad_x=grad_x*move_para*0.1,
        grad_y=grad_y*move_para*0.1;

        grad_x=floor(grad_x),
        grad_y=floor(grad_y);

        //log_debug_opt(to_string(grad_x)+" "+to_string(grad_y)+"\n")

        Corner_stitching::TILE *pad_tile=CS.TILE_map[NAM(pad)];
        string pad_name=P_NAM(pad_tile);
        CS.process_TILE=pad_tile;

        FPOS new_lb={0.0, 0.0},
             new_ur={0.0, 0.0};

        new_lb={X(COR(pad_tile))+grad_x, Y(COR(pad_tile))+grad_y};
        new_ur={X(COR(pad_tile))+X(DIM(pad_tile))+grad_x, 
                Y(COR(pad_tile))+Y(DIM(pad_tile))+grad_y};

        if(Y(new_ur) > Y(P.die_ur) || X(new_ur) > X(P.die_ur) || Y(new_lb) < Y(P.die_lb) || X(new_lb) < X(P.die_lb)){
            reject++;
            continue;
        }
        if(!CS.TILE_check_overlap(new_lb, new_ur)){
            reject++;
            continue;
        } 

        CS.TILE_deletion(pad_tile, 0);
        CS.TILE_creation(new_lb, new_ur, pad_name, 0);

        LB(pad)=new_lb,
        UR(pad)=new_ur;
        X(CT(pad))+=grad_x,
        Y(CT(pad))+=grad_y;

        for(int j=0; j<pin_change_before.size(); j++){
            NET *net=P.net_vec[KE(pin_change_before[j])];
            float pin_x=X(VL(pin_change_before[j])),
                  pin_y=Y(VL(pin_change_before[j]));

            float new_pin_x=pin_x+grad_x,
                  new_pin_y=pin_y+grad_y;

            P.lse-=500*(log(X(net->ExpSum_P))+
                        log(Y(net->ExpSum_P))+
                        log(X(net->ExpSum_N))+
                        log(Y(net->ExpSum_N)));
            X(net->ExpSum_P)+=(exp((new_pin_x)*0.002)-exp(pin_x*0.002)),
            Y(net->ExpSum_P)+=(exp((new_pin_y)*0.002)-exp(pin_y*0.002)),
            X(net->ExpSum_N)+=(exp(-(new_pin_x)*0.002)-exp(-pin_x*0.002)),
            Y(net->ExpSum_N)+=(exp(-(new_pin_y)*0.002)-exp(-pin_y*0.002));
            P.lse+=500*(log(X(net->ExpSum_P))+
                        log(Y(net->ExpSum_P))+
                        log(X(net->ExpSum_N))+
                        log(Y(net->ExpSum_N)));
        }
    }
    return reject;
}

float wl_opt::cal_hpwl(class Parser_file &P, class Corner_stitching &CS){
    float hpwl = 0.0;
    for(pair<string, NET *> elem : P.net_vec){
        NET *net=VL(elem);
        float min_x=1e9,
              min_y=1e9,
              max_x=0.0,
              max_y=0.0;
        float expsum_x=0.0,
              expsum_y=0.0,
              expsum_x_n=0.0,
              expsum_y_n=0.0;

        for(int j=0; j<PS(net).size(); j++){
            PIN *pin=PS(net)[j];
            PAD *pad=P(pin);
            Corner_stitching::TILE *pad_tile=CS.TILE_map[NAM(pad)];
            float pin_x=X(COR(pad_tile))+X(DIM(pad_tile))/2+X(OF(pin)),
                  pin_y=Y(COR(pad_tile))+Y(DIM(pad_tile))/2+Y(OF(pin));
            min_x = (min_x>pin_x) ? pin_x : min_x;
            min_y = (min_y>pin_y) ? pin_y : min_y; 
            max_x = (max_x<pin_x) ? pin_x : max_x;
            max_y = (max_y<pin_y) ? pin_y : max_y;   

            expsum_x   += exp(pin_x*0.002);
            expsum_y   += exp(pin_y*0.002);
            expsum_x_n += exp(-pin_x*0.002);
            expsum_y_n += exp(-pin_y*0.002);
        }
        net->pin_lb   = {min_x, min_y},
        net->pin_ur   = {max_x, max_y},
        net->ExpSum_P = {expsum_x,   expsum_y};
        net->ExpSum_N = {expsum_x_n, expsum_y_n};
        net->WL = (max_x - min_x) + (max_y - min_y);
        hpwl += net->WL;
    }
    return hpwl;
}

void log_debug_opt(const std::string &message) {
    std::ofstream log_file("debug_log.txt", std::ios::app);
    log_file << message << std::endl;
    log_file.close();
}

void  wl_opt::CG_optimize(class Parser_file &P, class Corner_stitching &CS, bool refresh){

    if(refresh == true){
        CS.TILE_map.clear();
        CS.SPACE_map.clear();
        CS.b_num = 1, CS.s_num = 1;
        CS.pad_insertion(P.pad_vec);
    }

    cal_hpwl(P, CS);
    fill_pq(P);

    float move_para = (refresh == true) ? 1.0 : 0.1;
    int   reject = 0;
    for(int i=0; i<300; i++){
        if(reject>opt_pq.size()*0.5)
            move_para=move_para*0.9;
        if(reject>opt_pq.size()*0.9)
            break;
        reject=pad_refinement(P, CS, move_para, i);
    }
    return;
}



