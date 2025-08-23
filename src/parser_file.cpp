#include <bits/stdc++.h>
#include <sys/stat.h>
#include <errno.h>

#include "parser_file.h"
#include "datatype.h"
#include "Corner_stitching.h"

using namespace std;

void Parser_file::parser_aux(string file_path){
    string aux,
           tempt,
           nodes,
           nets,
           wts,
           pl,
           scl;

    size_t found = file_path.find_last_of("/");
    
    aux = file_path.substr(found+1);
    
    found = aux.find_last_of(".");
    File_name = aux.substr(0, found);
    
    string relative_path = "../benchmarks/" + File_name + "/";
    aux = relative_path + aux;

    ifstream ifs(aux, ios::in);
    ifs>>tempt>>tempt;
    ifs>>nodes
       >>nets
       >>wts
       >>pl
       >>scl;


    nodes = relative_path + nodes,
    nets  = relative_path + nets,
    wts   = relative_path + wts,
    pl    = relative_path + pl,
    scl   = relative_path + scl;

    parser_nodes(nodes);
    cout<<"parser_nodes........\n";
    parser_pl(pl);
    cout<<"parser_pl...........\n";
    parser_nets(nets);
    cout<<"parser_nets.........\n";

    return;
}

void Parser_file::parser_nodes(string file_name){
    string tempt;
    float  num_1,
           num_2,
           idx;

    ifstream ifs(file_name, ios::in);
    getline(ifs, tempt, '\n'),
    getline(ifs, tempt, '\n'),
    getline(ifs, tempt, '\n');
    ifs>>tempt>>tempt>>cell_num;
    ifs>>tempt>>tempt>>pad_num;
    cell_num -= pad_num;
    for(int i=0; i<cell_num; i++){
        ifs>>tempt>>num_1>>num_2;

        MODULE* M=new MODULE;
        NAM(M)=tempt;
        W(O_WH(M))=num_1;
        H(O_WH(M))=num_2;
        A(M)=num_1 * num_2;

        cell_vec[NAM(M)]=M;
        cell_pad[NAM(M)]=0;
    }
    for(int i=0; i<pad_num; i++){
        ifs>>tempt>>num_1>>num_2;

        PAD* P=new PAD;
        NAM(P)=tempt;
        W(WH(P))=num_1;
        H(WH(P))=num_2;
        A(P)=num_1 * num_2;

        ifs>>tempt;
        if(tempt!="terminal")
            P->is_cover=1;

        pad_vec[NAM(P)]=P;
        cell_pad[NAM(P)]=1;
    }
    ifs.close();
    return;
}

void Parser_file::parser_nets(string file_name){
    string tempt_1,
           tempt_2,
           tempt_3; 
    float  num_1,
           num_2,
           num_3;

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

    ifstream ifs(file_name, ios::in);
    getline(ifs, tempt_1, '\n'),
    getline(ifs, tempt_1, '\n'),
    getline(ifs, tempt_1, '\n');

    ifs>>tempt_1
       >>tempt_1
       >>net_num;
    getline(ifs, tempt_1, '\n');
    getline(ifs, tempt_1, '\n');

    queue<PIN *> pins;

    for(int i=0; i<net_num; i++){
        ifs>>tempt_1
           >>tempt_1
           >>num_1
           >>tempt_1;

        float max_x = 0.0,
              min_x = 1e9,
              max_y = 0.0,
              min_y = 1e9;


        float expsum_x=0.0,
              expsum_y=0.0,
              expsum_x_n=0.0,
              expsum_y_n=0.0;

        for(int j=0; j<num_1; j++){
            ifs>>tempt_2
               >>tempt_3
               >>tempt_3
               >>num_2
               >>num_3;

            if(cell_pad[tempt_2]==0) continue;

            PAD *pad = pad_vec[tempt_2];
            PIN *pin = new PIN();

            int ort = ORT(pad);
            float off_x = num_2,
                  off_y = num_3;

            cal_offset(off_x, off_y, ort);

            X(OF(pin))=off_x;
            Y(OF(pin))=off_y;

            P(pin) = pad;

            pins.push(pin);

            if(F(pad)==0) continue;

            float pin_x = X(CT(pad)) + off_x;
            float pin_y = Y(CT(pad)) + off_y;

            max_x = (pin_x > max_x) ? pin_x : max_x;
            min_x = (pin_x < min_x) ? pin_x : min_x;
            max_y = (pin_y > max_y) ? pin_y : max_y;
            min_y = (pin_y < min_y) ? pin_y : min_y;

            expsum_x   += exp(pin_x*0.002);
            expsum_y   += exp(pin_y*0.002);
            expsum_x_n += exp(-pin_x*0.002);
            expsum_y_n += exp(-pin_y*0.002);
        }

        if(pins.size()>1){
            NET *net = new NET();
            NAM(net)=tempt_1;
            net->pin_lb={min_x, min_y};
            net->pin_ur={max_x, max_y};
            net->pin_lb_ori={min_x, min_y};
            net->pin_ur_ori={max_x, max_y};

            net->net_weight=1.0/pins.size();
            net->ExpSum_P={expsum_x,   expsum_y};
            net->ExpSum_N={expsum_x_n, expsum_y_n};
            net_vec[NAM(net)] = net;

            PIN *pin_ref=pins.front();
            while(pins.size()){
                PIN *pin = pins.front();
                pins.pop();

                if(NAM(P(pin_ref))!=NAM(P(pin))) {
                    net->same_pad=false;
                }
                if(!F(P(pin))){
                    net->fix_net=false;
                }

                N(pin) = net;

                PS(net).push_back(pin);

                PS(P(pin)).push_back(pin);
            }
        } else{
            if(pins.size()){
                PIN *tempt_pin = pins.front();
                pins.pop();
                delete tempt_pin;
            }
        }
    }
    ifs.close();
    return;
}

void Parser_file::parser_pl(string file_name){
    string tempt_1,
           tempt_2,
           tempt_3; 
    char   test;
    float  num_1,
           num_2;
    bool   lay;

    MODULE* cell;
    PAD*    pad;

    function<void(float &, float &)> swap_wh = [](float &w, float &h){
        float tempt=w;
        w=h;
        h=tempt;
    };

    function<void(MODULE *)> calculate_loc_module = [](MODULE* cell){
        X(CT(cell))=X(LB(cell))+W(O_WH(cell))/2;
        Y(CT(cell))=Y(LB(cell))+H(O_WH(cell))/2;
        X(UR(cell))=X(LB(cell))+W(O_WH(cell));
        Y(UR(cell))=Y(LB(cell))+H(O_WH(cell));
    };

    function<void(PAD *)> calculate_loc_pad = [](PAD* pad){
        X(CT(pad))=X(LB(pad))+W(WH(pad))/2;
        Y(CT(pad))=Y(LB(pad))+H(WH(pad))/2;
        X(UR(pad))=X(LB(pad))+W(WH(pad));
        Y(UR(pad))=Y(LB(pad))+H(WH(pad));
    };

    function<int(string)> orientation_assign = [](string o)->int{
        if(o=="N")       return 0;
        else if(o=="W")  return 1;
        else if(o=="S")  return 2;
        else if(o=="E")  return 3;
        else if(o=="FN") return 4;
        else if(o=="FW") return 5;
        else if(o=="FS") return 6;
        else             return 7;
    };
    
    ifstream ifs(file_name, ios::in);

    for(int i=0; i<cell_num+pad_num; i++){
        ifs>>tempt_1
           >>num_1
           >>num_2
           >>tempt_2
           >>tempt_3;

        if(tempt_3=="N" || tempt_3=="S" || tempt_3=="FN" || tempt_3=="FS"){
            lay=0;
        } else{
            lay=1;
        }

        if(tempt_3!="N") cout<<"***\n";

        test=ifs.peek();
        if(cell_pad[tempt_1]==0){
            cell=cell_vec[tempt_1];
            X(LB(cell))=num_1,
            Y(LB(cell))=num_2;
            if(lay==1){
                swap_wh(W(O_WH(cell)), H(O_WH(cell)));
            }
            calculate_loc_module(cell);

            X(die_ur) = (X(die_ur) < X(UR(cell))) ? X(UR(cell)) : X(die_ur);
            Y(die_ur) = (Y(die_ur) < Y(UR(cell))) ? Y(UR(cell)) : Y(die_ur);

            ORT(cell)=orientation_assign(tempt_3);
        } else{
            pad=pad_vec[tempt_1];
            X(LB(pad))=num_1,
            Y(LB(pad))=num_2;
            if(lay==1){
                swap_wh(W(WH(pad)), H(WH(pad)));
            }
            calculate_loc_pad(pad);

            X(die_ur) = (X(die_ur) < X(UR(pad))) ? X(UR(pad)) : X(die_ur);
            Y(die_ur) = (Y(die_ur) < Y(UR(pad))) ? Y(UR(pad)) : Y(die_ur);

            ORT(pad)=orientation_assign(tempt_3);
            getline(ifs, tempt_1, '\n');
            if(tempt_1.size()>5){
                F(pad)=true;
                fixed_pad_vec[NAM(pad)] = pad;
            } else{
                movable_pad_vec.push_back(NAM(pad));
            }
        }
        
    }
    ifs.close();
    return;
}

void Parser_file::out_pad_dpx(string file_name){
    ofstream ofs(file_name, ios::out);

    function<void(const FPOS &, const FPOS &)> output_corner = [&ofs](const FPOS &lb, const FPOS &ur){

        ofs<<"SR  ";
        ofs << std::fixed << std::setprecision(3) << X(lb) << " " << -Y(lb) << " " << X(ur) << " " << -Y(ur) << endl ;
        
    };
    ofs<<"COLOR black"<<endl;
    
    for(pair<string, PAD*> elem : pad_vec){
        class FPOS lb=LB(VL(elem)),
                   ur=UR(VL(elem));
        
        if(F(VL(elem))==0){
            ofs<<"COLOR green"<<endl;
        } else{
            ofs<<"COLOR red"<<endl;
        }

        output_corner(lb, ur);
        ofs<<endl;
    }
    ofs.close();
    return;
}

void Parser_file::out_matlab(class Corner_stitching &CS, string file_name){
    ofstream ofs("result.m", ios::out);
    ofs<<"axis equal;\n";
    ofs<<"hold on;\n";
    ofs<<"grid on;\n";
    for(auto it=CS.TILE_map.begin(); it!=CS.TILE_map.end();it++){
        ofs<<"block_x=[ "
        <<X(COR(it->second))<<" "
        <<X(COR(it->second))+W(DIM(it->second))<<" "
        <<X(COR(it->second))+W(DIM(it->second))<<" "
        <<X(COR(it->second))<<" "
        <<X(COR(it->second))<<" ];\n";
        ofs<<"block_y=[ "
        <<Y(COR(it->second))<<" "
        <<Y(COR(it->second))<<" "
        <<Y(COR(it->second))+H(DIM(it->second))<<" "
        <<Y(COR(it->second))+H(DIM(it->second))<<" "
        <<Y(COR(it->second))<<" ];\n";
        if(NAM(it->second)[0]=='s')
            ofs<<"fill(block_x,block_y,'white');\n";
        else{
            if(F(pad_vec[P_NAM(it->second)])==0)
                ofs<<"fill(block_x,block_y,'blue');\n";
            else
                ofs<<"fill(block_x,block_y,'red');\n";
        }
        //ofs<<"text ("<<X(COR(it->second))+X(DIM(it->second))/2<<","<<Y(COR(it->second))+Y(DIM(it->second))/2<<", '"<<it->first<<"');\n\n\n"; 
    }
}

void Parser_file::out_matlab_overlap(class Corner_stitching &CS, string file_name){
    ofstream ofs(file_name, ios::out);
    ofs<<"axis equal;\n";
    ofs<<"hold on;\n";
    ofs<<"grid on;\n";
    for(auto it=CS.TILE_map.begin(); it!=CS.TILE_map.end();it++){
        
        ofs<<"plot(["<<X(COR(it->second))<<", "<<X(COR(it->second))+X(DIM(it->second))<<"], [";
        ofs<<Y(COR(it->second))<<", "<<Y(COR(it->second))<<"], ";
        ofs<<"'Color', [0, 1, 0], 'LineWidth', 1);\n";

        ofs<<"plot(["<<X(COR(it->second))<<", "<<X(COR(it->second))<<"], [";
        ofs<<Y(COR(it->second))<<", "<<Y(COR(it->second))+Y(DIM(it->second))<<"], ";
        ofs<<"'Color', [0, 1, 0], 'LineWidth', 1);\n";

        ofs<<"plot(["<<X(COR(it->second))<<", "<<X(COR(it->second))+X(DIM(it->second))<<"], [";
        ofs<<Y(COR(it->second))+Y(DIM(it->second))<<", "<<Y(COR(it->second))+Y(DIM(it->second))<<"], ";
        ofs<<"'Color', [0, 1, 0], 'LineWidth', 1);\n";

        ofs<<"plot(["<<X(COR(it->second))+X(DIM(it->second))<<", "<<X(COR(it->second))+X(DIM(it->second))<<"], [";
        ofs<<Y(COR(it->second))<<", "<<Y(COR(it->second))+Y(DIM(it->second))<<"], ";
        ofs<<"'Color', [0, 1, 0], 'LineWidth', 1);\n";
        
        ofs<<"\n";
    }
}

void Parser_file::out_matlab_detection(class Corner_stitching &CS, string file_name, string tile){
    ofstream ofs(file_name, ios::out);
    ofs<<"axis equal;\n";
    ofs<<"hold on;\n";
    ofs<<"grid on;\n";
    for(auto it=CS.TILE_map.begin(); it!=CS.TILE_map.end();it++){
        
        ofs<<"block_x=[ "
        <<X(COR(it->second))<<" "
        <<X(COR(it->second))+W(DIM(it->second))<<" "
        <<X(COR(it->second))+W(DIM(it->second))<<" "
        <<X(COR(it->second))<<" "
        <<X(COR(it->second))<<" ];\n";
        ofs<<"block_y=[ "
        <<Y(COR(it->second))<<" "
        <<Y(COR(it->second))<<" "
        <<Y(COR(it->second))+H(DIM(it->second))<<" "
        <<Y(COR(it->second))+H(DIM(it->second))<<" "
        <<Y(COR(it->second))<<" ];\n";
        if(NAM(it->second)[0]=='s')
            ofs<<"fill(block_x,block_y,'white');\n";
        else{
            if(P_NAM(it->second)==tile)
                ofs<<"fill(block_x,block_y,'red');\n";
            else
                ofs<<"fill(block_x,block_y,'blue');\n";
        }
    }
}

void Parser_file::out_pad_matlab(string file_name){
    ofstream ofs(file_name, ios::out);
    ofs<<"axis equal;\n";
    ofs<<"hold on;\n";
    ofs<<"grid on;\n";
    for(auto it=pad_vec.begin(); it!=pad_vec.end(); it++){
        
        ofs<<"block_x=[ "
        <<X(LB(it->second))<<" "
        <<X(UR(it->second))<<" "
        <<X(UR(it->second))<<" "
        <<X(LB(it->second))<<" "
        <<X(LB(it->second))<<" ];\n";
        ofs<<"block_y=[ "
        <<Y(LB(it->second))<<" "
        <<Y(LB(it->second))<<" "
        <<Y(UR(it->second))<<" "
        <<Y(UR(it->second))<<" "
        <<Y(LB(it->second))<<" ];\n";
        if(NAM(it->second)[0]=='s')
            ofs<<"fill(block_x,block_y,'white');\n";
        else{
            ofs<<"fill(block_x,block_y,'blue');\n";
        }       
    }
    //plot([x1, x2], [y1, y2], 'Color', [1, 0, 0], 'LineWidth', 2);
    // for(auto it=net_vec.begin(); it!=net_vec.end(); it++){
    //     if(PS(it->second).size()>13 && it->second->fix_net==0){
    //         //cout<<NAM(it->second)<<", "<<PS(it->second).size()<<endl;
    //         for(int i=1; i<PS(it->second).size(); i++){
    //             ofs<<"plot(["<<X(CT(P(PS(it->second)[i-1])))<<", "<<X(CT(P(PS(it->second)[i])))<<"], [";
    //             ofs<<Y(CT(P(PS(it->second)[i-1])))<<", "<<Y(CT(P(PS(it->second)[i])))<<"], ";
    //             if(i%2==0)
    //                 ofs<<"'Color', [0, 1, 0], 'LineWidth', 1);\n";
    //             else
    //                 ofs<<"'Color', [1, 0, 0], 'LineWidth', 1);\n";
    //         }
    //     }
    // }
}

void Parser_file::parser_pl_out(string file_path, class Corner_stitching &CS, vector<pair<string, int>>& best_order){
    function<string(int)> orient_trans = [](int orient)->string{
        if(orient == 0)
            return "N";
        else if(orient == 1)
            return "W";
        else if(orient == 2)
            return "S";
        else if(orient == 3)
            return "E";
        else if(orient == 4)
            return "FN";
        else if(orient == 5)
            return "FW";
        else if(orient == 6)
            return "FS";
        else if(orient == 7)
            return "FE";
    }; 

    size_t found = file_path.find_last_of("/");
    string aux = file_path.substr(found+1);
    found = aux.find_last_of(".");
    string File_name = aux.substr(0, found);

    File_name = "benchmarks/" + File_name + "/" + File_name + ".pl_out";
    ofstream ofs(File_name, ios::out);
    
    for(auto it = fixed_pad_vec.begin(); it != fixed_pad_vec.end(); it++){
        string pad_name = it->first;
        Corner_stitching::TILE*  pad_tile = CS.TILE_map[pad_name];
        
        ofs<<P_NAM(pad_tile)<<" "<<X(COR(pad_tile))<<" "<<Y(COR(pad_tile))<<" : "<<"N"<<endl;
    }
    for(auto it = best_order.begin(); it != best_order.end(); it++){
        string pad_name = it->first;
        string orient   = orient_trans(it->second);
        Corner_stitching::TILE*  pad_tile = CS.TILE_map[pad_name];
        
        ofs<<P_NAM(pad_tile)<<" "<<X(COR(pad_tile))<<" "<<Y(COR(pad_tile))<<" : "<<orient<<endl;
    }
    ofs.close();
    return;
}

