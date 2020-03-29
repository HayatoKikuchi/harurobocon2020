//-----------------------------------------
// 軌道追従や位置のPID制御を行うためのクラス
// 作成：2019/05/15 by Yuki Ueno
//-----------------------------------------

#include "MotionGenerator.h"
#include "PIDclass.h"
#include "Filter.h"
#include "define.h"

PID posiPIDx(2.5, 0.0, 5.0, INT_TIME);
PID posiPIDy(3.0, 0.0, 2.0, INT_TIME);
PID posiPIDz(4.0, 0.0, 0.0, INT_TIME);

PID yokozurePID(3.0, 0.0, 1.5, INT_TIME);//(3.0, 0.0, 0.0, INT_TIME);
PID kakudoPID(5.0, 2.5, 0.0, INT_TIME);

// 二次遅れ使えるようになる
Filter sokduo_filter(INT_TIME);
Filter kakudo_filter(INT_TIME);

// コンストラクタ
MotionGenerator::MotionGenerator(int xmode){
    path_num = 0;
    max_pathnum = 0;
    t_be = 0.0;
    pre_t_be = 0.1;
    epsilon = 1.0;

    refVx = 0;
    refVy = 0;
    refVz = 0;

    mode = xmode;

    // PID関連初期化
	posiPIDx.PIDinit(0.0, 0.0);
	posiPIDy.PIDinit(0.0, 0.0);
	posiPIDz.PIDinit(0.0, 0.0);
	
	yokozurePID.PIDinit(0.0, 0.0);
	kakudoPID.PIDinit(0.0, 0.0);

	sokduo_filter.setSecondOrderPara(22.0, 1.0, 0.0);//(15.0, 1.0, 0.0);
    kakudo_filter.setSecondOrderPara(10.0, 1.0, 0.0);//(7.0, 1.0, 0.0);
   // angle = 2.35619;

    mode_changed = true;
    init_done = false;
}

// tを求めるための方程式
double MotionGenerator::func(int p, double t)
{
    return a_be[p] * pow(t, 5.0) + b_be[p] * pow(t,4.0) + c_be[p] * pow(t,3.0) + d_be[p] * pow(t,2.0) + e_be[p] * t + f_be[p];
}
// tを求めるための方程式の1階微分
double MotionGenerator::dfunc(int p, double t)
{
    return 5.0 * a_be[p] * pow(t, 4.0) +  4.0 * b_be[p] * pow(t,3.0) + 3.0 * c_be[p] * pow(t,2.0) + 2.0 * d_be[p] * t + e_be[p];
}

// tにおけるベジエ曲線の座標を求める関数
double MotionGenerator::bezier_x(int p, double t)
{
    return Ax[p]*pow(t,3.0) + 3.0*Bx[p]*pow(t,2.0) + 3.0*Cx[p]*t + Dx[p];
}
double MotionGenerator::bezier_y(int p, double t)
{
    return Ay[p]*pow(t,3.0) + 3.0*By[p]*pow(t,2.0) + 3.0*Cy[p]*t + Dy[p];
}

// ベジエ曲線式の1階微分
double MotionGenerator::dbezier_x(int p, double t)
{
    return 3.0*Ax[p]*pow(t,2.0) + 6.0*Bx[p]*t + 3.0*Cx[p];
}
double MotionGenerator::dbezier_y(int p, double t)
{
    return 3.0*Ay[p]*pow(t,2.0) + 6.0*By[p]*t + 3.0*Cy[p];
}

// ニュートン法のための係数の初期化
void MotionGenerator::initSettings(){
    for(int i = 0; i < PATHNUM; i++) {
        Ax[i] = Px[3*i+3] -3*Px[3*i+2] + 3*Px[3*i+1] - Px[3*i+0];
        Ay[i] = Py[3*i+3] -3*Py[3*i+2] + 3*Py[3*i+1] - Py[3*i+0];
        Bx[i] = Px[3*i+2] -2*Px[3*i+1] + Px[3*i+0];
        By[i] = Py[3*i+2] -2*Py[3*i+1] + Py[3*i+0];
        Cx[i] = Px[3*i+1] - Px[3*i+0];
        Cy[i] = Py[3*i+1] - Py[3*i+0];
        Dx[i] = Px[3*i+0];
        Dy[i] = Py[3*i+0];
    }

    for(int i = 0; i < PATHNUM; i++) {
        a_be[i] = pow(Ax[i], 2.0) + pow(Ay[i], 2.0);
        b_be[i] = 5*(Ax[i]*Bx[i] + Ay[i]*By[i]);
        c_be[i] = 2*((3*pow(Bx[i],2.0)+2*Ax[i]*Cx[i]) + (3*pow(By[i],2.0)+2*Ay[i]*Cy[i]));
        d_be_[i] = 9*Bx[i]*Cx[i] + 9*By[i]*Cy[i];
        e_be_[i] = 3*pow(Cx[i],2.0) + 3*pow(Cy[i],2.0);
        f_be_[i] = 0;
    }
    init_done = true;
}

// ベジエ曲線までの垂線距離をニュートン法で求めて，そこまでの距離と接線角度を計算する
void MotionGenerator::calcRefpoint(double Posix, double Posiy){
    if(init_done){
        double tmpx = Px[path_num * 3] - Posix;
        double tmpy = Py[path_num * 3] - Posiy;
                
        d_be[path_num] = d_be_[path_num] + Ax[path_num] * tmpx + Ay[path_num] * tmpy;
        e_be[path_num] = e_be_[path_num] + 2*Bx[path_num] * tmpx + 2*By[path_num] * tmpy;
        f_be[path_num] = f_be_[path_num] + Cx[path_num] * tmpx + Cy[path_num] * tmpy;
                
        int count_newton = 0;
        do {
            t_be = pre_t_be - func(path_num, pre_t_be)/dfunc(path_num, pre_t_be);
            epsilon = fabs((t_be - pre_t_be)/pre_t_be);
            
            pre_t_be = t_be;
            count_newton++;
        }while(epsilon >= 1e-4 && count_newton <= 50);
        
        //double onx, ony;    //ベジエ曲線上の点
        onx = bezier_x(path_num, t_be);
        ony = bezier_y(path_num, t_be);
                
        // 外積による距離導出
        //double angle;
        angle = atan2(dbezier_y(path_num, t_be), dbezier_x(path_num, t_be)); // ベジエ曲線の接線方向
        if(fabs(angle - preAngle) > PI){
            angle += 2 * PI;
        }
        //double dist;
        dist = (ony - Posiy)*cos(angle) - (onx - Posix)*sin(angle);

        epsilon = 1.0;
    }
}

// モードによって，それぞれ指令速度を計算する
int MotionGenerator::calcRefvel(double Posix, double Posiy, double Posiz){
    double refVxg, refVyg, refVzg; // グローバル座標系の指定速度
    double tmpPx, tmpPy;
    static int counter = 0;

    if(init_done){
        if(path_num <= max_pathnum){ // パスが存在する場合は以下の処理を行う
            if(mode == FOLLOW_TANGENT || mode == FOLLOW_COMMAND){ // ベジエ曲線追従モード
                calcRefpoint(Posix, Posiy);

                double refVtan, refVper, refVrot;
                if((acc_mode[path_num] == MODE_START || acc_mode[path_num] == MODE_START_STOP) && counter <= acc_count[path_num]){
                    counter++;
                    refVtan = refvel[path_num] * counter/(double)acc_count[path_num];
                }else if(t_be < dec_tbe[path_num]){
                    refVtan = refvel[path_num];
                }else if((acc_mode[path_num] == MODE_STOP || acc_mode[path_num] == MODE_START_STOP) && t_be >= dec_tbe[path_num]){
                    refVtan = refvel[path_num] - refvel[path_num] * (t_be - dec_tbe[path_num]) / (1.0 - dec_tbe[path_num]);
                }else if(t_be >= dec_tbe[path_num]){
                    refVtan = refvel[path_num] - (refvel[path_num] - refvel[path_num + 1]) * (t_be - dec_tbe[path_num]) / (1.0 - dec_tbe[path_num]);
                }

                //refVtan = sokduo_filter.SecondOrderLag(refvel[path_num]); // 接線方向速度
                refVper = yokozurePID.getCmd(dist, 0.0, refvel[path_num] * 2.0); // 横方向速度

                // 旋回は以下の2種類を mode によって変える
                if(mode == FOLLOW_TANGENT){
                    if(mode_changed){
                        kakudoPID.PIDinit(angle, Posiz);
                        mode_changed = false;
                    }
                    refVrot = kakudoPID.getCmd(angle, Posiz, 1.57);//(refKakudo, gPosiz, 1.57);
                }else{
                    if(mode_changed){
                        kakudoPID.PIDinit(refKakudo, Posiz);
                        kakudo_filter.initPrevData(refKakudo);
                        mode_changed = false;
                    }
                    refKakudo = kakudo_filter.SecondOrderLag(refangle[path_num]);
                    refVrot = kakudoPID.getCmd(refKakudo, Posiz, 1.57);
                }

                per = refVper;
                rot = refVrot;
                // ローカル座標系の指令速度(グローバル座標系のも込み込み)
                //refVxとrefVyをコメントアウトするとkakudoPIDのパラメータ調整が出来る(手で押してみて…)
                //refVperだけにするとyokozurePIDのパラメータ調整できる
                refVx =  refVtan * cos( Posiz - angle ) + refVper * sin( Posiz - angle );
                refVy = -refVtan * sin( Posiz - angle ) + refVper * cos( Posiz - angle );
                refVz =  refVrot;
            }else{ // PID位置制御モード
                if( mode_changed ){
                    Px[3 * path_num] = Posix;
                    Py[3 * path_num] = Posiy;
                    posiPIDx.PIDinit(Px[3 * path_num], Posix);	// ref, act
                    posiPIDy.PIDinit(Py[3 * path_num], Posiy);
                    posiPIDz.PIDinit(refangle[path_num], Posiz);
                    kakudo_filter.initPrevData(refKakudo);
                    setRefKakudo();
                    mode_changed = false;
                }

                if(counter <= acc_count[path_num]){
                    counter++;
                    tmpPx = Px[3 * path_num] + (Px[3 * path_num + 3] - Px[3 * path_num]) * counter / (double)acc_count[path_num];
                    tmpPy = Py[3 * path_num] + (Py[3 * path_num + 3] - Py[3 * path_num]) * counter / (double)acc_count[path_num];
                }else{
                    tmpPx = (Px[3 * path_num + 3]);
                    tmpPy = (Py[3 * path_num + 3]);
                }

                // PIDクラスを使って位置制御を行う(速度の指令地を得る)
                refVxg = posiPIDx.getCmd(tmpPx, Posix, refvel[path_num]);//(Px[30], gPosix, refvel[phase]);
                refVyg = posiPIDy.getCmd(tmpPy, Posiy, refvel[path_num]);//(Py[30], gPosiy, refvel[phase]);
                refKakudo = kakudo_filter.SecondOrderLag(refangle[path_num]);
                refVzg = posiPIDz.getCmd(refangle[path_num], Posiz, 1.57);//角速度に対してrefvelは遅すぎるから　refvel[path_num]);//(0.0, gPosiz, refvel[phase]);

                // 上記はグローバル座標系における速度のため，ローカルに変換
                refVx =  refVxg * cos(Posiz) + refVyg * sin(Posiz);
                refVy = -refVxg * sin(Posiz) + refVyg * cos(Posiz);
                refVz =  refVzg;
            }

            // 収束判定して，収束していたら　1　を返す
            dist2goal = sqrt(pow(Posix - Px[3 * path_num + 3], 2.0) + pow(Posiy - Py[3 * path_num + 3], 2.0));
            if(mode == FOLLOW_TANGENT || mode == FOLLOW_COMMAND){
                // 軌道追従制御なら，到達位置からの距離とベジエ曲線の t のどちらかの条件
                if(dist2goal <= conv_length || t_be >= conv_tnum){
                    counter = 0;
                    return 1;
                }
            }if(mode == POSITION_PID){
                // 位置制御なら，目標位置と角度両方を見る
                if(dist2goal <= conv_length && fabs(refangle[path_num] - Posiz)){
                    counter = 0;
                    return 1;
                }
            }
            
            // 収束していなかったら　0　を返す
            return 0;
        }else{
            // path_num が設定されたmax_pathnumを超えたら　-2　を返す
            return -2;
        }
    
    }else{
        // 初期化されていなかったら　-1　を返す
        return -1;
    }
}

// ベジエ曲線のパス番号をインクリメントする
void MotionGenerator::incrPathnum(double xconv_length, double xconv_tnum = 0.997){
    path_num++;
    pre_t_be = 0.1;
    setConvPara(xconv_length, xconv_tnum);
}

int MotionGenerator::getPathNum(){
    return path_num;
}

void MotionGenerator::setPathNum(int num){
    path_num = num;
}

// 収束判定に用いる距離などをセットする
void MotionGenerator::setConvPara(double xconv_length, double xconv_tnum = 0.997){
    conv_length = xconv_length;
    conv_tnum = xconv_tnum;
}

void MotionGenerator::setMode(int xmode){
    mode = xmode;
    mode_changed = true;
}

int MotionGenerator::getMode(){
    return mode;
}

void MotionGenerator::setMaxPathnum(int num){
    max_pathnum = num;
}

void MotionGenerator::setPosiPIDxPara(float xKp, float xKi, float xKd){
    posiPIDx.setPara(xKp, xKi, xKd);
}

void MotionGenerator::setPosiPIDyPara(float xKp, float xKi, float xKd){
    posiPIDy.setPara(xKp, xKi, xKd);
}

void MotionGenerator::setPosiPIDzPara(float xKp, float xKi, float xKd){
    posiPIDz.setPara(xKp, xKi, xKd);
}

void MotionGenerator::setYokozurePIDPara(float xKp, float xKi, float xKd){
    yokozurePID.setPara(xKp, xKi, xKd);
}

void MotionGenerator::setKakudoPIDPara(float xKp, float xKi, float xKd){
    kakudoPID.setPara(xKp, xKi, xKd);
}


void MotionGenerator::kakudoPIDinit(double Posiz){
    kakudoPID.PIDinit(refKakudo, Posiz);
}

void MotionGenerator::setRefKakudo(){
    refKakudo = refangle[path_num];
}

double MotionGenerator::getRefVper(){
    return per;
}

double MotionGenerator::getRefVrot(){
    return rot;
}