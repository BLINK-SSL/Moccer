#pragma once
#include <iostream>
#include<vector>
#include<cmath>
#include<cstring>
#include<algorithm>

#include "../models/robot.h"

using namespace std;

#define Max_Range 120
#define Delta 0.1
#define Predict_Delta 1.0
#define Velocity_Accuracy 200
#define Angular_Velocity_Accuracy 0.5
#define One_Block 1.0
#define Safe_Distance 2
#define Alpha 1 //Obstacle
#define Beta (-1.0) //Goal
#define Gamma 1.0 //Velocity
#define INT_MAX 1000000000
#define Delta2 (-5) //Dist_To_D_Star

class Bot_Model {
public:
    double Max_Velocity;
    double Max_Angular_Velocity;
    double Max_Velocity_Acceleration;
    double Max_Angular_Acceleration;
};

class Pair {
public:
    double Target_Velocity;
    double Target_Angular_Velocity;
};

class Coordinate {
public:
    int x;
    int y;
};

class Node {
public:
    double Dist_To_Obstacle;
    double Dist_To_Goal;
    double Velocity;
    double Angular_Velocity;
    double VELOCITY;
    double ANGULAR_VELOCITY;
    double Dist_To_D_Star;
};

class DWAPlanner {
public:
    DWAPlanner() {
        bool Finish_Tag = false;
        int Open_List_Number = 0;
        bool In_Open_List[Max_Range][Max_Range] = {false};
        bool In_Close_List[Max_Range][Max_Range] = {false};
        int Dx[] = {1, 0, -1, 0, 1, -1, 1, -1};
        int Dy[] = {0, 1, 0, -1, 1, -1, -1, 1};
    }
    ~DWAPlanner() {
        
    }

    void Init(Bot_Model bot_model, Coordinate car_coordinate, Coordinate car_destination);
    void Set_Obstacle_Set(vector<Coordinate> obstacle_set);
    void Set_Grid_Map(bool map[Max_Range][Max_Range]);
    void Get_Answer(Pair &answer);

    double MIN(double a, double b) {
        return a > b ? b : a;
    }

    double MAX(double a, double b) {
        return a < b ? b : a;
    }

    double Calc_Dist(Coordinate a, Coordinate b) {
        return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
    }

    // bool Legal_Coordinate(Coordinate x) {
    //     std::cout<< "Checking coordinate: " << x.x << ", " << x.y << std::endl;
    //     if (x.x < 0 || x.x >= Max_Range || x.y < 0 || x.y >= Max_Range) return false;
    //     else return true;
    // }
    bool Legal_Coordinate(Coordinate x) {
        // std::cout << "Checking coordinate: " << x.x << ", " << x.y << std::endl;
        // if (x.x < -6000 || x.x > 6000 || x.y < -4500 || x.y > 4500)
        //     return false;
        // else
            return true;
    }

    bool Get_Trajectory(Coordinate Car_Coordinate, double Now_Velocity, double Now_Angle, double Now_Angular_Velocity) {
        double Car_x = One_Block * Car_Coordinate.x;
        double Car_y = One_Block * Car_Coordinate.y;
        double Time_Sum = 0;
        while (Time_Sum <= Predict_Delta) {
            Time_Sum += Delta;
            double Next_Angle = Now_Angle + Now_Angular_Velocity * Delta;
            Car_x += Now_Velocity * cos(Next_Angle * 0.017453292) * Delta;
            Car_y += Now_Velocity * sin(Next_Angle * 0.017453292) * Delta;
            Now_Angle = Next_Angle;
            if (Legal_Coordinate({lround(Car_x / One_Block), lround(Car_y / One_Block)})) {
                // std::cout << "Time: " << Time_Sum
                //           << ", Car position: " << lround(Car_x / One_Block) << ", " << lround(Car_y / One_Block) << std::endl;
                // std::cout << "Adding trajectory point: " << lround(Car_x / One_Block) << ", " << lround(Car_y / One_Block) << std::endl;
                Trajectory.push_back({lround(Car_x / One_Block), lround(Car_y / One_Block)});
            } else {
                return false;
            }
        }
        if (Trajectory.empty()) return false;
        return true;
    }

    double Get_Dist_To_Obstacle() {
        if (Obstacle_Set.empty()) {
            return -1;
        }
        double minn = 1e100;
        for (auto &i: Trajectory) {
            for (auto &j: Obstacle_Set) {
                double Dist = Calc_Dist(i, j);
                minn = MIN(minn, Dist);
            }
        }
        return minn;
    }

    double Get_Dist_To_Goal(Coordinate Car_Destination) {
        return Calc_Dist(Trajectory[Trajectory.size() - 1], Car_Destination);
    }

    int Get_H(Coordinate Left, Coordinate Right) {
        return abs(Left.x - Right.x) + abs(Left.y - Right.y);
    }

    bool A_Star_Judge_Legal(bool Map[Max_Range][Max_Range], int Now_x, int Now_y) {//A_Star加上?个
        for (int i = Now_x - Safe_Distance; i <= Now_x + Safe_Distance; i++) {
            for (int j = Now_y - Safe_Distance; j <= Now_y + Safe_Distance; j++) {
                if (i >= Max_Range || i < 0 || j >= Max_Range || j < 0) continue;
                if (Map[i][j]) {
                    return false;
                }
            }
        }
        return true;
    }


    static bool cmp(Coordinate a, Coordinate b) {
        // 比較ロジック
        return a.x < b.x; // 例
    }

    void SORT() {
        if (Open_List_Number <= 1) return;
        sort(Open_List, Open_List + Open_List_Number, cmp);
    }

    double Get_D_Star_Dist(std::vector<Coordinate> D_Star_Road, Coordinate End_Road) {
        double Dist = INT_MAX;
        for (auto &i: D_Star_Road) {
            double Temp = Get_H(End_Road, i);//和求h的估??通用，都是坐?距?
            if (Temp < Dist) {//取点到点最小距?作?点到?的距?
                Dist = Temp;
            }
        }
        // std::cout << "Distance: " << Dist << std::endl;
        return Dist;
    }

    void A_Star(bool Map[Max_Range][Max_Range], Coordinate Start, Coordinate End) {
        
        Open_List[Open_List_Number++] = Start;
        g_Score[Start.x][Start.y] = 0;
        h_Score[Start.x][Start.y] = Get_H(Start, End);
        f_Score[Start.x][Start.y] = g_Score[Start.x][Start.y] + h_Score[Start.x][Start.y];
        In_Open_List[Start.x][Start.y] = true;
        while (Open_List_Number) {
            if (In_Open_List[End.x][End.y]) {
                Finish_Tag = true;
                break;
            }
            Coordinate Now_Node = Open_List[Open_List_Number - 1];
            Open_List_Number--;//?除最小的点
            In_Open_List[Now_Node.x][Now_Node.y] = false;
            In_Close_List[Now_Node.x][Now_Node.y] = true;
            for (int i = 0; i < 7; i++) {
                double Add = 1;
                if (i > 3) Add = 1.4;//走斜?
                int Next_x = Now_Node.x + Dx[i];
                int Next_y = Now_Node.y + Dy[i];
                if (!A_Star_Judge_Legal(Map, Next_x, Next_y)) continue;//安全距?内有障碍物
                if (!Legal_Coordinate({Next_x, Next_y})) continue;//不合法
                if (In_Close_List[Next_x][Next_y]) continue;
                if (!In_Open_List[Next_x][Next_y]) {
                    Open_List[Open_List_Number++] = {Next_x, Next_y};
                    In_Open_List[Next_x][Next_y] = true;
                    Father[Next_x][Next_y] = Now_Node;
                    g_Score[Next_x][Next_y] = g_Score[Now_Node.x][Now_Node.y] + Add;
                    h_Score[Next_x][Next_y] = Get_H({Next_x, Next_y}, End);
                    f_Score[Next_x][Next_y] = g_Score[Next_x][Next_y] + h_Score[Next_x][Next_y];
                    SORT();
                } else {
                    if (g_Score[Next_x][Next_y] > g_Score[Now_Node.x][Now_Node.y] + Add) {
                        g_Score[Next_x][Next_y] = g_Score[Now_Node.x][Now_Node.y] + Add;
                        Father[Next_x][Next_y] = Now_Node;
                        f_Score[Next_x][Next_y] = g_Score[Next_x][Next_y] + h_Score[Next_x][Next_y];
                        SORT();
                    }
                }
            }
        }
        if (!Finish_Tag) {
            printf("Error,Can not reach to the destination!\n");
            return;
        }
        Coordinate Temp = End;
        A_Star_Road.push_back(Temp);
        while (Temp.x != Start.x || Temp.y != Start.y) {
            Temp = Father[Temp.x][Temp.y];
            A_Star_Road.push_back(Temp);
        }
    }

    void Refresh_Programme() {
        Obstacle_Set.clear();
        Ok_List.clear();
        Finish_Tag = false;
        Open_List_Number = 0;
        memset(g_Score, 0, sizeof(g_Score));
        memset(h_Score, 0, sizeof(h_Score));
        memset(f_Score, 0, sizeof(f_Score));
        memset(In_Open_List, false, sizeof(In_Open_List));
        memset(In_Close_List, false, sizeof(In_Close_List));
        A_Star_Road.clear();
    }

    Pair DWA(std::vector<Coordinate> D_Star_Road, Coordinate Car_Coordinate, double Now_Angle, double Now_Velocity,
             double Now_Angular_Velocity, Coordinate Car_Destination, Bot_Model Model, Robot* blueRobots, Robot* yellowRobots) {
        // std::cout << "coordinate: " << Car_Coordinate.x << ", " << Car_Coordinate.y << std::endl;
        Refresh_Programme();
        // for (auto &robot: blueRobots.begin()) {
        for (int i = 0; i < 11; ++i) {
            Obstacle_Set.push_back({static_cast<int>(yellowRobots[i].x / One_Block), static_cast<int>(yellowRobots[i].y / One_Block)});
        }
        double MIN_Dist_To_Obstacle = 1e100;
        double MAX_Dist_To_Obstacle = -10;
        double MIN_Dist_To_Goal = 1e100;
        double MAX_Dist_To_Goal = -10;
        double MIN_Velocity = 1e100;
        double MAX_Velocity = -1e100;
        double MIN_Angular_Velocity = 1e100;
        double MAX_Angular_Velocity = -1e100;
        double MIN_Dist_To_D_Star = 1e100;
        double MAX_Dist_To_D_Star = -10;
        for (double Velocity = Now_Velocity - Predict_Delta * Model.Max_Velocity_Acceleration;
             Velocity <=
             Now_Velocity + Predict_Delta * Model.Max_Velocity_Acceleration; Velocity += Velocity_Accuracy) {
            for (double Angular_Velocity = Now_Angular_Velocity - Predict_Delta * Model.Max_Angular_Acceleration;
                 Angular_Velocity <= Now_Angular_Velocity + Predict_Delta *
                                                            Model.Max_Angular_Acceleration; Angular_Velocity += Angular_Velocity_Accuracy) {
                if (fabs(Velocity) > Model.Max_Velocity || fabs(Angular_Velocity) > Model.Max_Angular_Velocity ||
                    Velocity < 0) {
                    // std::cout << "Velocity or Angular_Velocity out of range: "
                    //           << "Velocity=" << Velocity
                    //           << ", Angular_Velocity=" << Angular_Velocity
                    //           << std::endl;
                    continue;
                }
                Trajectory.clear();
                if (!Get_Trajectory(Car_Coordinate, Velocity, Now_Angle, Angular_Velocity)) continue;
                // std::cout << "Trajectory size: " << Trajectory.size() << std::endl;
                // for (auto &i: Trajectory) {
                //     std::cout << "Trajectory point: " << i.x << ", " << i.y << std::endl;
                // }
                double Break_Length = ((Velocity * Velocity / (2 * Model.Max_Velocity_Acceleration)) / One_Block);
                double Dist_To_Obstacle = Get_Dist_To_Obstacle();
                if (Dist_To_Obstacle != -1 && Dist_To_Obstacle < Safe_Distance) continue;
                if (Dist_To_Obstacle != -1 && Break_Length > Dist_To_Obstacle) continue;
                double Dist_To_Goal = Get_Dist_To_Goal(Car_Destination);
                // std::cout << "Dist to goal: " << Dist_To_Goal << std::endl;
                double Dist_To_D_Star = Get_D_Star_Dist(D_Star_Road, Trajectory[Trajectory.size() - 1]);
                MIN_Dist_To_Obstacle = MIN(Dist_To_Obstacle, MIN_Dist_To_Obstacle);
                MAX_Dist_To_Obstacle = MAX(Dist_To_Obstacle, MAX_Dist_To_Obstacle);
                MIN_Dist_To_Goal = MIN(Dist_To_Goal, MIN_Dist_To_Goal);
                MAX_Dist_To_Goal = MAX(Dist_To_Goal, MAX_Dist_To_Goal);
                MIN_Velocity = MIN(Velocity, MIN_Velocity);
                MAX_Velocity = MAX(Velocity, MAX_Velocity);
                MIN_Angular_Velocity = MIN(Angular_Velocity, MIN_Angular_Velocity);
                MAX_Angular_Velocity = MAX(Angular_Velocity, MAX_Angular_Velocity);
                MIN_Dist_To_D_Star = MIN(Dist_To_D_Star, MIN_Dist_To_D_Star);
                MAX_Dist_To_D_Star = MAX(Dist_To_D_Star, MAX_Dist_To_D_Star);
                Ok_List.push_back(
                        {Dist_To_Obstacle, Dist_To_Goal, Velocity, Angular_Velocity, Velocity, Angular_Velocity,
                         Dist_To_D_Star});
            }
        }
        for (auto &i: Ok_List) {
            i.Dist_To_Obstacle =
                    (i.Dist_To_Obstacle - MIN_Dist_To_Obstacle) / (MAX_Dist_To_Obstacle - MIN_Dist_To_Obstacle + 1e-8);
            i.Dist_To_Goal = (i.Dist_To_Goal - MIN_Dist_To_Goal) / (MAX_Dist_To_Goal - MIN_Dist_To_Goal + 1e-8);
            i.Velocity = (i.Velocity - MIN_Velocity) / (MAX_Velocity - MIN_Velocity + 1e-8);
            i.Angular_Velocity =
                    (i.Angular_Velocity - MIN_Angular_Velocity) / (MAX_Angular_Velocity - MIN_Angular_Velocity + 1e-8);
            i.Dist_To_D_Star =
                    (i.Dist_To_D_Star - MIN_Dist_To_D_Star) / (MAX_Dist_To_D_Star - MIN_Dist_To_D_Star + 1e-8);
        }
        double MAX_Score = -1e100;
        Pair Target = {0, 0};
        for (auto &i: Ok_List) {
            // std::cout << "Dist_To_Obstacle: " << i.Dist_To_Obstacle
            //           << ", Dist_To_Goal: " << i.Dist_To_Goal
            //           << ", Velocity: " << i.Velocity
            //           << ", Angular_Velocity: " << i.Angular_Velocity
            //           << ", Dist_To_D_Star: " << i.Dist_To_D_Star << std::endl;
            double Now_Score = 0;
//            Now_Score += Alpha * i.Dist_To_Obstacle;
            Now_Score += Beta * i.Dist_To_Goal;
            Now_Score += Gamma * i.Velocity;
            Now_Score += Delta2 * i.Dist_To_D_Star;
            if (Now_Score > MAX_Score) {
                MAX_Score = Now_Score;
                Target.Target_Angular_Velocity = i.ANGULAR_VELOCITY;
                Target.Target_Velocity = i.VELOCITY;
            }
        }
        if (Target.Target_Angular_Velocity == 0 && Target.Target_Velocity == 0) {
            //Ok list cout
            std::cout << Ok_List.size() << " valid trajectories found." << std::endl;
        }
        return Target;
    }

private:
    vector<Coordinate> Obstacle_Set;
    vector<Node> Ok_List;
    vector<Coordinate> Trajectory;

    bool Finish_Tag;
    int Open_List_Number;
    double g_Score[Max_Range][Max_Range];
    double h_Score[Max_Range][Max_Range];
    double f_Score[Max_Range][Max_Range];
    bool In_Open_List[Max_Range][Max_Range];
    bool In_Close_List[Max_Range][Max_Range];
    Coordinate Father[Max_Range][Max_Range];
    Coordinate Open_List[3 * Max_Range];
    int Dx[8];
    int Dy[8];
    vector<Coordinate> A_Star_Road;
};



// int main() {
//     freopen("out.txt", "w", stdout);
//     for (int i = 0; i < Max_Range; i++) {
//         for (int j = 0; j < Max_Range; j++) {
//             MM[i][j] = '0';
//         }
//     }
// //    for(int i = 8;i <= 8;i ++) {
// //        for(int j = 20;j <= 100;j ++) {
// //            MM[i][j] = '1';
// //        }
// //    }
//     for (int i = 50; i <= 52; i++) {
//         for (int j = 10; j <= 60; j++) {
//             MM[i][j] = '1';
//         }
//     }
//     for (int i = 40; i <= 50; i++) {
//         for (int j = 10; j <= 12; j++) {
//             MM[i][j] = '1';
//         }
//     }
//     for (int i = 40; i <= 50; i++) {
//         for (int j = 58; j <= 60; j++) {
//             MM[i][j] = '1';
//         }
//     }
// //    for (int i = 30; i <= Max_Range - 30; i++) {
// //        for (int j = Max_Range - 13; j <= Max_Range - 12; j++) {
// //            MM[i][j] = '1';
// //        }
// //    }
// //    for (int j = Max_Range - 1; j >= 0; j--) {
// //        for (int i = 0; i < Max_Range - 1; i++) {
// //            printf("%c", MM[i][j]);
// //        }
// //        printf("\n");
// //    }
//     Get_DWA_Answer::Pair Target;
//     Get_DWA_Answer::Coordinate Car_Coordinate = {0, 0};
//     Get_DWA_Answer::Coordinate Car_Destination = {0, 100};
// //    MM[Car_Destination.x][Car_Destination.y] = '9';
//     double Angle = 50;
//     double Velocity = 0;
//     double Angular_Velocity = 0;
//     Get_DWA_Answer::Bot_Model Model = {1.0, 60, 0.5, 60};
//     for (int i = 1; i <= 20; i++) {
//         Target = Get_DWA_Answer::DWA(MM, Car_Coordinate, Angle, Velocity, Angular_Velocity, Car_Destination, Model);
//         double Car_x = One_Block * Car_Coordinate.x;
//         double Car_y = One_Block * Car_Coordinate.y;
//         double Time_Sum = 0;
//         while (Time_Sum <= Predict_Delta) {
//             Time_Sum += Delta;
//             double Next_Angle = Angle + Target.Target_Angular_Velocity * Delta;
//             Car_x += Target.Target_Velocity * cos(Next_Angle * 0.017453292) * Delta;
//             Car_y += Target.Target_Velocity * sin(Next_Angle * 0.017453292) * Delta;
//             MM[lround(Car_x / One_Block)][lround(Car_y / One_Block)] = '7';
//             Angle = Next_Angle;
//         }
//         MM[lround(Car_x / One_Block)][lround(Car_y / One_Block)] = '7';
//         Velocity = Target.Target_Velocity;
//         Angular_Velocity = Target.Target_Angular_Velocity;
//         Car_Coordinate.x = lround(Car_x / One_Block);
//         Car_Coordinate.y = lround(Car_y / One_Block);
// //        for (int j = Max_Range - 1; j >= 0; j--) {
// //            for (int i = 0; i < Max_Range; i++) {
// //                printf("%c", MM[i][j]);
// //            }
// //            printf("\n");
// //        }
// //        printf("-------------------------------------------------------------------------\n");
// //        printf("-------------------------------------------------------------------------\n");
//     }
//     for (int j = Max_Range - 1; j >= 0; j--) {
//         for (int i = 0; i < Max_Range; i++) {
//             if (MM[i][j] != '0') printf("%c", MM[i][j]);
//             else {
//                 printf(" ");
//             }
//         }
//         printf("\n");
//     }
//    printf("%lf  %lf\n", Target.Target_Velocity, Target.Target_Angular_Velocity);
//     std::cout << "Hello, World!" << std::endl;
//     return 0;
// }