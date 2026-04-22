#include "gbp.hpp"




void print(std::vector<Variable*> vn)
{
    std::string s;
    for(auto &v : vn)
        s += v->repr() + " ";
    printf("%s\n", s.c_str());
}



int main(int argc, char **argv)
{
    
    
    // GBP test

    Variable *v0, *v1, *v2, *v3, *v4, *v5;
    Factor *f0, *f1, *f2, *f3, *f4, *f5;


    for(int t = 1; t < 2; t++)
    {
        std::vector<Factor*>      fac_nodes;
        std::vector<Variable*>    var_nodes;



        //var_nodes = {v0, v1};

        switch(t)
        {
            case 0:
                // f0-v0-f1-v1-f3
                //     |     |
                //     +-f2--+
                v0  = new Variable(0);
                v1  = new Variable(1);
                f0  = new Pose_factor(10, 0, {v0},        Vector2f(-1,0), 1.0);
                f1  = new Meas_factor(11, 0, {v0, v1},    Vector2f(1,0), 1.0);
                f2  = new Meas_factor(12, 0, {v0, v1},    Vector2f(2,2), 1.0);
                f3  = new Pose_factor(13, 0, {v1},        Vector2f(3,0), 1.0);
                
                var_nodes.push_back(v0);
                var_nodes.push_back(v1);
                fac_nodes.push_back(f0);
                fac_nodes.push_back(f1);
                fac_nodes.push_back(f2);
                fac_nodes.push_back(f3);
                break;
            case 1:
                // f0-v0-f1-v1
                // Works - expected (1,0), (2,0)
                v0  = new Variable(0);
                f0  = new Pose_factor(10, 0, {v0},        Vector2f(0, 0), 1.0);
                v1  = new Variable(1);
                f1  = new Meas_factor(11, 0, {v0, v1},    Vector2f(1, 0), 0.2);
                v2  = new Variable(2);
                f2  = new Meas_factor(12, 0, {v1, v2},    Vector2f(2, 0), 0.2);
                f3  = new Pose_factor(13, 0, {v2},        Vector2f(2, 0), 1.0);
                
                var_nodes.push_back(v0);
                fac_nodes.push_back(f0);
                var_nodes.push_back(v1);
                fac_nodes.push_back(f1);
                var_nodes.push_back(v2);
                fac_nodes.push_back(f2);
                fac_nodes.push_back(f3);
                break;
            case 2:
                // f0-v0-f1-v1-f2
                // Works - expected (0.990,0.01) (2.010,0.99)
                // v0  = new Variable(0, Gaussian((MatrixXf(2,1) << 0.9, 0).finished(), 0));
                // v1  = new Variable(1, Gaussian((MatrixXf(2,1) << 2.0, 0).finished(), 0));
                v0  = new Variable(0);
                v1  = new Variable(1);
                f0  = new Pose_factor(10, 0, {v0},        Vector2f(1,0), 1.0);
                f1  = new Meas_factor(11, 0, {v0, v1},    Vector2f(2,0), 1.0);
                f2  = new Pose_factor(12, 0, {v1},        Vector2f(2,0), 10.0);
                
                var_nodes.push_back(v0);
                var_nodes.push_back(v1);
                fac_nodes.push_back(f0);
                fac_nodes.push_back(f1);
                fac_nodes.push_back(f2);
                break;
            case 3:
                // f0-v0-f1
                //     |
                // f2--+
                //
                // Works = expected (1.5,0.333)
                v0  = new Variable(0);
                f0  = new Pose_factor(10, 0, {v0},        Vector2f(1, 0), 1.0);
                f1  = new Pose_factor(11, 0, {v0},        Vector2f(2, 0), 10.0);
                f2  = new Pose_factor(12, 0, {v0},        Vector2f(1.5, 1), 0.1);
                
                var_nodes.push_back(v0);
                fac_nodes.push_back(f0);
                fac_nodes.push_back(f1);
                fac_nodes.push_back(f2);
                break;
            case 4:
                // f0-v0-f1-v1
                //     |     |
                //     +-f2--+
                v0  = new Variable(0);
                v1  = new Variable(1);
                f0  = new Pose_factor(10, 0, {v0},        Vector2f(1,0), 1.0);
                f1  = new Meas_factor(11, 0, {v0, v1},    Vector2f(1,0), 1.0);
                f2  = new Meas_factor(12, 0, {v0, v1},    Vector2f(2,0), 0.1);
                
                var_nodes.push_back(v0);
                var_nodes.push_back(v1);
                fac_nodes.push_back(f0);
                fac_nodes.push_back(f1);
                fac_nodes.push_back(f2);
                break;
            case 5:
                // f0-v0-f1-v1-f2-v3-f3
                v0  = new Variable(0);
                v1  = new Variable(1);
                v2  = new Variable(2);
                f0  = new Pose_factor(10, 0, {v0},        Vector2f(1,0), 1.0);
                f1  = new Meas_factor(11, 0, {v0, v1},    Vector2f(1,0), 1.0);
                f2  = new Meas_factor(12, 0, {v1, v2},    Vector2f(1,0), 1.0);
                f3  = new Pose_factor(13, 0, {v2},        Vector2f(2,0), 1.0);
                
                var_nodes.push_back(v0);
                var_nodes.push_back(v1);
                var_nodes.push_back(v2);
                fac_nodes.push_back(f0);
                fac_nodes.push_back(f1);
                fac_nodes.push_back(f2);
                fac_nodes.push_back(f3);
                break;
            case 6:
                // f0-v0-f1
                // 
                v0  = new Variable(0);
                f0  = new Pose_factor(10, 0, {v0},        Vector2f(1,0), 1.0);
                f1  = new Pose_factor(11, 0, {v0},        Vector2f(2,0), 0.1);
                
                var_nodes.push_back(v0);
                fac_nodes.push_back(f0);
                fac_nodes.push_back(f1);
                break;
            case 7:
                // f0-v0-f1
                // 
                v0  = new Variable(0);
                f0  = new Pose_factor(10, 0, {v0},        Vector3f(0,0,0.5), 1.0);
                f1  = new Pose_factor(11, 0, {v0},        Vector3f(1,1,0.5), 1.0);
                
                var_nodes.push_back(v0);
                fac_nodes.push_back(f0);
                fac_nodes.push_back(f1);
                break;
            case 8:
                // f0-v0-f1-v1-f2
                // 
                v0  = new Variable(0);
                v1  = new Variable(1);
                f0  = new Pose_factor(10, 0, {v0},         Vector3f(1, 1, 0.2), 1.0);
                f1  = new Meas_factor(11, 0, {v0, v1},     Vector3f(1, 1, 0.5), 1.0);
                //f2  = new Pose_factor(12, Vars({v1}),         Vector3f(2, 2, 0.7), 1.0);
                
                var_nodes.push_back(v0);
                var_nodes.push_back(v1);
                fac_nodes.push_back(f0);
                fac_nodes.push_back(f1);
                //fac_nodes.push_back(f2);
                break;
            case 9:
                // f0-v0
                // 
                v0  = new Variable(0);
                f0  = new Pose_factor(10, 0, {v0},        Vector2f(1.5,1), 1.0);
                
                var_nodes.push_back(v0);
                fac_nodes.push_back(f0);
                break;
        }

// Result 0
// Value 0: (gtsam::Pose2) // (-0.0966717, -0.145649, -0.354043)
// Value 1: (gtsam::Pose2) // (2.09667, 0.145649, -0.236029)

// Result 1
// Value 0: (gtsam::Pose2) // (1, 0, 0)
// Value 1: (gtsam::Pose2) // (2, 0, 0)

// Result 2
// Value 0: (gtsam::Pose2) // (0.990196, 0, 0)
// Value 1: (gtsam::Pose2) // (2.98039, 0, 0)

// Result 3
// Value 0: (gtsam::Pose2) // (1.4951, 0.990001, 0)

// Result 4
// Value 0: (gtsam::Pose2) // (1, 0, 0)
// Value 1: (gtsam::Pose2) // (2.9901, 0, 0)

// Result 5
// Value 0: (gtsam::Pose2) // (0.75, 0, 0)
// Value 1: (gtsam::Pose2) // (1.5, 0, 0)
// Value 2: (gtsam::Pose2) // (2.25, 0, 0)

// Result 6
// Value 0: (gtsam::Pose2) // (1.9901, 0, 0)

// Result 7
// Value 0: (gtsam::Pose2) // (0.5, 0.5, 0.25)

// Convergence speed - to about 1%, in terms of factor nodes randomly selected, sending and
// recieving messages from all connected variable nodes
//      2d  lie             fac var
//  0   42  -               4   2   loop
//  1   5   6               2   2
//  2   5   11              3   2
//  3   3   3               3   1
//  4   49  90  (bouncy)    3   2   loop
//  5   19  -               4   3
//  6   3   3               2   1


        std::string s;
        setseed(2);

        {
       

            for(int i = 0; i < 15; i++)
            {
                Factor     *f;
                Edge       *e;


                f = fac_nodes[randi(0, fac_nodes.size() - 1)];
                // for(auto &e : f->edges)
                // {
                // }
                f->send_and_get_message(f->edges[0].v->id, 0.8);
                printf("%3d %2d: ", i, f->id);print(var_nodes);
                if (f->edges.size()==2) f->send_and_get_message(f->edges[1].v->id, 0.8);
                printf("%3d %2d: ", i, f->id);print(var_nodes);
            }
        }
        float error = 0;

        printf("%2d %6.3f ", t, error);
        if (error > 0.1)   printf("ERROR! ");
        else                printf("ok     ");
        for(auto &v : var_nodes)
        {
            printf("%3d %6.3f %6.3f   ", v->id,
                v->belief.mu(0), v->belief.mu(1));
            
        }
        printf("\n");
        
        
    }
    //exit(0);



}