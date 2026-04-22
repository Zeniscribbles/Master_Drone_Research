#ifndef GBP_HPP
#define GBP_HPP


#include <Eigen/Dense>
// #include <manif/manif.h>
#include <map>
#include <vector>
#include <deque>
#include "utils.hpp"
// #include <cassert>
// #include <assert.h>

using namespace Eigen;
// using namespace manif;


// Classes for factor and variable nodes, and messages. The nodes do not hold values,
// just links to edges over which messages pass. The edge contains the value of the
// last message
//
// Section and equation numbers are from:
// FutureMapping 2: Gaussian Belief Propagation for Spatial AI
// \cite{davison2019futuremapping}


class Edge;
class Factor;
class Variable;

typedef std::vector<Variable*>  Vars;
typedef std::vector<Factor*>    Facs;
typedef std::deque<Edge*>      pEdges;
typedef std::vector<Edge>       Edges;

//  source robot    4
//  dest robot      4
//  count           1
const int out_msg_overhead = 12;

//  timestep    4
const int out_msg_size = 4;

//  source robot    4
//  dest robot      4
//  count           1
const int ret_msg_overhead = 12;
// 
//  #valid           1
//      eta         8
//      lambda      4
//      timestep    4
//
// valid encoded in timestep being positive
const int ret_msg_size = 16;


class Gaussian
{
public:
    Gaussian(MatrixXf _eta, MatrixXf _lambda)
    :   eta(_eta), lambda(_lambda)
    {
        Gaussian();
        get_mean();
    }
    Gaussian(Vector2f _mu, float sigma)
    {
        mu = _mu;
        float p = 1 / (sigma * sigma);
        lambda.resize(2, 2);
        lambda << p, 0, 0, p;
        eta = lambda * mu;
        timestep    = -1;
        tick        = -1;
    }
    Gaussian()
    {
        eta     = MatrixXf::Zero(2, 1);
        mu      = MatrixXf::Zero(2, 1);
        lambda  = Matrix2f::Zero();

        timestep    = -1;
        tick        = -1;
    }
    int         timestep;
    int         tick;
    int         otype;
    int         oid;
    int         box;
    int         id = -1;
    VectorXf    eta;
    MatrixXf    mu;
    MatrixXf    lambda;
    MatrixXf    cov;


    MatrixXf    get_cov()
    {
        return cov = lambda.inverse();
    }
    MatrixXf    get_mean()
    {
        // assert(!isnan(lambda(0,0)));
        // if (isnan(lambda(0,0)))
        // {
        //     printf("nan\n");
        //     exit(1);
        // }
        get_cov();
        if (lambda(0,0) == 0 || lambda(1,1) == 0)
            return mu = MatrixXf::Zero(2, 1);
        return  mu = cov * eta;
    }
    std::string repr()
    {
        get_mean();
        std::string s;
        s  = string_format("% 6.3f % 6.3f (% 9.3f) ", mu(0), mu(1), lambda(0,0));
        return s;
    }
    void print()
    {
        printf("    mu         lambda\n%s\n", repr().c_str());
    }
};




class Edge
{
public:
    Edge()
    :   f   (nullptr),
        v   (nullptr),
        var_to_factor_msg   (Gaussian()),
        factor_to_var_msg   (Gaussian()),
        fresh   (false)
        {}
    Edge(Factor *_f, Variable *_v) 
    :   f   (_f), 
        v   (_v),
        var_to_factor_msg   (Gaussian()),
        factor_to_var_msg   (Gaussian()),
        fresh   (false)
        {}
    Factor          *f;
    Variable        *v;
    int             robot_id;     // id of robot connecting to a var node
    Gaussian        var_to_factor_msg;
    Gaussian        factor_to_var_msg;
    int             timestep;
    bool            fresh;
};


class Variable
{
public:
    Variable() : Variable(uid(), 0) {}
    Variable(int _ts, Gaussian _g) : Variable(uid(), _ts, _g) {}
    Variable(int _id) : Variable(_id, 0) {}
    Variable(int _id, int _ts) : Variable(_id, _ts, Gaussian()) {}
    Variable(int _id, int _ts, Gaussian _g) 
    :   id      (_id), 
        timestep(_ts),
        belief  (_g),
        prior   (_g),
        ticks   (timer::ticks())
    {
        belief.timestep = timestep;
        prior.timestep  = timestep;
    }

    
    int update_belief();
    


    std::string repr()
    {
        float sig = sqrt(1.0 / belief.lambda(0,0));
        std::string s = string_format("id %5d ts %4d mu % 6.3f % 6.3f sigma %6.3f lambda % 9.3f %6d ", id, timestep,
            belief.mu(0), belief.mu(1), sig,
            belief.lambda(0,0), ticks
            );
        return s;
    }
    void print(int name);
    void print(){print(0);};


    int                     id;
    int                     otype;
    int                     name;
    int                     bid;
    int                     timestep;
    pEdges                  edges;
    Vector2f                gt;

    Gaussian                belief;
    MatrixXf                delta;
    Gaussian                prior;
    bool                    updated = false;
    int                     ticks;



};

class Factor
{
public:
    // 3.1 to specify a factor, we need:
    //  h(x)    measurement dependence on local state variables
    //  z       observed measurement
    //  Lambda  precision matrix of measurement
    //
    // We linearise using eqn 36. For now, assuming linear
    Factor(int _id, int _ts, Vars _vars, VectorXf _z)
    :   id          (_id),
        timestep    (_ts),
        vars        (_vars),
        edges       (_vars.size()),
        ticks       (timer::ticks())
    {

        // For each variable node, create an edge linking to here   
        for (int i = 0; i < vars.size(); i++)
        {
            edges[i] = Edge(this, vars[i]);
            edges[i].timestep = timestep;
            if (vars[i])
            {
                // A variable can be NULL, don't access if so
                auto e = edges.data() + i;
                vars[i]->edges.push_back(e);
                e->var_to_factor_msg = vars[i]->belief;
            }
            // else
            //     edges[i].var_to_factor_msg = Gaussian(Vector2f::Zero(), 2500);
        }
        //factor = Gaussian();
        factor.timestep = timestep;

        z(0) = _z(0);
        z(1) = _z(1);

    }


    virtual ~Factor() = default;
    MatrixXf x0();
    int linearise();
    // void linearise_lie();
    int send_message(int vid, float damping);
    int send_and_get_message(int vid, float damping);
    int send_message(int vid) {return send_message(vid, 0.0);}
    int send_and_get_message(int vid) {return send_and_get_message(vid, 0.0);};
    // void send_random_message();

    std::string repr()
    {
        std::string s = "";
        if (factor.eta.rows() > 0)
        {
            // if (factor.eta(0) == 0)
            // {
            //     s = string_format("et ------ ------ lambda % 9.3f z % 6.3f % 6.3f ", factor.lambda(0, 0), z(0), z(1));
            // }
            // else
            {
                // factor.get_mean();
                s = string_format("id %5d ts %4d h  % 6.3f % 6.3f % 6.3f lambda % 9.3f z % 6.3f % 6.3f %6d ", id, timestep, h()(0), h()(1), distance,
                        factor.lambda(0, 0), z(0), z(1), ticks);
            }
        }
        return s;
    }
    std::string fr()
    {
        std::string s = "";
        s = string_format("e0:vf %s fv %s ", edges[0].var_to_factor_msg.repr().c_str(), edges[0].factor_to_var_msg.repr().c_str());
        if (edges.size() > 1)
            s += string_format("e1:vf %s fv %s ", edges[1].var_to_factor_msg.repr().c_str(), edges[1].factor_to_var_msg.repr().c_str());
        
        return s;
    }
    void print()
    {

        printf("    fac  %s ", repr().c_str());
        for(auto &e : edges)
        {
            int vid = 0;
            if(e.v)
                vid = e.v->id;
            printf("id:%4d ", vid);
        }
        printf("\n");
    }

    // These must be defined by specific factor nodes
    virtual Vector2f        h() = 0;
    virtual MatrixXf        J() = 0;


    int                     id;
    int                     other_id;
    int                     timestep;
    bool                    updated = false;
    bool                    linearised = false;

    Vars                    vars;
    Edges                   edges;
    Vector2f                z;
    Gaussian                factor;
    //MatrixXf                eta;
    Matrix2f                lambda;
    MatrixXf                lambda_prime;
    float                   distance = 0;
    float                   adj_factor = 1.0;
    int                     ticks;
};





class Pose_factor : public Factor
{
public:
    Pose_factor(int _id, int _ts, Vars _vars, VectorXf x, float sigma)
    :   Pose_factor (_id, _ts, _vars, x, Vector2f(sigma, sigma)) {}
    Pose_factor(int _id, int _ts, Vars _vars, VectorXf x, Vector2f sigma)
    :   Factor (_id, _ts, _vars, x)
    {
        factor.lambda << 1.0 / (sigma(0) * sigma(0)), 0.0, 0.0, 1.0 / (sigma(1) * sigma(1));
    }
    Vector2f h()
    {
        return edges[0].var_to_factor_msg.mu;
    }
    MatrixXf J()
    {
        MatrixXf J {{1.0, 0.0}, {0.0, 1.0}};
        // MatrixXf J(2, 2);
        // J << 1.0, 0.0, 0.0, 1.0;
        return J;
    }

};

class Meas_factor : public Factor
{
public:
    Meas_factor(int _id, int _ts, Vars _vars, VectorXf x, float sigma)
    :   Meas_factor(_id, _ts, _vars, x, Vector2f(sigma, sigma))
    {}
    Meas_factor(int _id, int _ts, Vars _vars, VectorXf x, Vector2f sigma, int _other_id = 0)
    :   Factor(_id, _ts, _vars, x)
    {
        factor.lambda << 1.0 / (sigma(0) * sigma(0)), 0.0, 0.0, 1.0 / (sigma(1) * sigma(1));

        other_id = _other_id;
        edges.resize(2);
    }
    Vector2f h()
    {
        Vector2f xfrom  = edges[0].var_to_factor_msg.mu;
        Vector2f xto    = edges[1].var_to_factor_msg.mu;
        Vector2f h      = xto - xfrom;
        return h;
    }
    MatrixXf J()
    {
        // This doesn't work on eigen 3.3 
        // https://stackoverflow.com/questions/73448417/brace-initialization-of-eigen-matrix
        MatrixXf J {{-1.0, 0.0, 1.0, 0.0}, {0.0, -1.0, 0.0, 1.0}};
        // MatrixXf J(2, 4);
        // J << -1.0, 0.0, 1.0, 0.0, 0.0, -1.0, 0.0, 1.0;
        return J;
    }


};



#endif