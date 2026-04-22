

#include "gbp.hpp"

// // FIXME!! Hack, need to sort includes properly
// namespace Otype { enum
// {
//     WALL    = 1,
//     ROBOT,
//     BOX
// }; }

constexpr int DEBUG = 0;
constexpr int DEBUG2 = 0;

float damping_f = 0.5;
float damping_v = 0.0;


void Variable::print(int name)
{
    if (otype == 3)
        printf("var box%d %s                 ", name, repr().c_str());
    else
        printf("var node %s                 ", repr().c_str());
    for(auto &e : edges)
        if (e)
        {
            if (e->f)
                printf("id:%4d ", e->f->id);
            else
                printf("id:e%3d ", e->robot_id);
        }
    printf("\n");
}




int Variable::update_belief()
{
    int flops = 0;
    {
        // Vector2f    eta         = prior.eta;
        // Matrix2f    lambda      = prior.lambda;
        Vector2f    eta         = Vector2f::Zero();
        Matrix2f    lambda      = Matrix2f::Zero();

        bool fresh = false;
        for(auto &e : edges)
        {
            eta     += e->factor_to_var_msg.eta;
            lambda  += e->factor_to_var_msg.lambda;
            fresh   |= e->fresh;
            e->fresh = false;
            flops += 3;
            // if (otype == Otype::BOX && timestep < e->factor_to_var_msg.timestep)
            //     timestep = e->factor_to_var_msg.timestep;
        }

        if (lambda(0,0) < 0) {printf("neg lambda %f\n", lambda(0,0)); exit(1);}
        
        // belief.get_mean();
        //std::string before = belief.repr();
        // delta = belief.mu;

        belief.eta      = eta;
        belief.lambda   = lambda;
        // belief.timestep = timestep;
        // belief.ticks    = ticks;
        belief.get_mean();
        // delta -= belief.mu;
        //printf("%6d %f %s %s\n", id, delta.norm(), before.c_str(), belief.repr().c_str());

        // 3
        //flops += 3;

        belief.tick     = timer::ticks();
        // Send belief to all edges
        for(auto &e : edges)
        {
            e->var_to_factor_msg    = belief;
            if (e->f)
                if(DEBUG2)printf("var %4d setting belief on %4d %s f_to_v is %s\n", id, e->f->id, belief.repr().c_str(), e->factor_to_var_msg.repr().c_str());
        }
    }
    updated = true;


    if(DEBUG)
    {
        std::string s;
        for(auto &e : edges)
        {
            int id = 0;
            if (e->f) id = e->f->id;
            auto &m = e->factor_to_var_msg;
            s += string_format("id:%4d % 6.3f % 6.3f % 6.3f ", id, m.eta(0), m.eta(1), m.lambda(0,0));
        }
        printf("Var node %d belief %s %s\n", id, belief.repr().c_str(), s.c_str());
        //belief.print();
    }
    return flops;
}

int Factor::linearise()
{
    {
        if (linearised)
            return 0;

        // -------------- orig
        Vector2f hlocal = h();
        MatrixXf Jlocal = J();


        // 
        // distance    = sqrt((z - hlocal).transpose() * factor.lambda * (z - hlocal));

        lambda      = factor.lambda;
        if (lambda(0,0) < 0) {printf("%s neg lambda %f\n", __func__, lambda(0,0)); exit(1);}


        // We modify factors when the distance between measurement and expectation 
        // is above a threshold.

        adj_factor  = 1.0;


        lambda_prime    = Jlocal.transpose() * lambda * Jlocal;
        factor.eta      = Jlocal.transpose() * lambda * z;
        
        linearised = true;

        // We need no flops for this operation
        return 0;

        // Because linear, lambda is always diagonal
        // Unary factor 
        // lambda_prime = lambda
        // eta          = lambda_prime * z
        //
        // Binary factor
        // lambda_prime =   l  -l
        //                 -l   l
        // eta          = lambda_prime * z




        //  lambda_prime = J_s^T*Lambda*J_s =
        // -1   0                                      -s   0       s   0  -s   0
        //  0  -1   *   s   0   *  -1   0   1   0   =   0  -s  =>   0   s   0  -s
        //  1   0       0   s       0  -1   0   1       s   0      -s   0   s   0
        //  0   1                                       0   s       0  -s   0   s
        //
        //  z   =   1   0
        //  x0  =   x0  y0  x1  y1
        //  hlocal =    x1-x0
        //              y1-y0
        //  Jlocal * x0 +z - hlocal =   
        // -1   0   1   0   *   x0  =   x1-x0   -hlocal = 0 + z =   zx
        //  0  -1   0   1       y0      y1-y0                       zy
        //                      x1
        //                      y1
        //
        // eta = J_s^T**Lambda*z
        // -1   0                   -s   0  
        //  0  -1   *   s   0   =    0  -s
        //  1   0       0   s        s   0
        //  0   1                    0   s
        //
        // eta =   -s   0              -szx
        //          0  -s   *   zx  =  -szy
        //          s   0       zy      szx
        //          0   s               szy
        //
        // So the is no dependence on the actual state variables, this would not be true if
        // the measurement function h() was not linear.

        //
        //  dx  dy *    s   0   *   dx      =   sdx sdy *   dx  =   sdx^2 + sdy^2
        //              0   s       dy                      dy
    }

}



MatrixXf Factor::x0()
{
    // Stack all mu as x0
    long s = edges.size();
    MatrixXf x(2 * s, 1);
    for (int i = 0; i < edges.size(); i++)
    {
        x(seqN(i * 2, 2), 0) = edges[i].var_to_factor_msg.mu;
    }
    return x;
}


int Factor::send_and_get_message(int vid, float damping)
{
    int flops = send_message(vid, damping);
    bool not_updated = false;
    for(auto &e : edges)
    {
        if (e.v)
        {
            if (e.v->id == vid)
                flops += e.v->update_belief();
            if (!e.v->updated)
                not_updated = true;
        }
    }
    //updated = !not_updated;
    return flops;
}


int Factor::send_message(int vid, float damping)
{
    int flops = 0;
    float deviation = 0;
    int     outedge;
    if(1){
        // printf("==================== ID: %d\n", id);
        flops += linearise();
        // -------------- orig
        // eta has twice as many rows as there are edges, lambda_prime is 2*edges x 2*edges
        VectorXf le = factor.eta;
        MatrixXf ll = lambda_prime;
        if(DEBUG2)
        {
            if (edges.size()==2)printf("fac=>%4d %9.3f      %9.3f %9.3f %9.3f %9.3f\n", id, le(0), ll(0,0), ll(0,1), ll(0,2), ll(0,3));
            if (edges.size()==2)printf("fac  %4d %9.3f      %9.3f %9.3f %9.3f %9.3f\n", id, le(1), ll(1,0), ll(1,1), ll(1,2), ll(1,3));
            if (edges.size()==2)printf("fac  %4d %9.3f      %9.3f %9.3f %9.3f %9.3f\n", id, le(2), ll(2,0), ll(2,1), ll(2,2), ll(2,3));
            if (edges.size()==2)printf("fac  %4d %9.3f      %9.3f %9.3f %9.3f %9.3f\n", id, le(3), ll(3,0), ll(3,1), ll(3,2), ll(3,3));
        }

        // if (edges.size()==2)printf("fac  %4d e %6.3f %6.3f %6.3f %6.3f lp %6.3f\n", id, le(0), le(1), le(2), le(3), ll(0,0));
        int offset = 0, outoffset;
        int s = 2;
        // eqn 42, 43
        VectorXf sedge_eta = Vector2f::Zero();;
        MatrixXf sedge_lambda = Matrix2f::Zero();
        for(int i = 0; i < edges.size(); i++)
        {
            auto &e = edges[i];
            VectorXf edge_eta;
            MatrixXf edge_lambda;
            // if (e.var_to_factor_msg.timestep == -1)
            // {
            //     return 0;
            //     edge_eta       = e.factor_to_var_msg.eta;
            //     edge_lambda    = e.factor_to_var_msg.lambda;
            // }
            // else
            {
                edge_eta       = e.var_to_factor_msg.eta      - e.factor_to_var_msg.eta;
                edge_lambda    = e.var_to_factor_msg.lambda   - e.factor_to_var_msg.lambda;
                // 2 subs for eta, 1 for lambda
                flops += 3;
            }

            // if (e.var_to_factor_msg.otype == Otype::BOX && timestep < e.var_to_factor_msg.timestep)
            //     timestep = e.var_to_factor_msg.timestep;

            if(DEBUG2)
            {
                if (edges.size()==2)printf("fac  %4d %8.3f      %8.3f %8.3f\n", id, edge_eta(0), edge_lambda(0,0), edge_lambda(0,1));
                if (edges.size()==2)printf("fac  %4d %8.3f      %8.3f %8.3f\n", id, edge_eta(1), edge_lambda(1,0), edge_lambda(1,1));
            }
            //if (edge_lambda(0,0) <= 0.001)
            // if (e.factor_to_var_msg.lambda(0,0) == 0)
            // {
            //     // Hack to handle startup conditions ##FIXME## think more about this
            //     // We can't know how much contribution the factor would have made in a previous
            //     // message, so assume half
            //     edge_lambda = e.var_to_factor_msg.lambda;
            //     edge_eta    = e.var_to_factor_msg.eta;

            // }
            //assert(edge_lambda(0,0) > 0);
            if(DEBUG2)
            {
                if (edges.size()==2)printf("fac  %4d %8.3f      %8.3f %8.3f\n", id, edge_eta(0), edge_lambda(0,0), edge_lambda(0,1));
                if (edges.size()==2)printf("fac  %4d %8.3f      %8.3f %8.3f\n", id, edge_eta(1), edge_lambda(1,0), edge_lambda(1,1));
            }
            // A nullptr on the variable means the variable is external, and a zero vid means
            // send message to it
            if ((e.v && (e.v->id != vid)) || (!e.v && vid))
            {
                le(seqN(i * s, s))                  += edge_eta;
                ll(seqN(i * s, s), seqN(i * s, s))  += edge_lambda;
                flops += 3;
                sedge_eta = edge_eta;
                sedge_lambda = edge_lambda;
            }
            else
            {
                outoffset   = i * s;
                outedge     = i;
            }
        }

        // eqn 50 Shuffle to get variable we care about to top of stacked vector
        VectorXf etatemp            = le(seqN(0, s));
        le(seqN(0, s))              = le(seqN(outoffset, s));
        le(seqN(outoffset, s), 0)   = etatemp;
        
        // eqn 51 Shuffle to get variable we care about to top left of matrix
        MatrixXf lptemp             = ll(seqN(0, s), all);
        ll(seqN(0, s), all)         = ll(seqN(outoffset, s), all);
        ll(seqN(outoffset, s), all) = lptemp;
        lptemp                      = ll(all, seqN(0, s));
        ll(all, seqN(0, s))         = ll(all, seqN(outoffset, s));
        ll(all, seqN(outoffset, s)) = lptemp;
        
        // eqn 46, 47 Marginalise
        // cases like seq(2, last) for a 2 element vector work because
        // equiv to seqn(2, 1-2+1) = seqn(2,0), and will return an empty vector
        VectorXf ea     = le(seqN(0, s));
        VectorXf eb     = le(seq(s, last));
        MatrixXf laa    = ll(seqN(0, s),    seqN(0, s));
        MatrixXf lab    = ll(seqN(0, s),    seq(s, last));
        MatrixXf lba    = ll(seq(s, last),  seqN(0, s));
        MatrixXf lbb    = ll(seq(s, last),  seq(s, last));
        
        // for a unary factor
        // ea = le
        // eb = <>
        // laa = ll
        // lab = lba = lbb = <>
        // emarg = le
        // lmarg = ll
        //
        // for a binary factor
        // ea = -Lambda*z
        // eb = Lambda*z + eta_m2
        // laa = Lambda_s
        // lab = -Lambda_s
        // lba = -Lambda_s
        // lbb = Lambda_s + Lambda_m2
        // bbinv = lbb^-1
        //
        // bbinv =  [ 1/(p_s+p_m)           0   ]
        //          [ 0           1/(p_s+p_m)   ]
        //
        // lab * bbinv =    p_s/(p_s+p_m)               0
        //                  0               p_s/(p_s+p_m)
        //
        // lab * bbinv * eb = 

        MatrixXf bbinv  = lbb.inverse();                // 1
        MatrixXf emarg  = ea - lab * bbinv * eb;        // 1 + 2 + 2 = 5
        MatrixXf lmarg  = laa - lab * bbinv * lba;      // 1 + 1 = 2
        // 1 + 5 + 2
        flops += 8;
        
        auto &e = edges[outedge];
        
        if(DEBUG2)
        {
            if (edges.size()==2)printf("fac  %4d %9.3f      %9.3f %9.3f %9.3f %9.3f\n", id, le(0), ll(0,0), ll(0,1), ll(0,2), ll(0,3));
            if (edges.size()==2)printf("fac  %4d %9.3f      %9.3f %9.3f %9.3f %9.3f\n", id, le(1), ll(1,0), ll(1,1), ll(1,2), ll(1,3));
            if (edges.size()==2)printf("fac  %4d %9.3f      %9.3f %9.3f %9.3f %9.3f\n", id, le(2), ll(2,0), ll(2,1), ll(2,2), ll(2,3));
            if (edges.size()==2)printf("fac  %4d %9.3f      %9.3f %9.3f %9.3f %9.3f\n", id, le(3), ll(3,0), ll(3,1), ll(3,2), ll(3,3));

            if (edges.size()==2)printf("fac  %4d em%6.3f %6.3f lm %6.3f\n", id, emarg(0), emarg(1), lmarg(0,0));
        }


        // emarg = emarg * adj_factor;
        // lmarg = lmarg * adj_factor;
        // // 3
        // flops += 3;


        if (0)
        {
            float l;
            if (edges.size() == 2)
            {
                l = lambda_prime(0, 0);
                if (!outedge)
                {
                    emarg   = factor.eta(seqN(0, 2));
                }
                else
                {
                    emarg   = factor.eta(seqN(0, 2));
                }
                float alpha = l / (l + sedge_lambda(0, 0));

                emarg   = (1 - alpha) * emarg + alpha * sedge_eta;
                lmarg   = alpha * sedge_lambda;
            }
            else
            {
                emarg   = factor.eta(seqN(0, 2));
                lmarg   = lambda_prime(seqN(0, 2), seqN(0, 2));
            }
        }



        if (lmarg(0,0) < 0) {
            if(DEBUG2)printf("%s 1 neg lambda %f ts %d %d\n", __func__, lmarg(0,0), 
                edges[0].var_to_factor_msg.timestep,
                edges[1].var_to_factor_msg.timestep
                ); 
            if(DEBUG2)printf("%s\n", repr().c_str());
            // ###FIXME### this is a dirty hack!!
            emarg(0) = 0; 
            emarg(1) = 0;
            lmarg(0,0) = 0.1;
            lmarg(1,1) = 0.1;
            //exit(1);
        }


        if (e.factor_to_var_msg.eta(0) == 0)
        {
            // No damping on first iteration
            e.factor_to_var_msg.eta         = emarg;
            e.factor_to_var_msg.lambda      = lmarg;
        }
        else
        {
            //                                   1          2       2         2
            e.factor_to_var_msg.eta         = (1 - damping) * emarg + damping * e.factor_to_var_msg.eta;
            //                                   0          1       1         1
            e.factor_to_var_msg.lambda      = (1 - damping) * lmarg + damping * e.factor_to_var_msg.lambda;
            // e.factor_to_var_msg.lambda      = lmarg;
            // 7 + 3
            flops += 10;
        }
        e.factor_to_var_msg.timestep    = timestep;
        e.factor_to_var_msg.tick        = timer::ticks();
    
        e.factor_to_var_msg.get_mean();
        if (lmarg(0,0) < 0) {printf("%s neg lambda %f\n", __func__, lmarg(0,0)); exit(1);}
        // assert(e.factor_to_var_msg.lambda(0,0)>0);

    }




    updated = true;

    auto v = edges[outedge].v;
    if(DEBUG)printf("Fac node %d sent to var node %d : %s\n", id, v ? edges[outedge].v->id : 0, edges[outedge].factor_to_var_msg.repr().c_str());

    return flops;

}