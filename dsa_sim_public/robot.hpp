#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "glhelpers.hpp"
#include <vector>
#include <deque>
#include <random>
#include <math.h>
#include "draw.h"
#include <Eigen/Dense>

#include "sim.hpp"
#include "gbp.hpp"
#include "utils.hpp"
#include "factor_graph.hpp"


struct Obs_box_state
{
    Obs_box_state() : p(Vector2f::Zero()), t(0) {}
    Vector2f    p;
    int         t;
    int         name;
};



struct Senses
{
    // Measured velocity, distorted with noise
    Vector2f        velocity;
    // Estimated swarm-relative position
    Gaussian        position;
    // 
    int             num_neighbours;
    // Angle of largest concentration of neighbouring robots
    float           near_neighbours;
    Vector2f        neighbours;
    // Are we inside a shape?
    bool            in_shape;
    // Have we converged?
    bool            converged;
    // Field gradient
    Vector2f        gradient;
    float           angle;
};

class Robot : public Object
{
public:
    Robot(Sim *_sim, Vector2f _s);
    const float radius      = 0.125;
    const float ltof_range  = 2.0;
    const float ltof_angle  = M_PI * 20.0 / 180.0;
    float vel           = 1.0;
    float error         = 0.0;
    float gbp_error     = 0.0;
    float max_error     = 0.0;
    int                 chasing_name        = -1;
    bool                all_boxes_observed  = false;


    void step();
    void sensors();
    void controller();
    void set_sensor_radius(float r);
    void add_sensed_object(uintptr_t sensed);
    void remove_sensed_object(uintptr_t sensed);
    void list_sensed(int type = 0);
    Vector2f velocity_sense_model(Vector2f true_vel);
    Vector2f position_sense_model(b2Body *b);
    // void measure_error();
    void choose_new_trajectory(bool seek_boxes);
    void render(DebugDraw *dd, Vector2f bias);
    
    void build_graph();
    void iterate_graph();
    void update_blackboard();


    b2Color                     colour;
    int                         name;
    int                         id;
    
    
    Vector2f                    cmd_vel;
    Vector2f                    odometry;
    Vector2f                    est_pos;
    float                       abs_move;
    float                       move_variance;

    b2Vec2                      col_vel;
    b2Vec2                      contact_normal;
    int                         col_otype;

    float                       traj_time;
    float                       traj_direction;
    std::vector<float>          all_error;
    std::vector<float>          all_gbp_error;
    std::vector<float>          all_bias_error;
    std::vector<float>          deviation;
    std::map<int, Box_state>    box_states;
    Factor_graph                fg;
    // Factor_graph                fgcs;
    bool                        time_for_new_node;
    Vector2f                    bias;
    std::vector<float>          box_dist;
    std::vector<Obs_box_state>  oracle_observed_box_state;
    std::vector<Obs_box_state>  rel_observed_box_state;
    bool                        carrying;
    Object                      *carried;

    // BT controller stuff
    Senses                      senses;
    BT::Tree                    tree;


    int                         total_out_bytes;
    int                         total_ret_bytes;
    int                         total_bb_bytes;

    std::set<int>               seen_robots;
    bool                        met_half = false;
    float                       t_convproxy;

private:
    Sim                         *sim;
    Vector2f                    state;
    std::vector<uintptr_t>      sensed_objects;
    std::map<int, Static_box*>  sensed_boxes;
    struct Sr 
    {
        Sr() {}
        Sr(int _t, Robot *_r) : tick(_t), r(_r) {}
        int tick;
        Robot *r;
    }; 
    std::map<int, Sr>           sensed_robots;
    std::map<int, Sr>           commed_robots;
    Vector2f                    Q;
    Vector2f                    true_vel;
    b2Fixture                   *sensefix;



};








#endif