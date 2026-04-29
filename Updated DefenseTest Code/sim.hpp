
#ifndef SIM_HPP
#define SIM_HPP

#include "glhelpers.hpp"

#include <box2d/box2d.h>

#include <vector>
#include <random>
#include <math.h>
#include "draw.h"
#include <eigen3/Eigen/Dense>
#include "gbp.hpp"
#include "bt.hpp"

#include "behaviortree_cpp/bt_factory.h"


using namespace Eigen;



// Helpers from Box2d types
Vector2f to_eigen(b2Vec2 v);
Vector3f to_eigen3(b2Vec2 v, float th);

// Forward definitions
class Sim;
class Robot;
class Static_box;

// Hue Sat Lum to RGB to make it easier to choose nice colours
b2Color hsl2rgb(float h, float s, float l);


namespace Category { enum
{
    ARENA       = 1 << 0,
    ROBOT_BODY  = 1 << 1,
    ROBOT_CAM   = 1 << 2,
    BOX         = 1 << 3,
}; }

namespace Otype { enum
{
    WALL    = 1,
    ROBOT,
    BOX
}; }

namespace Ctrl { enum
{
    random  = 0,
    seek,
    shape,
    sfile
}; }

// Container for Box2d objects, with a singleton id variable
class Object
{
public:
    Object()    {id = ++gid;}
    int         otype;
    b2Body      *body;
    static int  gid;
    int         id;
};


class Box_state
{
public:
    Box_state() {}
    Box_state(Vector2f _x, Vector2f _P) 
    : x(_x), P(_P) {}
    Vector2f x;
    Vector2f P;
    Vector2f gt;
    Variable *v = NULL;
};






class Static_box : public Object
{
public:
    Static_box(Sim *_sim, b2Vec2 _pos, float _width, float _height, int _otype, int _name=0);
    ~Static_box();
    void render(DebugDraw *dd, b2Color c);
    int     name;
    b2Vec2  pos;
private:
    Sim     *sim;
    float   width;
    float   height;
};


class Contact_listener : public b2ContactListener
{
    void BeginContact(b2Contact *contact);
    void EndContact(b2Contact *contact);
    void PreSolve(b2Contact *contact, const b2Manifold *oldManifold);
};

struct Sim_params
{
    int     num_boxes_gui       = 5;            // yingfei mark only, no change
    int     num_robots_gui      = 10;
    float   arena_size          = 5.0;
    float   start_size          = 5.0; // yingfei add a staring size, NEW: Controls the initial spawn area
    //////////////////////////////////////////
    int     num_robots          = num_robots_gui;
    float   robot_vel           = 0.5;
    int     num_boxes           = num_boxes_gui;
    float   dt                  = 1.0 / 60;
    float   min_sep             = 0.26;
    float   sigma_vsense_est    = 0.1;  
    // float   sigma_vsense_est_min = 0.01;  
    float   sigma_psense_est    = 0.02;
    float   sigma_boxcopy_est   = 1.0;
    float   sigma_anchor_est    = 1.0;
    float   sigma_vsense_noise  = 0.1;  
    float   sigma_psense_noise  = 0.02;
    float   sigma_prior         = 1.0;
    float   r_sense             = 0.5;
    float   max_error           = 7.0;
    float   speed               = 1.0;
    float   beta                = 3.0;
    // commands
    bool    ctrl_en             = true;
    // GBP params
    int     gbp_iters           = 1;
    int     ticks_per_update    = 6;
    int     msg_schedule        = 0;
    int     ticks_comms_dwell   = ticks_per_update * 10;
    float   damping             = 0.8;              // yingfei mark only, no change

    // Time before creating new pose node
    float   new_node_time       = 0.5;
    int     bb_recheck_time     = 1;
    //float   new_node_time   = 0.1;
    int     fg_window           = 10;
    float   fg_history_time     = new_node_time * fg_window;

    // Debug messages
    bool    debug_fg        = false;
    bool    debug_msgs      = false;

    // movement
    float   sigma_seek_noise= 0.01;
    bool    move_en         = false;
    bool    rmove_en        = false;
    float   box_vel         = 0.01;
    float   box_dist        = 1.0;
    int     move_state      = 0;
    int     move_counter    = 0;
    int     move_dwell      = 10000;
    int     move_run        = 300;
    float   box_offset      = 8.0;
    float   box_gain        = 5.0;
    FILE*   fp              = 0;

    // Controller
    int     ctrl_idx        = 0;
    float   agg_time        = 0.1;
    float   agg_noise       = 0.01;
    float   agg_vel       = 0.1;
    int     shape_idx       = 0;
    uint8_t     *shape_data;
    std::string shape_file  = "";
    float   attr            = 0.1;
    float   repul           = 0.1;


    // Visualisation
    int     decay_time      = 15;
    float   lighten         = 1.0;
    float   alpha           = 1.0;



    struct
    {
        bool    box         = true;
        bool    robots      = true;
        bool    fg          = false;
        bool    sensor      = false;
        bool    sense_line  = true;
        bool    msg         = false;
        bool    factor      = false;
        bool    ebpos       = false;
        bool    shape       = false;
        bool    name        = false;
        bool    origin      = false;
        bool    oline       = false;
        bool    estats      = false;
        bool    rbias       = false;
        bool    obox        = false;
        bool    mono        = false;
        bool    aim_oldest  = false;
        bool    wall        = false;
    } gui;
};

class Sim
{
public:
    Sim(Sim_params &_sp, int _seed = 1);
    int seed;
    void create_arena(float size);
    void create_robot(Vector2f s);
    void create_robot(Vector2f s, b2Vec2 v, b2Color c, int name, float duration);
    void create_bt_factory();
    void create_box(b2Vec2 p, int otype, int name);
    bool check_obstruction(b2Vec2 p);
    bool check_obstruction(b2Vec2 p, int box, bool check_robots);
    bool create_random_robots(int num, float minx, float maxx, float miny, float maxy);
    void create_random_boxes(int num, float minx, float maxx, float miny, float maxy);
    float move_random_box(float dist, float minx, float maxx, float miny, float maxy);
    float move_random_box2(float dist, float minx, float maxx, float miny, float maxy);
    void step();
    void render(int w, int h, int rplot_flags[], bool show_fg);
    void reset();
    bool in_shape(Vector2f p);
    // float randi(int min, int max)
    // {
    //     std::uniform_int_distribution<float> dist(min, max);
    //     return dist(generator);
    // }
    // float rand(float min, float max)
    // {
    //     std::uniform_real_distribution<float> dist(min, max);
    //     return dist(generator);
    // }
    // float randn(float std)
    // {
    //     std::normal_distribution<float> dist(0.0, std);
    //     return dist(generator);
    // }
    b2World                     world;
    DebugDraw                   *dd;
    Camera                      *cam;
    Sim_params                  &sp;
    
    std::vector<Robot*>         robots;
    // std::vector<float>          system_error;
    std::vector<float>          system_gbp_error;
    std::vector<float>          system_rpos_error;
    std::vector<float>          system_ideal_error;
    std::vector<float>          system_algo2_error;
    std::vector<float>          system_bias_mag;
    std::vector<float>          system_box_gbp_est;
    std::vector<float>          system_box_est;
    std::vector<std::vector<float> >    system_box_state;
    Vector2f                    system_bias;
    int                         sensed;
    float                       time_since_step;
    float                       total_time;
    std::vector<Vector2f>       observed_box_state;
    float                       ticks_to_update;

    float                       hs_bandwidth;
    float                       bb_bandwidth;
    float                       flops_create;
    float                       flops_update;

    // For GUI
    float                       se;
    float                       re;
    bool                        placed;

    std::map<int,int>           global_box_obs;
    int                         oldest_box;
    std::vector<std::pair<int,int> > sortbox;
    std::vector<std::pair<int,int> > popbox;

    std::vector<Static_box*>    static_boxes;

    // ttf_t                       *font = NULL;
private:
    friend class Robot;
    std::vector<b2Body*>        static_bodies;
    std::vector<b2Body*>        dynamic_bodies;
    std::vector<Static_box*>    carrier_boxes;
    Contact_listener            cl;
    std::default_random_engine  generator;
    int                         moving_box;

    BT::BehaviorTreeFactory     bt_factory;

    
};



#endif