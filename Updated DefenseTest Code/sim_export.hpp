#ifndef SIM_EXPORT_HPP
#define SIM_EXPORT_HPP


struct Sim_params
{
    int     num_boxes_gui       = 5;
    int     num_robots_gui      = 10;
    float   arena_size          = 5.0;
    int     num_robots          = num_robots_gui;
    float   robot_vel           = 0.5;
    int     num_boxes           = num_boxes_gui;
    float   dt                  = 1.0 / 60;
    float   min_sep             = 0.3;
    float   sigma_vsense_est    = 0.1;  
    float   sigma_vsense_est_min = 0.01;  
    float   sigma_psense_est    = 0.02;
    float   sigma_boxcopy_est   = 1.0;
    float   sigma_anchor_est    = 1.0;
    float   sigma_vsense_noise  = 0.1;  
    float   sigma_psense_noise  = 0.02;
    float   sigma_prior         = 1.0;
    float   r_sense             = 0.5;
    float   max_error           = 7.0;
    int     speed               = 1;
    // commands
    bool    ctrl_en             = true;
    // GBP params
    int     gbp_iters           = 1;
    float   ticks_per_update    = 1;
    int     msg_schedule        = 0;
    float   damping             = 0.8;

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
    float   agg_noise       = 0;
    int     shape_idx       = 0;

    struct
    {
        bool    box         = true;
        bool    fg          = false;
        bool    sensor      = false;
        bool    sense_line  = true;
        bool    erpos       = false;
        bool    ebpos       = false;
        bool    shape       = false;
        bool    origin      = false;
        bool    detector    = true;   // PERSON B: detector overlay
    } gui;

    // PERSON B: CSV export path (written on sim exit)
    bool        export_csv  = true;
    std::string csv_path    = "detector_log.csv";
};


#endif