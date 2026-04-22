
#ifndef GUI_HPP
#define GUI_HPP

#include "glhelpers.hpp"
#include "sim.hpp"
#include "robot.hpp"

class Gui
{
public:
    Gui(State &_state, Sim_params &_sp);
    void ui();
    void new_frame();
    void render();
    void set_framebuffer_size(int w, int h) { width = w; height = h; }
    void swap_buffers() {state.swap_buffers();}
    void set_sim(Sim *s);
    void draw_text(int id, float x, float y, std::string t, bool centre);


    int                 width, height;
    std::vector<bool>   key_down;
    bool                dd_flags[5] = {0};
    int                 rplot_flags[10];
    bool                pause = true;
    bool                step;
    bool                cap;
    const std::vector<std::string> stateset = {"All", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9"};
    bool                reset = false;
    int                 seed;
    int                 robots;
    bool                dragstart = false;
    Sim_params          &sp;

    int                 ctrl_width      = 100;
    int                 graph_height    = 200;



    bool                show_fg;

private:
    State               &state;
    Sim                 *sim;
    ImGuiIO             io;
    ImFont              *fixed;
    

};



#endif