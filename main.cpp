#include "time.h"

#include "glhelpers.hpp"
#include "gui.hpp"
#include "sim.hpp"
#include "ttf2mesh.h"

#define _POSIX_C_SOURCE 200809L

#define PERROR(...) {printf(__VA_ARGS__); exit(1);}

// https://stackoverflow.com/questions/7616511/calculate-mean-and-standard-deviation-from-a-vector-of-samples-in-c-using-boos#:~:text=stddev%20%3D%20std%3A%3Asqrt(variance_n,Hope%20this%20helps!
float stddev(std::vector<float> const & func)
{
    float mean = std::accumulate(func.begin(), func.end(), 0.0) / func.size();
    float sq_sum = std::inner_product(func.begin(), func.end(), func.begin(), 0.0,
        [](float const & x, float const & y) { return x + y; },
        [mean](float const & x, float const & y) { return (x - mean)*(y - mean); });
    return std::sqrt(sq_sum / func.size());
}

int main(int argc, char **argv)
{

    
    
    
    Sim_params  sp;
    
    bool text_mode = false;
    int seed = 1;
    int timesteps = 21600;
    int capture = 18000;
    bool astop  = false;
    int dw      = 1024;
    int dh      = 900;
    std::string ofile = "";
    for(int i = 0; i < argc; i++)
    {
        if (!strcmp(argv[i], "-h"))
        {
            printf(R"help(
Usage: sim [options]
    -t          text mode
    -sve <val>  sigma v_est        (0.1)
    -spe <val>  sigma p_est        (0.02)
    -svn <val>  sigma v_noise      (0.1)
    -spn <val>  sigma p_noise      (0.02)
    -r  <val>   sense radius        (0.5)
    -s  <val>   random seed         (1)
    -n  <val>   number of robots    (2)
    -c  <val>   number of carriers  (5)
    -as <val>   arena size          (5)
    -hw <val>   history window      (10)
    -ht <val>   history time        (5)
    -it <val>   iterations per tick (1)
    -o  <file>  output file
    -d  <val>   sim duration (seconds)
    -dc <val>   capture time (seconds)
    -m          enable box movement
    -mv <val>   mean box velocity
    -mx <val>   box move distance
    -bc <val>   sigma boxcopy
    -an <val>   sigma anchor
    -da <val>   damping
    -ct <val>   controller index
            0   random walk
            1   seek boxes
            2   shape
    -sf <file>  shape file (png) with precomputed gradients
    -ssn <val>  sigma seek noise
    -tpu <val>  ticks per update
    -a          auto stop after convergence
    -ac <val>   speedup factor
    -dw <val>   display width   (1024)
    -dh <val>   display height  (900)

)help");
            exit(0);
        }
        else if (!strcmp(argv[i], "-t"))
        {
            text_mode = true;
        }
        else if (!strcmp(argv[i], "-a"))
        {
            astop = true;
        }
        else if (!strcmp(argv[i], "-sve"))
        {
            if (i < argc - 1)
                sp.sigma_vsense_est = atof(argv[++i]);
            else
                PERROR("Missing sve val!\n");
        }
        else if (!strcmp(argv[i], "-spe"))
        {
            if (i < argc - 1)
                sp.sigma_psense_est = atof(argv[++i]);
            else
                PERROR("Missing spe val!\n");
        }
        else if (!strcmp(argv[i], "-svn"))
        {
            if (i < argc - 1)
                sp.sigma_vsense_noise = atof(argv[++i]);
            else
                PERROR("Missing sve val!\n");
        }
        else if (!strcmp(argv[i], "-spn"))
        {
            if (i < argc - 1)
                sp.sigma_psense_noise = atof(argv[++i]);
            else
                PERROR("Missing spe val!\n");
        }
        else if (!strcmp(argv[i], "-r"))
        {
            if (i < argc - 1)
                sp.r_sense = atof(argv[++i]);
            else
                PERROR("Missing sp val!\n");
        }
        else if (!strcmp(argv[i], "-as"))
        {
            if (i < argc - 1)
                sp.arena_size = atof(argv[++i]);
            else
                PERROR("Missing arena size val!\n");
        }
        else if (!strcmp(argv[i], "-s"))
        {
            if (i < argc - 1)
                seed = atoi(argv[++i]);
            else
                PERROR("Missing seed val!\n");
        }
        else if (!strcmp(argv[i], "-it"))
        {
            if (i < argc - 1)
                sp.gbp_iters = atoi(argv[++i]);
            else
                PERROR("Missing iters val!\n");
        }
        else if (!strcmp(argv[i], "-n"))
        {
            if (i < argc - 1)
                sp.num_robots = atoi(argv[++i]);
            else
                PERROR("Missing number of robots!\n");
        }
        else if (!strcmp(argv[i], "-c"))
        {
            if (i < argc - 1)
                sp.num_boxes = atoi(argv[++i]);
            else
                PERROR("Missing number of carriers!\n");
        }
        else if (!strcmp(argv[i], "-hw"))
        {
            if (i < argc - 1)
                sp.fg_window = atoi(argv[++i]);
            else
                PERROR("Missing history window!\n");
        }
        else if (!strcmp(argv[i], "-ac"))
        {
            if (i < argc - 1)
                sp.speed = atoi(argv[++i]);
            else
                PERROR("Missing speedup factor!\n");
        }
        else if (!strcmp(argv[i], "-ht"))
        {
            if (i < argc - 1)
                sp.fg_history_time = atof(argv[++i]);
            else
                PERROR("Missing history time!\n");
        }
        else if (!strcmp(argv[i], "-d"))
        {
            if (i < argc - 1)
                timesteps = atoi(argv[++i]) * 60;
            else
                PERROR("Missing duration val!\n");
        }
        else if (!strcmp(argv[i], "-dw"))
        {
            if (i < argc - 1)
                dw = atoi(argv[++i]);
            else
                PERROR("Missing display width!\n");
        }
        else if (!strcmp(argv[i], "-dh"))
        {
            if (i < argc - 1)
                dh = atoi(argv[++i]);
            else
                PERROR("Missing display width!\n");
        }
        else if (!strcmp(argv[i], "-dc"))
        {
            if (i < argc - 1)
                capture = atoi(argv[++i]) * 60;
            else
                PERROR("Missing duration val!\n");
        }
        else if (!strcmp(argv[i], "-o"))
        {
            if (i < argc - 1)
                ofile = std::string(argv[++i]);
            else
                PERROR("Missing csv output file!\n");
        }
        else if (!strcmp(argv[i], "-sf"))
        {
            if (i < argc - 1)
                sp.shape_file = std::string(argv[++i]);
            else
                PERROR("Missing shape file!\n");
        }
        else if (!strcmp(argv[i], "-m"))
        {
            sp.rmove_en = true;
        }
        else if (!strcmp(argv[i], "-mv"))
        {
            if (i < argc - 1)
                sp.box_vel = atof(argv[++i]);
            else
                PERROR("Missing box velocity value!\n");
        }
        else if (!strcmp(argv[i], "-mx"))
        {
            if (i < argc - 1)
                sp.box_dist = atof(argv[++i]);
            else
                PERROR("Missing box move dist value!\n");
        }
        else if (!strcmp(argv[i], "-bc"))
        {
            if (i < argc - 1)
                sp.sigma_boxcopy_est = atof(argv[++i]);
            else
                PERROR("Missing boxcop sigma!\n");
        }
        else if (!strcmp(argv[i], "-an"))
        {
            if (i < argc - 1)
                sp.sigma_anchor_est = atof(argv[++i]);
            else
                PERROR("Missing anchor sigma!\n");
        }
        else if (!strcmp(argv[i], "-tpu"))
        {
            if (i < argc - 1)
                sp.ticks_per_update = atof(argv[++i]);
            else
                PERROR("Missing anchor sigma!\n");
        }
        else if (!strcmp(argv[i], "-da"))
        {
            if (i < argc - 1)
                sp.damping = atof(argv[++i]);
            else
                PERROR("Missing damping!\n");
        }
        else if (!strcmp(argv[i], "-ct"))
        {
            sp.ctrl_idx = atoi(argv[++i]);
        }
        else if (!strcmp(argv[i], "-ssn"))
        {
            if (i < argc - 1)
                sp.sigma_seek_noise = atof(argv[++i]);
            else
                PERROR("Missing boxcop sigma!\n");
        }
        else if (!strcmp(argv[i], "-ss")) // yingfei
        {
            if (i < argc - 1)
                sp.start_size = atof(argv[++i]);
            else
                PERROR("Missing start size val!\n");
        }
    }
    


    int     tconv       = 0;
    int     converged   = 0;

    if (!text_mode)
    {
        State       state("Sim", 0, 0, dw, dh);
        Gui         gui(state, sp);
        // Sim         *sim = new Sim(sp, gui.seed);
        Sim         *sim = new Sim(sp, seed);
        gui.set_sim(sim);
        
        float frac_accum    = 0.0;
        int counter         = 0;
        
        Timer master;
        master.set_time();
        
        while (!state.window_should_close())
        {
            // Poll for and process events. When resizing window,
            // poll_events is never left, window updates need to take
            // place in callback
            state.poll_events();
            
            
            gui.new_frame();
            gui.ui();
            
            if (gui.reset)
            {
                delete sim;
                sim             = new Sim(sp, gui.seed);
                counter         = 0;
                converged       = 0;
                tconv           = 0;
                sp.move_counter = 0;
                sp.move_state   = 0;
                gui.set_sim(sim);
                gui.reset       = false;
            }
            if (not gui.pause || gui.step)
            {

                while (frac_accum > 0)
                {
                    frac_accum -= 1.0 / (60 * sp.speed);
                    
                    sim->step();
                    
                    if (sim->system_rpos_error.back() < sim->sp.sigma_psense_noise * 2)
                    {
                        converged++;
                        if (converged >= 60 && tconv == 0)
                        {
                            tconv = counter;
                            printf("System converged %.2f\n", sim->total_time);
                        }
                    }
                }
                frac_accum += 1.0 / 60;

            }

            // Turn the wall on or off
            if (sp.gui.wall && sim->static_boxes.size() == 4)
            {
                sim->static_boxes.push_back(new Static_box(sim, b2Vec2(0.0f, 0.0f), sp.r_sense, sp.arena_size, Otype::WALL));
                printf("Adding wall\n");
            }
            if (!sp.gui.wall && sim->static_boxes.size() == 5)
            {
                delete(sim->static_boxes.back());
                sim->static_boxes.pop_back();
                printf("Removing wall\n");
            }

            if (gui.cap)
            {
                // Save the current sim settings
                printf("seed %d num_robots %d num_boxes %d arena_size %f speed %f decay_time %d lighten %f alpha %f ticks_per_update %d\n",
                gui.seed, sp.num_robots, sp.num_boxes, sp.arena_size, sp.speed, sp.decay_time, sp.lighten, sp.alpha, sp.ticks_per_update);
                printf("zoom %f width %d height %d\n", sim->cam->m_zoom, gui.width, gui.height);
                printf("box %d robots %d fg %d sensor %d sense_line %d msg %d factor %d ebpos %d shape %d name %d origin %d estats %d rbias %d\n", 
                sp.gui.box, sp.gui.robots, sp.gui.fg, sp.gui.sensor, sp.gui.sense_line, sp.gui.msg, sp.gui.factor, sp.gui.ebpos, sp.gui.shape,
                sp.gui.name, sp.gui.origin, sp.gui.estats, sp.gui.rbias);
            }

            gui.render();
            
            // Swap front and back buffers
            state.swap_buffers();
            
            // Something wrong with vsync on Ventura with OpenGL; swap buffers
            // causes a fluctuating 8-12 ish ms frame period. Put this hack in
            // as a temp fix
            while(master.elapsed_time() < 16666);
            int t = master.elapsed_time(true);
            //printf("%d\n", t);
            counter++;
        }
        state.terminate();
    }
    else
    {
        // printf("robots %d\n", sp.num_robots);
        Sim     *sim    = new Sim(sp, seed);
        FILE    *fp     = 0;
        if (ofile.size())
        {
            // printf("opening file %s\n", ofile.c_str());
            fp = fopen(ofile.c_str(), "w");
            sim->sp.fp = fp;
        }


        //printf("Running for %d timesteps\n", timesteps);
        // printf("Placement %d\n", sim->placed);
        if(sim->placed)
            for(int i = 0; i < timesteps; i++)
            {


                sim->step();
                if (sim->system_rpos_error.back() < sim->sp.sigma_psense_noise * 2)
                {
                    converged++;
                    if (converged >= 60 && tconv == 0)
                    {
                        tconv = i;
                        //printf("System converged %.2f\n", sim->total_time);
                    }
                }
                if (astop && tconv && (i > tconv + capture))
                {
                    break;
                }
                
                if (fp && (i % 60 == 0))
                {
                    fprintf(fp, "%6d ", i);
                    // for(int j = 0; j < sp.num_boxes; j++)
                        fprintf(fp, "%12.6f %12.6f ", sim->system_rpos_error.back(), sim->system_algo2_error.back());
                    fprintf(fp, "\n");

                }

            }
        else
        {

            for(int i = 0; i < argc; i++)
                printf("%s ", argv[i]);
            printf("time: 0.0 tconv: 0.0 rconv 0.0 rcstd 0.0 arena: 0.0 rpos: 0.0 ideal: 0.0 algo2: 0.0 fprc: 0.0 fpru: 0.0 bwpr: 0.0\n" );
            exit(0);
        }

        auto &ie = sim->system_ideal_error;
        int start = ie.size() - capture - 1;
        float iemean = 0;
        for (int i = 0; i < capture; i++)
        {
            //printf("%6d %12.5f\n", i, se[i + start]);
            iemean += ie[start + i];
        }
        iemean /= capture;


        auto &ae = sim->system_algo2_error;
        float aemean = 0;
        for (int i = 0; i < capture; i++)
        {
            //printf("%6d %12.5f\n", i, se[i + start]);
            aemean += ae[start + i];
        }
        aemean /= capture;


        auto &ce = sim->system_rpos_error;
        float cemean = 0;
        for (int i = 0; i < capture; i++)
        {
            //printf("%6d %12.5f\n", i, se[i + start]);
            cemean += ce[start + i];
        }
        cemean /= capture;


        int total_ticks = tconv + capture;
        if (tconv == 0)
            total_ticks = timesteps;
        for(int i = 0; i < argc; i++)
            printf("%s ", argv[i]);

        printf("time: %8.1f tconv: %8.1f rconv %8.1f rcstd %7.2f arena: %8.2f rpos: %8.4f ideal: %8.4f algo2: %8.4f fprc: %8.4f fpru: %8.4f bwpr: %8.4f\n", 
            (float)(total_ticks) / 60, (float)tconv / 60, 0.0, 0.0, sim->sp.arena_size, cemean, iemean, aemean, 
            sim->flops_create, sim->flops_update, sim->hs_bandwidth);

        if (fp)
            fclose(fp);
        
    }

}



extern "C" int testlib()
{   
    printf("This is a c function\n");
    return 23;
}



extern "C" Sim* sim_ptr(Sim_params sp)
{
    return new Sim(sp);
}