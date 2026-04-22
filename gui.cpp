
#include "gui.hpp"
#include "draw.h"
#include <iostream>

DebugDraw g_debugDraw;
Camera g_camera;

static void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    glViewport(0, 0, width, height);
    ((Gui *)glfwGetWindowUserPointer(window))->set_framebuffer_size(width, height);
    ((Gui *)glfwGetWindowUserPointer(window))->new_frame();
    ((Gui *)glfwGetWindowUserPointer(window))->ui();
    ((Gui *)glfwGetWindowUserPointer(window))->render();
    ((Gui *)glfwGetWindowUserPointer(window))->swap_buffers();
}

static void scroll_callback(GLFWwindow *window, double dx, double dy)
{
    if (dy > 0)
    {
        g_camera.m_zoom /= 1.05f;
    }
    else
    {
        g_camera.m_zoom *= 1.05f;
    }
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    //printf("key press %d\n", key);
    auto &key_down = ((Gui *)glfwGetWindowUserPointer(window))->key_down;
    if (action == GLFW_PRESS)
        key_down[key] = true;
    if (action == GLFW_RELEASE)
        key_down[key] = false;
    if (action == GLFW_REPEAT)
        return;
    // for(int i = 0; i < key_down.size(); i++)
    //     printf("%c", key_down[i] ? '1' : '0');
    // printf("\n");
}

static void mouse_cursor_callback(GLFWwindow *window, double xpos, double ypos)
{
    static double sx, sy;
    static b2Vec2 centre;
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT))
    {
        if (((Gui *)glfwGetWindowUserPointer(window))->dragstart)
        {
            sx = xpos;
            sy = ypos;
            centre = g_camera.m_center;
            ((Gui *)glfwGetWindowUserPointer(window))->dragstart = false;
        }
        // Factor 50 is due to the code building the projection matrix, which uses 25.0 as 
        // half screen height
        auto z = g_camera.m_zoom * 50 / g_camera.m_height;
        b2Vec2 pan_scr = b2Vec2(z * (xpos - sx), z * (sy - ypos));
        g_camera.m_center = centre - pan_scr;
    }
}

static void mouse_button_callback(GLFWwindow *window, int button, int action, int mods)
{
    if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS)
    {
        ((Gui *)glfwGetWindowUserPointer(window))->dragstart = true;
    }
}







Gui::Gui(State &_state, Sim_params &_sp) : state(_state), sp(_sp)
{
    ImGui::CreateContext();
    ImPlot::CreateContext();
    io = ImGui::GetIO();
    // io.Fonts->AddFontFromFileTTF("../src/Menlo-Regular.ttf", 13);
    //yingfei io.Fonts->AddFontFromFileTTF("/System/Library/Fonts/HelveticaNeue.ttc", 13);
    io.Fonts->AddFontFromFileTTF("/home/yingfei/dsa_sim_public/src/Menlo-Regular.ttf", 13);
    // yingfei fixed = io.Fonts->AddFontFromFileTTF("/System/Library/Fonts/Menlo.ttc", 13);
    fixed = io.Fonts->AddFontFromFileTTF("/home/yingfei/dsa_sim_public/src/Menlo-Regular.ttf", 13);
    //io.Fonts->AddFontFromFileTTF("/System/Library/Fonts/HelveticaNeue.ttc", 13);
    //fixed = io.Fonts->AddFontFromFileTTF("/System/Library/Fonts/Menlo.ttc", 13);
    ImGui::StyleColorsLight();
    
    // Callbacks must be set up before ImGui init so that chaining works
    glfwSetWindowUserPointer(state.window, this);
    glfwSetFramebufferSizeCallback(state.window, framebuffer_size_callback);
    glfwSetScrollCallback(state.window, scroll_callback);
    glfwSetKeyCallback(state.window, key_callback);
    glfwSetCursorPosCallback(state.window, mouse_cursor_callback);
    glfwSetMouseButtonCallback(state.window, mouse_button_callback);
    
    ImGui_ImplGlfw_InitForOpenGL(state.window, true);
    ImGui_ImplOpenGL3_Init("#version 150");


    g_debugDraw.Create();
    //g_debugDraw.SetFlags(b2Draw::e_shapeBit);
    g_debugDraw.SetFlags(0xff);
    g_camera.ResetView();
    g_camera.m_zoom *= 0.6;
    // g_camera.m_center += b2Vec2(0,0.35);
   
    //set_sim(_sim);
    
    key_down.resize(GLFW_KEY_LAST, 0);
    
    for(int i = 0; i < 10; rplot_flags[i++] = 1);
    
    show_fg = true;
    //dd_flags = {0};

    //yingfei    
    int ret = ttf_load_from_file("/home/yingfei/dsa_sim_public/src/Menlo-Regular.ttf", &(g_debugDraw.font), false);
    if (g_debugDraw.font == NULL) 
    {
        printf("Font not loaded %d\n", ret);
        exit(1);
    }

    printf("font \"%s\" loaded\n", g_debugDraw.font->names.full_name);

}

void Gui::set_sim(Sim *_sim)
{
    sim = _sim;
    sim->dd = &g_debugDraw;
    sim->cam = &g_camera;
    sim->world.SetDebugDraw(&g_debugDraw);
}

// #include <memory>
// template<typename ... Args>
// std::string string_format( const std::string& format, Args ... args )
// {
//     size_t size = snprintf( nullptr, 0, format.c_str(), args ... ) + 1; // Extra space for '\0'
//     std::unique_ptr<char[]> buf( new char[ size ] );
//     snprintf( buf.get(), size, format.c_str(), args ... );
//     return std::string( buf.get(), buf.get() + size - 1 ); // We don't want the '\0' inside
// }

void Gui::new_frame()
{
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
}

void Gui::ui()
{
    //ImGui::ShowDemoWindow();


    ImVec2 fp = ImGui::GetStyle().FramePadding;
    ImVec2 wp = ImGui::GetStyle().WindowPadding;
    // printf("x:%f y:%f %f %f\n", fp.x, fp.y, wp.x, wp.y);
    float os = ctrl_width * 0.5;
    float mg = wp.x + fp.x * 2;
    ImVec2 bs(ctrl_width - mg, 0);


    ImGui::SetNextWindowPos(ImVec2(0,0));
    ImGui::SetNextWindowSize(ImVec2(100, ImGui::GetIO().DisplaySize.y));
    ImGui::Begin("Test", 0,
                 ImGuiWindowFlags_NoTitleBar
                 |ImGuiWindowFlags_NoMove
                 |ImGuiWindowFlags_AlwaysAutoResize);


    ImGui::Text("FPS");ImGui::SameLine(os);ImGui::Text("%.1f", ImGui::GetIO().Framerate);

    ImGui::PushItemWidth(ctrl_width - os - wp.x);
    if (ImGui::Button("Reset")) reset = true;ImGui::SameLine(os);ImGui::InputInt("##Seed", &seed, 0);
    ImGui::Text("Robots");ImGui::SameLine(os);ImGui::InputInt("##Robots", &sp.num_robots, 0);
    ImGui::Text("Boxes");ImGui::SameLine(os);ImGui::InputInt("##Boxes##1", &sp.num_boxes, 0);
    ImGui::Text("Size");ImGui::SameLine(os);ImGui::InputFloat("##Size", &sp.arena_size, 0);
    

    ImGui::PushItemWidth(bs.x);
    if (ImGui::Button(pause ? "Run" : "Pause", bs)) pause = not pause;
    step = ImGui::Button("Step", bs);
    cap = ImGui::Button("Capture", bs);
    ImGui::SliderFloat("##7", &sp.speed, 0.1, 100, "acc:%.1f", ImGuiSliderFlags_Logarithmic);

    if (ImGui::CollapsingHeader("GUI", ImGuiTreeNodeFlags_DefaultOpen))
    {
        // Get tiny checkboxes by reducing the frame padding
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0,0));
        ImGui::Checkbox("Boxes", &sp.gui.box);
        ImGui::Checkbox("Robots", &sp.gui.robots);
        ImGui::Checkbox("Factor graph", &sp.gui.fg);
        ImGui::Checkbox("Sense radius", &sp.gui.sensor);
        ImGui::Checkbox("Sense line", &sp.gui.sense_line);
        ImGui::Checkbox("Message", &sp.gui.msg);
        ImGui::Checkbox("Factor", &sp.gui.factor);
        ImGui::Checkbox("Est box pos", &sp.gui.ebpos);
        ImGui::Checkbox("Origin", &sp.gui.origin);
        ImGui::Checkbox("Oline", &sp.gui.oline);
        ImGui::Checkbox("Estats", &sp.gui.estats);
        ImGui::Checkbox("Shape", &sp.gui.shape);
        ImGui::Checkbox("Name", &sp.gui.name);
        ImGui::Checkbox("Robot bias", &sp.gui.rbias);
        ImGui::Checkbox("Oldest box", &sp.gui.obox);
        ImGui::Checkbox("Monochrome", &sp.gui.mono);
        ImGui::Checkbox("Aim oldest", &sp.gui.aim_oldest);
        ImGui::Checkbox("Wall", &sp.gui.wall);
        const char * items[] = {"rect", "circle", "cross", "wavy", "hline", "vline"};
        ImGui::Combo("##10combo", &sp.shape_idx, items, IM_ARRAYSIZE(items));
        ImGui::SliderInt("##i1", &sp.decay_time, 5, 120, "decay %d");
        ImGui::SliderFloat("##f1", &sp.lighten, 1.0, 2.0, "lighten %.1f");
        ImGui::SliderFloat("##f2", &sp.alpha, 0.0, 1.0, "alpha %.1f");
        ImGui::PopStyleVar();
    }


    if (ImGui::CollapsingHeader("Sim"))
    {
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0,0));
        ImGui::Checkbox("Ctrl on", &sp.ctrl_en);
        const char * items[] = {"Random walk", "Seek boxes", "Shape", "Sfile", "BT test", "Patrol"};
        ImGui::Combo("combo", &sp.ctrl_idx, items, IM_ARRAYSIZE(items));
        ImGui::Checkbox("Box move", &sp.rmove_en);
        ImGui::PopStyleVar();
        ImGui::SliderFloat("##8", &sp.agg_noise, 0.01f, 1.0f, "agg n %.3f");
        ImGui::SliderFloat("##8a", &sp.agg_vel, 0, 0.5, "agg_v %.2f");
        bool changed = ImGui::SliderFloat("##Move", &sp.box_vel, 0.001f, 0.1f, "box v %.3f");
        if (changed) sp.move_counter = 300;
        ImGui::Text("Sense model");
        ImGui::SliderFloat("##4", &sp.sigma_vsense_noise, 0.01, 1.0, "vel %.2f");
        ImGui::SliderFloat("##5", &sp.sigma_psense_noise, 0.01, 1.0, "pos %.2f");
        ImGui::SliderFloat("##6", &sp.r_sense, 0, 1.0, "r:%.2f");
        ImGui::SliderFloat("##9", &sp.repul, 0, 1.0, "repul %.2f");
    }


    if (ImGui::CollapsingHeader("GBP"))
    {
        ImGui::SliderFloat("##Damping", &sp.damping, 0.0f, 1.0f, "damp  %.2f");
        ImGui::SliderInt("##window", &sp.fg_window, 2, 20, "window  %2d");
        ImGui::SliderInt("##iter", &sp.ticks_per_update, 1, 60, "tpu  %d");
        ImGui::SliderFloat("##nt", &sp.fg_history_time, 1, 30, "hist  %2f");
        sp.new_node_time = sp.fg_history_time / sp.fg_window;

        ImGui::Text("Estimation");
        ImGui::SliderFloat("##0", &sp.sigma_vsense_est, 0.01, 1.0, "vel %.2f");
        ImGui::SliderFloat("##1", &sp.sigma_psense_est, 0.01, 1.0, "pos %.2f");
        ImGui::SliderFloat("##2", &sp.sigma_boxcopy_est, 0.01, 10.0, "box %.2f");
        ImGui::SliderFloat("##3", &sp.sigma_anchor_est, 0.1, 5.0, "anch %.2f");
    }
    if (ImGui::CollapsingHeader("More.."))
    {
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0,0));
        ImGui::Checkbox("Debug fg", &sp.debug_fg);
        ImGui::Checkbox("Debug msg", &sp.debug_msgs);
        ImGui::PopStyleVar();
    }



    ImGui::PopItemWidth();
    ImGui::End();
    
    ImGui::SetNextWindowPos(ImVec2(100,0));
    ImGui::SetNextWindowSize(ImVec2(ImGui::GetIO().DisplaySize.x - 100, 200));
    ImGui::Begin("Test2", 0,
                 ImGuiWindowFlags_NoTitleBar
                 |ImGuiWindowFlags_NoMove
                 |ImGuiWindowFlags_AlwaysAutoResize);
    if (ImPlot::BeginPlot("Test3", ImVec2(-1,-1), ImPlotFlags_NoTitle|ImPlotFlags_NoLegend))
    {
        ImPlot::SetupAxis(ImAxis_X1, "X-axis", ImPlotAxisFlags_AutoFit|ImPlotAxisFlags_NoLabel);
        //ImPlot::SetupAxis(ImAxis_Y1, "Y-axis", ImPlotAxisFlags_AutoFit|ImPlotAxisFlags_NoLabel);
        ImPlot::SetupAxis(ImAxis_Y1, "Y-axis", ImPlotAxisFlags_NoLabel);
        // ImPlot::SetupAxisLimits(ImAxis_Y1, 0, sim->sp.max_error * 1.01);
        ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 0.25);
        ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 1);
        const unsigned long pts = 50000;
        

        auto &e1 = sim->system_rpos_error;
        unsigned long start = e1.size() < pts ? 0 : e1.size() - pts;
        ImPlot::SetNextLineStyle(ImVec4(0,1,0,1));
        ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 3);
        // ImPlot::PlotLine("System", e1.data() + start, e1.size() >= pts ? pts : e1.size());
        ImPlot::PlotLine("System", e1.data() + start, e1.size() >= pts ? pts : e1.size(), 1, start);

        // auto &e2 = sim->system_algo2_error;
        // start = e2.size() < pts ? 0 : e2.size() - pts;
        // ImPlot::SetNextLineStyle(ImVec4(1,0,0,1));
        // ImPlot::PushStyleVar(ImPlotStyleVar_LineWeight, 3);
        // ImPlot::PlotLine("System_gbp1", e2.data() + start, e2.size() >= pts ? pts : e2.size(), 1, start);

        ImPlot::EndPlot();
    }
    ImGui::End();

}

void Gui::draw_text(int id, float x, float y, std::string t, bool centre = false)
{
    b2Vec2 p = g_camera.ConvertWorldToScreen(b2Vec2(x, y));
    ImGui::SetNextWindowPos(ImVec2(p.x + ctrl_width, p.y + graph_height), ImGuiCond_Always, centre ? ImVec2(0.5, 0.5) : ImVec2(0, 0));
    ImGui::SetNextWindowBgAlpha(0.0f);
    ImGui::Begin(string_format("Window%d", id).c_str(), 0, ImGuiWindowFlags_NoDecoration 
                                            | ImGuiWindowFlags_NoBackground
                                            | ImGuiWindowFlags_AlwaysAutoResize 
                                            | ImGuiWindowFlags_NoSavedSettings 
                                            | ImGuiWindowFlags_NoFocusOnAppearing 
                                            | ImGuiWindowFlags_NoNav);
    ImGui::PushFont(fixed);
    ImGui::Text("%s", t.c_str());
    ImGui::PopFont();
    ImGui::End();

}
void Gui::render()
{
    glfwGetFramebufferSize(state.window, &width, &height);
    // printf("%d %d\n", width, height);
    // glClearColor(0.5, 0.5, 0.5, 1.0);
    glClearColor(1.0, 1.0, 1.0, 1.0);
    //glClearColor(0.45, 0.45, 0.45, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    
    int vp_width    = width - ctrl_width;
    int vp_height   = height - graph_height;
    glViewport(ctrl_width, 0, vp_width, vp_height);

    g_camera.m_width    = vp_width;
    g_camera.m_height   = vp_height;
    
    //g_debugDraw.DrawCircle(b2Vec2(0,0), 0.1, b2Color(255,255,255));
    int flags = 0;
    for(int i = 0; i < 5; i++)
        flags = (flags << 1) | dd_flags[4 - i];
    
    g_debugDraw.SetFlags(flags);
    
    sim->render(width - ctrl_width, height - graph_height, rplot_flags, show_fg);
    g_debugDraw.Flush();


    float scale     = g_camera.m_zoom * 1.5;
    float as2       = sp.arena_size / 2;
    if (sp.gui.estats)
    {    
        std::string s   = string_format("t      %6.1f\nr_error %.3f", sim->total_time, sim->re);
        if (sim->sp.num_boxes)
            s += string_format("\ns_error %.3f", sim->se);
        draw_text(0, -as2, as2, s);
    }
    // int id = 1;
    // for(auto &r : sim->robots)
    // {
    //     b2Vec2 p = r->body->GetPosition();
    //     draw_text(id++, p.x, p.y, string_format("%2d", r->name), true);
    // }

    
    // glViewport(0, 0, width, height);
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    
}