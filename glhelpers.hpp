//
//  glhelpers.hpp
//  gles2play
//
//  Created by Simon Jones on 20/03/2022.
//

#include <time.h> //yingfei

#ifndef glhelpers_hpp
#define glhelpers_hpp

#include <stdint.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>



#include <stdio.h>
#include <string>
#include <vector>
#include <map>
#include <stdexcept>
#include "stb_image.h"


#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"




class Timer
{
public:
    Timer(){verbose=false;}
    void set_time(bool v=false)
    {
        verbose = v;
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        t0 = t = ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
        if (verbose) printf("set:%llu\n", t);
    }
    int diff_time()
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t d = ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
        int delta = d - t;
        if (verbose) printf("diff:%llu %llu %d\n", d, t, delta);
        t = d;
        return delta;
    }
    int elapsed_time(bool reset=false)
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t d = ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
        int delta = d - t0;
        if (verbose) printf("diff:%llu %llu %d\n", d, t0, delta);
        if (reset) t0 = t = d;
        return delta;
    }
private:
    uint64_t t0, t;
    bool verbose;
};






#define GLCHECK(...) {if(glGetError()){printf(__VA_ARGS__);exit(1);}}




class Texture;
class Framebuffer
{
public:
    Framebuffer();
    void bind(Texture &t);
    void gen();
private:
    GLuint fb;
};

class State
{
public:
    State(std::string name, int x, int y, int width, int height);
    ~State(){}
    
    int window_should_close()
    {
        int vp[4];
        glGetIntegerv(GL_VIEWPORT, vp);
        vpwidth = vp[2];
        vpheight = vp[3];
        return glfwWindowShouldClose(window);
    }
    void swap_buffers()
    {
        glfwSwapBuffers(window);
    }
    void bind_fb_screen();
    void poll_events()
    {
        glfwPollEvents();
    }
    void terminate()
    {
        glfwTerminate();
    }

    void setup_perf_counters();
    void clear_perf_counters();
    void report_perf_counters(int y, int x);


    //GLuint create_program(const char *vs, const char *fs);
    //std::map<std::string, GLuint> attribute_map, uniform_map;
    int vpwidth, vpheight;
    GLFWwindow *window;
    GLuint vao;
    Framebuffer fbo;
    // Attribute and uniform map from name to addr
    
    
};

class Program
{
public:
    Program(const char *vs, const char *fs, const char *label);
    ~Program(){}
    void use_program()
    {
        glUseProgram(p);
    }
    void set_uniform(std::string name, std::vector<float> v);
    std::map<std::string, GLuint> attribute_map, uniform_map;
    std::map<std::string, std::vector<float> > uniforms;
private:
    int res;
    GLuint p;
};


class Mesh
{
public:
    Mesh(
         const std::vector<std::string> pack,
         const std::vector<int> _asize);
    Mesh(
         const std::vector<std::string> pack,
         const std::vector<int> _asize,
         std::vector<float> vertices);
    ~Mesh();
    void draw(Program &p, GLuint mode=GL_TRIANGLE_STRIP);
    void set_vertices(std::vector<float> vertices);
private:
    //Program     &p;
    GLuint      vbo_id;
//    VertexType  type;
    std::vector<std::string> packing;
    std::vector<int> asize;
    unsigned    floats_per_vertex;
    unsigned    vertex_count;
};


class Texture
{
public:
    Texture(State &_state, int w, int h, bool interp);
    void bind(GLuint tunit)
    {
        glActiveTexture(tunit);
        glBindTexture(GL_TEXTURE_2D, texture);
    }
    void load_texture_data(const char *fname);
    void set_texture_data(uint8_t *data);
    GLuint texture;
    int width, height, channels;
    // Structure to hold info for shared videocore buffer that will back the texture
    State &state;
private:
};




void render_pass(Framebuffer &f, Mesh &m, Program &p, Texture &in, Texture &out, 
                GLuint mode=GL_TRIANGLE_STRIP);


int pot(int i);


void dump(State &state);




#endif /* glhelpers_hpp */