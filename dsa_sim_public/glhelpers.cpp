//
//  glhelpers.cpp
//  gles2play
//
//  Created by Simon Jones on 20/03/2022.
//
#include "glhelpers.hpp"

#include <time.h> //yingfei

State::State(std::string name, int x, int y, int width, int height)
{


    //-------------------------------------------------
    printf("About to init\n");
    if (!glfwInit())
    {
        printf("glfw failed to init\n");
        exit(1);
    }
    glfwDefaultWindowHints();
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_COCOA_RETINA_FRAMEBUFFER, GL_FALSE);
    glfwWindowHint(GLFW_SAMPLES, 4);
    window = glfwCreateWindow(width, height, name.c_str(), 0, 0);
    glfwMakeContextCurrent(window);
    if (!window)
    {
        printf("Failed to get window\n");
        glfwTerminate();
        exit(1);
    }
    if (glewInit() != GLEW_OK)
    {
        printf("glew failed to init\n");
        glfwTerminate();
        exit(1);
    }
    glfwSwapInterval(1);
    glEnable(GL_MULTISAMPLE);  


    
    


    printf("Got window and make current\n");

    const GLubyte *renderer     = glGetString(GL_RENDERER);
    const GLubyte *vendor       = glGetString(GL_VENDOR);
    const GLubyte *version      = glGetString(GL_VERSION);
    const GLubyte *glslVersion  = glGetString(GL_SHADING_LANGUAGE_VERSION);
     

     
    printf("GL Vendor            : %s\n", vendor);
    printf("GL Renderer          : %s\n", renderer);
    printf("GL Version (string)  : %s\n", version);
    printf("GLSL Version         : %s\n", glslVersion);

    int vp[4];
    glGetIntegerv(GL_VIEWPORT, vp);
    printf("%d %d %d %d\n", vp[0],vp[1],vp[2],vp[3]);
    vpwidth = vp[2];
    vpheight = vp[3];
    
    fbo.gen();
    
}
void State::bind_fb_screen()
{
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, vpwidth, vpheight);
    glClearColor(0.0, 0.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
}


const char *typestr(GLenum type)
{
    switch (type)
    {
        case GL_FLOAT: return "GL_FLOAT";
        case GL_FLOAT_VEC2: return "GL_FLOAT_VEC2";
        case GL_FLOAT_VEC3: return "GL_FLOAT_VEC3";
        case GL_FLOAT_VEC4: return "GL_FLOAT_VEC4";
        case GL_FLOAT_MAT2: return "GL_FLOAT_MAT2";
        case GL_FLOAT_MAT3: return "GL_FLOAT_MAT3";
        case GL_FLOAT_MAT4: return "GL_FLOAT_MAT4";
        default: return "undefined";
    }
}



Program::Program(const char *vs, const char *fs, const char *label)
{
    int bf[4];
    glGetIntegerv(GL_NUM_SHADER_BINARY_FORMATS, bf);
    printf("Shader bin formats %d\n", bf[0]);
    GLuint v = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(v, 1, &vs, 0);
    glCompileShader(v);
    int res = GL_TRUE;
    glGetShaderiv(v, GL_COMPILE_STATUS, &res);
    if (res != GL_TRUE)
    {
        char log[1024];
        glGetShaderInfoLog(v, 1024, 0, log);
        printf("VShader compilation failed!\n%s\n", log);
        exit(1);
    }
    GLuint f = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(f, 1, &fs, 0);
    glCompileShader(f);
    glGetShaderiv(f, GL_COMPILE_STATUS, &res);
    if (res != GL_TRUE)
    {
        char log[1024];
        glGetShaderInfoLog(f, 1024, 0, log);
        printf("FShader compilation failed!\n%s\n", log);
        exit(1);
    }
    p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    glGetProgramiv(p, GL_LINK_STATUS, &res);
    if (res != GL_TRUE)
    {
        char log[1024];
        glGetProgramInfoLog(p, 1024, 0, log);
        printf("Program link failed! %s\n%s\n", label, log);
        exit(1);
    }
    
    char name[1024];
    int size;
    GLenum type;
    printf("Active attributes\n");
    glGetProgramiv(p, GL_ACTIVE_ATTRIBUTES, &res);
    for(int i = 0; i < res; i++)
    {
        glGetActiveAttrib(p, i, 1024, 0, &size, &type, name);
        glBindAttribLocation(p, i, name);
        attribute_map[std::string(name)] = i;
        printf("%d %s %s bound at %d\n", size, typestr(type), name, attribute_map[std::string(name)]);
    }
    
    printf("Active uniforms\n");
    glGetProgramiv(p, GL_ACTIVE_UNIFORMS, &res);
    for(int i = 0; i < res; i++)
    {
        glGetActiveUniform(p, i, 1024, 0, &size, &type, name);
        int loc = glGetUniformLocation(p, name);
        if (loc == -1)
        {
            printf("GetUniformLocation failed %d %s\n", i, name);
            exit(1);
        }
        uniform_map[std::string(name)] = loc;
        printf("%d %s %s location %d\n", size, typestr(type), name, loc);
    }
}


Mesh::Mesh(const std::vector<std::string> pack, const std::vector<int> _asize, 
            std::vector<float> vertices)
: asize(_asize)
{
    packing = pack;
    floats_per_vertex = 0;
    for (int i = 0; i < packing.size(); i++)
    {
        floats_per_vertex += asize[i];
    }
    vertex_count = (unsigned)vertices.size() / floats_per_vertex;
    
    // Set up vertex buffer
    glGenBuffers(1, &vbo_id);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
    GLCHECK("buffer\n");
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    //printf("ID: %d Vertex count: %d  floats per vertex: %d\n", vbo_id, vertex_count, floats_per_vertex);
}
Mesh::~Mesh()
{
    glDeleteBuffers(1, &vbo_id);
}
Mesh::Mesh(const std::vector<std::string> pack, const std::vector<int> _asize)
: asize(_asize)
{
    packing = pack;
    floats_per_vertex = 0;
    for (int i = 0; i < packing.size(); i++)
    {
        floats_per_vertex += asize[i];
    }
    //vertex_count = (unsigned)vertices.size() / floats_per_vertex;
    
    // Set up vertex buffer
    glGenBuffers(1, &vbo_id);
    //glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
    //glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
    //GLCHECK("buffer\n");
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
void Mesh::set_vertices(std::vector<float> vertices)
{
    vertex_count = (unsigned)vertices.size() / floats_per_vertex;
    glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * vertices.size(), &vertices[0], GL_STATIC_DRAW);
    GLCHECK("buffer\n");
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
void Mesh::draw(Program &p, GLuint mode)
{
    glBindBuffer(GL_ARRAY_BUFFER, vbo_id);
    GLCHECK("draw:bind\n");

    GLCHECK("bind\n");
    unsigned offset = 0;
    for (int i = 0; i < packing.size(); i++)
    {
        GLuint addr = p.attribute_map[packing[i]];
        glVertexAttribPointer(addr,
                              asize[i],
                              GL_FLOAT,
                              GL_FALSE,
                              sizeof(float) * floats_per_vertex,
                              (void *)(sizeof(float) * offset));
        GLCHECK("draw:attrptr\n");
        glEnableVertexAttribArray(addr);
        GLCHECK("draw:attrarray\n");
        offset += asize[i];
    }
    p.use_program();
    GLCHECK("draw:program\n");
    for (auto u : p.uniforms)
    {
        auto uname = u.first;
        auto uval = u.second;
        int s = (int)uval.size();
        //printf("%s %f %d\n", uname.c_str(),uval[0],s);
        if (s==1) glUniform1f(p.uniform_map[uname], uval[0]);
        else if (s==2) glUniform2f(p.uniform_map[uname], uval[0], uval[1]);
        else if (s==3) glUniform3f(p.uniform_map[uname], uval[0], uval[1], uval[2]);
        else if (s==4) glUniform4f(p.uniform_map[uname], uval[0], uval[1], uval[2], uval[3]);
    }
    GLCHECK("draw:uniforms\n");
    glDrawArrays(mode, 0, vertex_count);
    GLCHECK("draw:drawarrays %d %d\n", mode, vertex_count);
}

void Program::set_uniform(std::string name, std::vector<float> v)
{
    uniforms[name] = v;
}

int pot(int i)
{
    int j = 1;
    while (j < i) j *= 2;
    return j;
}

Texture::Texture(State &_state, int w, int h, bool interp) : state(_state)
{
    width   = w;
    height  = h;


    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
    if (interp)
    {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    }
    else
    {
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    }
    //glEGLImageTargetTexture2DOES(GL_TEXTURE_2D, egl_buffer);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Texture::load_texture_data(const char *fname)
{
    int w, h, c;
    uint8_t *data = stbi_load(fname, &w, &h, &c, 4);
    if (w != width || h != height)
    {
        printf("Load texture mismatch in dimensions: %d %d %d %d\n", w, width, h, height);
        exit(1);
    }
    set_texture_data(data);
    stbi_image_free(data);
}

void Texture::set_texture_data(uint8_t *data)
{
}


Framebuffer::Framebuffer()
{
    printf("created framebuffer, addr is %llx\n", (uint64_t)&fb);
}
void Framebuffer::gen()
{
    glGenFramebuffers(1, &fb);
}

void Framebuffer::bind(Texture &t)
{
    glBindFramebuffer(GL_FRAMEBUFFER, fb);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, t.texture, 0);
    if(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE)
    {
        //printf("fbo %d complete\n", fb);
    }
    else
    {
        printf("fbo %d not complete!\n", fb);
        exit(1);
    }
    glViewport(0, 0, t.width, t.height);
}




void render_pass(Framebuffer &f, Mesh &m, Program &p, Texture &in, Texture &out,
    GLuint mode)
{
    in.bind(GL_TEXTURE0);
    f.bind(out);
    glClearColor(0.0, 1.0, 0.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    m.draw(p, mode);
    //glFinish();
}