#include "sim.hpp"
#include "robot.hpp"
#include "draw.h"
#include "gbp.hpp"


// Initialise the global object ID. This will provide a unique id to each instance
// of robot
int Object::gid = 1;

Vector2f to_eigen(b2Vec2 v)
{
    return Vector2f(v.x, v.y);
}

Vector3f to_eigen3(b2Vec2 v, float th)
{
    return Vector3f(v.x, v.y, th);
}

b2Color hsl2rgb(float h, float s, float l)
{
    float c = (1.0 - fabs(2.0 * l - 1.0)) * s;
    //float c = fabs(2.0 * l - 1.0);
    float ht = h / 60;
    float x = c * (1 - fabs(fmod(ht, 2) - 1));
    //printf("h:%f s:%f l:%f c:%f ht:%f x:%f\n", h, s, l, c, ht, x);
    float r, g, b;
    if      (ht < 1) {r = c; g = x; b = 0;}
    else if (ht < 2) {r = x; g = c; b = 0;}
    else if (ht < 3) {r = 0; g = c; b = x;}
    else if (ht < 4) {r = 0; g = x; b = c;}
    else if (ht < 5) {r = x; g = 0; b = c;}
    else if (ht < 6) {r = c; g = 0; b = x;}
    float m = l - c / 2;
    return b2Color(r + m, g + m, b + m);
}

bool get_sensor_sensed(b2Contact *contact, uintptr_t &sensor, uintptr_t &sensed)
{
    b2Fixture *fa = contact->GetFixtureA();
    b2Fixture *fb = contact->GetFixtureB();
    // only trigger is only one fixture is a sensor
    if(!(fa->IsSensor() ^ fb->IsSensor()))
        return false;
    
    if (fa->IsSensor())
    {
        sensor = fa->GetBody()->GetUserData().pointer;
        sensed = fb->GetBody()->GetUserData().pointer;
    }
    else
    {
        sensor = fb->GetBody()->GetUserData().pointer;
        sensed = fa->GetBody()->GetUserData().pointer;
    }
    return true;
}

void Contact_listener::BeginContact(b2Contact *contact)
{
    uintptr_t sensor;
    uintptr_t sensed;
    if (get_sensor_sensed(contact, sensor, sensed))
        ((Robot*)sensor)->add_sensed_object(sensed);
}
void Contact_listener::EndContact(b2Contact *contact)
{
    uintptr_t sensor;
    uintptr_t sensed;
    if (get_sensor_sensed(contact, sensor, sensed))
        ((Robot*)sensor)->remove_sensed_object(sensed);
}

void Contact_listener::PreSolve(b2Contact *contact, const b2Manifold *oldManifold)
{
    // We are not using the solver for robot collisions, but regarding the robot
    // fixture as a 'sensor' for the purpose of controlling the robot. We explicitly
    // capture the collision velocity and the contact normal so we can simulate
    // arbitrary robot sensor behaviour. If we were to use the IsSensor property
    // on the robot body fixture, it is not possible to extract contact normal
    
    // If either fixture is a sensor, we allow the normal handling of the contact to
    // continue
    b2Fixture *fa = contact->GetFixtureA();
    b2Fixture *fb = contact->GetFixtureB();
    if (fa->IsSensor() || fb->IsSensor())
        return;
    
    
    b2WorldManifold worldManifold;
    contact->GetWorldManifold(&worldManifold);
 
    b2PointState state1[2], state2[2];
    b2GetPointStates(state1, state2, oldManifold, contact->GetManifold());
    
    //contact->SetEnabled(false);
    if (state2[0] == b2_addState)
    {
        b2Body *ba = fa->GetBody();
        b2Body *bb = fb->GetBody();
        b2Vec2 point = worldManifold.points[0];
        b2Vec2 va = ba->GetLinearVelocityFromWorldPoint(point);
        b2Vec2 vb = bb->GetLinearVelocityFromWorldPoint(point);
 
               
        if (((Object*)(ba->GetUserData().pointer))->otype == Otype::ROBOT)
        {
            ((Robot*)(ba->GetUserData().pointer))->col_vel = vb - va;
            ((Robot*)(ba->GetUserData().pointer))->contact_normal = worldManifold.normal;
            ((Robot*)(ba->GetUserData().pointer))->col_otype = ((Object*)(bb->GetUserData().pointer))->otype;
        }
        if (((Object*)(bb->GetUserData().pointer))->otype == Otype::ROBOT)
        {
            ((Robot*)(bb->GetUserData().pointer))->col_vel = va - vb;
            ((Robot*)(bb->GetUserData().pointer))->contact_normal = worldManifold.normal;
            ((Robot*)(bb->GetUserData().pointer))->col_otype = ((Object*)(ba->GetUserData().pointer))->otype;
        }
    }
}



Static_box::Static_box(Sim *_sim, b2Vec2 _pos, float _width, float _height, int _otype, int _name)
: sim(_sim), pos(_pos), width(_width), height(_height), name(_name)
{
    otype                   = _otype;
    b2BodyDef               body_def;
    b2PolygonShape          box;
    b2FixtureDef            fix;
    body_def.position.Set(pos.x, pos.y);
    body_def.type           = b2_kinematicBody;
    body                    = sim->world.CreateBody(&body_def);
    box.SetAsBox(width / 2.0, height / 2.0);
    fix.shape               = &box;
    fix.restitution         = 1.0;
    fix.filter.categoryBits = otype == Otype::WALL ? Category::ARENA : Category::BOX;
    fix.filter.maskBits     = 0xffff;
    body->CreateFixture(&fix);
    body->GetUserData().pointer     = (uintptr_t)this;
}
Static_box::~Static_box()
{
    sim->world.DestroyBody(body);
}
void Static_box::render(DebugDraw *dd, b2Color c)
{
    std::vector<b2Vec2> v;
    b2Vec2 pos = body->GetPosition();
    v.push_back(b2Vec2(pos.x - width / 2, pos.y - height / 2));
    v.push_back(b2Vec2(pos.x + width / 2, pos.y - height / 2));
    v.push_back(b2Vec2(pos.x + width / 2, pos.y + height / 2));
    v.push_back(b2Vec2(pos.x - width / 2, pos.y + height / 2));
    dd->DrawPolygon(v.data(), 4, c);

    if (otype == Otype::BOX)
        dd->DrawFont(pos - b2Vec2(0, 0.02), 0.25, string_format("%d", name).c_str(), 1);
}


// -------------------------------------------------------------------------------------------------
//
//  Main simulator class. Create the world and populate
//
// -------------------------------------------------------------------------------------------------
Sim::Sim(Sim_params &_sp, int _seed) : sp(_sp), seed(_seed), world(b2Vec2(0.0f, 0.0f))
{


    // Seed the random number generator
    // generator.seed(seed);
    setseed(seed);
    world.SetContactListener(&cl);
    time_since_step = 0;
    total_time      = 0;
    ticks_to_update = sp.ticks_per_update;
    system_bias     = Vector2f::Zero();

    // sp.num_boxes = sp.num_boxes_gui;
    // sp.num_robots = sp.num_robots_gui;
    system_box_state.resize(sp.num_boxes);
    observed_box_state.resize(sp.num_boxes);
    
    create_arena(sp.arena_size);

    // Create the BT factory, registering all the nodes used by the robots
    // to build their own controller trees    
    create_bt_factory();


    // Spawn robots and boxes into the arena, with a margin of 0.2m from edges
    float margin    = 0.2;
    // yingfei 
    // float as2       = sp.arena_size / 2 - margin;
    float as2       = sp.start_size / 2 - margin;

    // printf("Creating %d robots and %d boxes\n", sp.num_robots, sp.num_boxes);
    placed = create_random_robots(sp.num_robots, -as2, as2, -as2, as2);
    create_random_boxes(sp.num_boxes, -as2, as2, -as2, as2);

    if (sp.shape_file.size())
    {
        int x, y, n;
        // Load shape data, force RGBA arrangement
        sp.shape_data = stbi_load(sp.shape_file.c_str(), &x, &y, &n, 4);
        if (!sp.shape_data || x != 256 || y != 256)
        {
            printf("Failed to read shape file, %d %d\n", x, y);
            exit(1);   
        }
        printf("Loaded shape file %d %d %d\n", x, y, n);
    }

}



void Sim::create_arena(float size)
{
    const float width       = size;
    const float height      = size;
    const float wall_width  = 0.1f;
    
    
    static_boxes.push_back(new Static_box(this, b2Vec2((width + wall_width) / 2.0f, 0.0f), wall_width, height, Otype::WALL));
    static_boxes.push_back(new Static_box(this, b2Vec2(-(width + wall_width) / 2.0f, 0.0f), wall_width, height, Otype::WALL));
    static_boxes.push_back(new Static_box(this, b2Vec2(0.0f, (height + wall_width) / 2.0f), width, wall_width, Otype::WALL));
    static_boxes.push_back(new Static_box(this, b2Vec2(0.0f, -(height + wall_width) / 2.0f), width, wall_width, Otype::WALL));
}

void Sim::create_box(b2Vec2 p, int otype, int name)
{
    carrier_boxes.push_back(new Static_box(this, p, 0.35, 0.35, otype, name));
}


void Sim::create_robot(Vector2f s)
{
    robots.push_back(new Robot(this, s));
}

void Sim::create_robot(Vector2f s, b2Vec2 v, b2Color c, int name, float time)
{
    auto r          = new Robot(this, s);
    r->vel          = v.x;
    r->cmd_vel(0)   = v.x * cos(v.y);
    r->cmd_vel(1)   = v.x * sin(v.y);
    r->colour       = c;
    r->name         = name;
    r->fg.name      = name;
    if (time < 0.1) time = 0.1;
    r->traj_time    = time;

    // PERSON A: Configure robot 0 as attacker
    if (name == 0)
    {
        r->attack_type      = 1;
        r->attack_delta     = Vector2f(1.5f, 0.0f);
        r->attack_alpha     = 500.0f;
        r->attack_k_trigger = 3;
    }

    robots.push_back(r);cf
}


void Sim::create_bt_factory()
{
    // Here we register all the leaf nodes we define in bt.cpp/bt.hpp
    bt_factory.registerNodeType<SaySomething>("SaySomething");
    bt_factory.registerNodeType<ThinkWhatToSay>("ThinkWhatToSay");
}


bool Sim::check_obstruction(b2Vec2 p)
{
    return check_obstruction(p, -1, true);
}
bool Sim::check_obstruction(b2Vec2 p, int box, bool check_robots)
{
    bool overlap = false;
    if (check_robots)for(int j = 0; j < robots.size(); j++)
    {
        if (b2Vec2(robots[j]->body->GetPosition() - p).Length() < sp.min_sep)
        {
            overlap = true;
            break;
        }
    }
    for(int j = 0; j < carrier_boxes.size(); j++)
    {
        if (box != j && b2Vec2(carrier_boxes[j]->body->GetPosition() - p).Length() < sp.min_sep + 0.2)
        {
            overlap = true;
            break;
        }
    }
    return overlap;
}

bool Sim::create_random_robots(int num, float minx, float maxx, float miny, float maxy)
{
    // Brute force place num robots
    for(int i = 0; i < num; i++)
    {
        b2Color c = hsl2rgb(360.0 / num * i, 1.0, 0.3);
        //printf("Colour r:%f g:%f b:%f\n", c.r, c.g, c.b);
        bool found = false;
        int tries = 0;
        b2Vec2 p;
        while(not found)
        {
            p = b2Vec2(rand(minx, maxx), rand(miny, maxy));
            if (not check_obstruction(p))
                found = true;
            else
            {
                tries++;
                if (tries > 200)
                {
                    // printf("Failed to find initial placement solution!\n");
                    return false;
                }
            }
        }
        create_robot(Vector2f(p.x, p.y), b2Vec2(sp.robot_vel, rand(-M_PI, M_PI)), c, i, 2.0 + randn(1.0));
    }
    return true;
}


void Sim::create_random_boxes(int num, float minx, float maxx, float miny, float maxy)
{
    for(int i = 0; i < num; i++)
    {
        bool found  = false;
        int tries   = 0;
        b2Vec2 p;
        while(not found)
        {
            p = b2Vec2(rand(minx, maxx), rand(miny, maxy));
            if (not check_obstruction(p))
                found = true;
            else
            {
                tries++;
                if (tries > 100)
                {
                    printf("Failed to find initial placement solution!\n");
                    return;
                }
            }
        }
        create_box(p, Otype::BOX, i);
    }
}


float Sim::move_random_box(float dist, float minx, float maxx, float miny, float maxy)
{
    bool found  = false;
    int tries   = 0;
    b2Vec2 p;
    b2Vec2 s;
    int b;
    while (not found)
    {
        b = randi(0, carrier_boxes.size() - 1);
        s = carrier_boxes[b]->body->GetPosition();

        float a = rand(-M_PI, M_PI);
        p = s + dist * (b2Vec2(cos(a), sin(a)));

        if (p.x > minx && p.x < maxx && p.y > miny && p.y < maxy && not check_obstruction(p, b, true))
            found = true;
        else
        {
            tries++;
            if (tries > 100)
            {
                printf("Failed to find box move solution!\n");
                return 0;
                // exit(1);
            }
        }
    }
    carrier_boxes[b]->body->SetTransform(p, 0);
    return b2Vec2(s - p).Length();
}
float Sim::move_random_box2(float dist, float minx, float maxx, float miny, float maxy)
{
    // Zero all box velocities
    for(int i = 0; i < carrier_boxes.size(); i++)
        carrier_boxes[i]->body->SetLinearVelocity(b2Vec2(0, 0));

    bool found  = false;
    int tries   = 0;
    b2Vec2 p;
    b2Vec2 s;
    float a;
    int b;
    while (not found)
    {
        b = randi2(0, carrier_boxes.size() - 1);
        s = carrier_boxes[b]->body->GetPosition();

        a = rand2(-M_PI, M_PI);
        p = s + dist * (b2Vec2(cos(a), sin(a)));

        if (p.x > minx && p.x < maxx && p.y > miny && p.y < maxy && not check_obstruction(p, b, false))
        // if (p.x > minx && p.x < maxx && p.y > miny && p.y < maxy)
            found = true;
        else
        {
            tries++;
            if (tries > 100)
            {
                printf("Failed to find box move solution!\n");
                return 0;
                // exit(1);
            }
        }
    }
    float v = sp.box_vel * carrier_boxes.size();
    b2Vec2 vel = v * b2Vec2(cos(a), sin(a));
    carrier_boxes[b]->body->SetLinearVelocity(vel);
    return b2Vec2(s - p).Length();
}


void Sim::step()
{
    // ---------------------------------------------
    // Do one physics step of the simulation
    // ---------------------------------------------

    world.Step(sp.dt, 8, 3);
    
    float error     = 0;
    float gbp_error = 0;
    sensed          = 0;
    system_bias     = Vector2f::Zero();


    // Get the swarm mean bias, i.e.
    for(auto &r : robots)
    {
        system_bias += r->bias;
    }
    system_bias /= robots.size();

    float box_offset    = 5;
    float box_gain      = 5;
    float box_gbp_est   = 0;
    float box_est       = 0;
    int box_num         = 0;
    int box_o_num       = 0;




    for(auto &r : robots)
    {
        r->step();
        error       += r->error;
        gbp_error   += r->gbp_error;

        //printf("%d ", r->seen_robots.size(),r->senses.converged);
    }
    //printf("\n");
    error       /= robots.size();
    gbp_error   /= robots.size();


    // Update the blackboard separately
    ticks_to_update -= 1.0;
    if (ticks_to_update <= 0)
    {
        // printf("update\n");
        ticks_to_update += sp.ticks_per_update;


        
        // for(auto &r : robots)
        // {
        //     auto &ch = r->rel_observed_box_state[r->chasing_name];
        //     printf("%5d %5d %5d | ", r->name, ch.name, ch.t);
        //     for(auto &s : r->rel_observed_box_state)
        //     {
        //         printf("%5d %5d |", s.name, s.t);
        //     }
        //     printf("\n");
        // }

        for(auto &r : robots)
        {
            r->update_blackboard();
        }
        for(auto &r : robots)
        {
            r->iterate_graph();
        }
        // Find least reccently observed box
        int t = std::numeric_limits<int>::max();
        sortbox.clear();
        for(auto &b : global_box_obs)
        {
            if (b.second < t)
            {
                t           = b.second;
                oldest_box  = b.first;
            }
            sortbox.push_back(std::pair<int,int>(b.first, b.second));
            std::sort(sortbox.begin(), sortbox.end(), 
                [](std::pair<int,int> x, std::pair<int,int> y){return x.second < y.second;} );
        }
        // printf("oldest: %5d %5d\n",oldest_box, t);

        std::vector<int> popular;
        popular.resize(sp.num_boxes);
        for(auto &r : robots)
        {
            if (r->senses.converged && r->all_boxes_observed && r->chasing_name >= 0)
            {
                popular[r->chasing_name]++;
            }
        }
        // Colour robots with one of the top n carriers
        // Colour is related to the carrier number
        // std::vector<std::pair<int,int> > pr;
        popbox.clear();
        for(int i = 0; i < sp.num_boxes; i++)
        {
            popbox.push_back(std::pair<int,int>(popular[i], i));
        }
        std::sort(popbox.begin(), popbox.end(), 
            [](std::pair<int,int> x, std::pair<int,int> y){return x.first > y.first;} );


        // Find most popular chased boxes in robots, colour those robots
        // with diferent colours
        // for(auto &i : popbox)
        // {
        //     printf("%5d %5d\n", i.first, i.second);
        // }
    }





    timer::tick();
    time_since_step += sp.dt;

    if (time_since_step >= sp.new_node_time)
    {
        time_since_step = 0;
        for(auto &r : robots)
        {
            r->time_for_new_node = true;
            r->fg.timestep++;
            // r->fgcs.timestep++;
        }
    }

    // -----------------------------------------------------------
    // We track various error metrics:
    //
    //  rpos_error      mean distance between
    // Get the theoretically best possible observation
    float ideal_error       = 0.0;
    float algo2_error       = 0.0;
    float rpos_error        = 0.0;

    Vector2f gt_centroid    = Vector2f::Zero();
    Vector2f est_centroid   = Vector2f::Zero();
    for(auto &r : robots)
    {
        for(int i = 0; i < sp.num_boxes; i++)
        {
            Vector2f p = to_eigen(carrier_boxes[i]->body->GetPosition());
            ideal_error += (p - r->oracle_observed_box_state[i].p).norm();
            algo2_error += (p - (r->rel_observed_box_state[i].p - r->senses.position.mu 
                            + to_eigen(r->body->GetPosition()))).norm();
        }
        gt_centroid     += to_eigen(r->body->GetPosition());
        est_centroid    += r->senses.position.mu;
    }
    ideal_error /= sp.num_boxes * sp.num_robots;
    algo2_error /= sp.num_boxes * sp.num_robots;

    gt_centroid /= sp.num_robots;
    est_centroid /= sp.num_robots;
    
    for(auto &r : robots)
    {
        Vector2f p0 = to_eigen(r->body->GetPosition()) - gt_centroid;
        Vector2f p1 = r->senses.position.mu - est_centroid;
        rpos_error += (p0 - p1).norm();
    }
    rpos_error = rpos_error / sp.num_robots;


    //printf("Ideal error %6.3f\n", ideal_error);
    system_ideal_error.push_back(ideal_error);
    system_algo2_error.push_back(algo2_error);


    system_box_est.push_back((box_est) * sp.box_gain + sp.box_offset);
    system_box_gbp_est.push_back((box_gbp_est - system_bias(0)) * sp.box_gain + sp.box_offset);

    system_bias_mag.push_back(system_bias.norm());
    // system_error.push_back(error);
    system_gbp_error.push_back(gbp_error);
    system_rpos_error.push_back(rpos_error);



    total_time += sp.dt;

    hs_bandwidth = 0;
    bb_bandwidth = 0;
    flops_create = 0;
    flops_update = 0;
    for(auto &r : robots)
    {
        hs_bandwidth += r->total_out_bytes + r->total_ret_bytes;
        bb_bandwidth += r->total_bb_bytes;
        flops_create += r->fg.flops_create;
        flops_update += r->fg.flops_update;
        //printf("%6d ", r->total_out_bytes + r->total_ret_bytes);
    }
    hs_bandwidth /= total_time * robots.size();
    bb_bandwidth /= total_time * robots.size();
    flops_create /= total_time * robots.size();
    flops_update /= total_time * robots.size();

    // printf("%12.2f %12.2f %12.2f %12.2f\n", hs_bandwidth, bb_bandwidth, flops, total_time);


    if (sp.rmove_en)
    {
        if (sp.move_counter <= 0)
        {
            float move_area = sp.arena_size / 2 - 0.2;
            float dist = move_random_box2(sp.box_dist, -move_area, move_area, -move_area, move_area);
            // mean v = dist/(n * t)
            // t = dist / (v * n)
            float t = dist / (sp.box_vel * carrier_boxes.size());
            int ticks = t / sp.dt;

            sp.move_counter = ticks;
            //printf("Dist %6.3f vel %6.3f boxes %2d time %6.3f ticks %4d\n", dist, sp.box_vel, sp.num_boxes, t, ticks);
        }
        else
        {
            sp.move_counter--;
        }
    }

}

bool Sim::in_shape(Vector2f p)
{
    int s = sp.shape_idx;
    float r = p.norm();

    float x = p(0);
    float y = p(1);
    float t1 = 0.5;
    float t2 = 0.75;
    switch (s)
    {
        case (0) : 
        {
            float d = 1.5;
            return (fabs(x - d) < t1 || fabs(x + d) < t1 || fabs(y - d) < t1 || fabs(y + d) < t1)
                    && (fabs(x) < (d + t1) && fabs(y) < (d + t1));
        }
        case (1) : // circle
        {
            return r > 1.7 && r < 2.5;
        }
        case (2) : // cross
        {
            return ((fabs(x - y) < t2 || fabs(x + y) < t2) && r < 2.5);
        }
        case (3) : // wavy
        {
            float a = 2.0;
            float b = 0.5;
            float c = 1.5;
            return 
                (fabs(b * cos(a * x) - y + c) < t1 || fabs(b * cos(a * x) - y - c) < t1) && fabs(x) < 2;
        }
        case (4) : 
        {
            float w = 0.6;
            float d = 3.0;
            return (fabs(y) < w && fabs(x) < d);
        }
        case (5) :
        {
            float w = 0.6;
            float d = 3.0;
            return (fabs(x) < w && fabs(y) < d);
        }
        default:    return 0;

    }
}


void Sim::render(int w, int h, int rplot_flags[], bool show_fg)
{
    for(auto r : robots)
        r->render(dd, system_bias);
        // r->render(dd, rplot_flags, show_fg, Vector2f::Zero());
    for(auto b : static_boxes)
        b->render(dd, b2Color(0.7, 0.7, 0.7));
    if (sp.gui.box)for(auto b : carrier_boxes)
    {
        b->render(dd, b2Color(0.2, 0.2, 0.6));
        if (sp.gui.obox && (b->name == oldest_box))
        {
            // dd->DrawShadedCircle(b->body->GetPosition(), 0.3, b2Color(0.9,0.1,0.1));
            dd->DrawShadedCircle(b->body->GetPosition(), 0.3, hsl2rgb(360 * oldest_box / sp.num_boxes, 1.0, 0.8));
        }
    }

    if (sp.gui.ebpos)for(int i = 0; i < carrier_boxes.size(); i++)
    {
        int num = 0;
        Vector2f pos = Vector2f::Zero();

        
        // Ideal
        pos = Vector2f::Zero();
        num = 0;
        for(auto r : robots)
        {
            if (r->rel_observed_box_state[i].t > 0)
            {
                pos += r->rel_observed_box_state[i].p;
                num++;
            }
        }
        if (num)
        {
            pos = pos / sp.num_robots - system_bias;
            dd->DrawShadedCircle(b2Vec2(pos(0), pos(1)), 0.2, b2Color(0.2,0.2,0.2));
        }
    }

    if (sp.gui.origin)    
    {
        float l = 0.3;
        for(auto r : robots)
        {
            Vector2f p =  to_eigen(r->body->GetPosition()) - r->senses.position.mu;
            dd->DrawThickSegment(b2Vec2(p(0), p(1)), b2Vec2(p(0) + l , p(1)), b2Color(0.8, 0, 0), 0.02);
            dd->DrawThickSegment(b2Vec2(p(0), p(1)), b2Vec2(p(0) , p(1) + l), b2Color(0, 0.8, 0), 0.02);
            if (sp.gui.oline)
                dd->DrawSegment(b2Vec2(p(0), p(1)), r->body->GetPosition(), b2Color(0.6, 0.6, 0.6));
        }

    }


    if (0 && sp.gui.shape)
    {
        for(auto r : robots)
        {
            float as2 = sp.arena_size / 2;
            float inc = 0.05;
            float i2 = inc / 2;
            Vector2f p = r->senses.position.mu - to_eigen(r->body->GetPosition());
            for (float x = -as2; x < as2; x+= inc)
            {
                for (float y = -as2; y < as2; y+= inc)
                {
                    float ax = x - p(0);
                    float ay = y - p(1);
                    if (in_shape(Vector2f(x, y)))
                    {
                        // dd->DrawShadedCircle(b2Vec2(ax, ay), 0.05, b2Color(0.2,0.2,0.2));
                        // b2Vec2 v[4] = {{ax - i2, ay - i2}, {ax + i2, ay - i2}, {ax + i2, ay + i2}, {ax - i2, ay - i2}};
                        dd->DrawSquare(b2Vec2(ax, ay), inc, b2Color(0.2, 0.2, 0.2, 0.2 / sp.num_robots));
                    }
                }
            }
        }
    }


        // yingfei
    if (1 & sp.gui.shape)
    {
        float as2 = sp.arena_size / 2;
        float inc = 0.05;
        for (float x = -as2; x < as2; x+= inc)
        {
            for (float y = -as2; y < as2; y+= inc)
            {
                if (in_shape(Vector2f(x, y)))
                {
                    // 1. Draw the Attacked / Estimated Shape (Dark Grey)
                    // This moves with the system_bias as the swarm gets hijacked
                    Vector2f p = system_bias;
                    float ax = x - p(0);
                    float ay = y - p(1);
                    dd->DrawSquare(b2Vec2(ax, ay), inc, b2Color(0.2, 0.2, 0.2, 0.2));

                    // 2. Draw the Original Ground-Truth Shape (Green Ghost)
                    // This is permanently locked to the true Box2D origin (0,0)
                    dd->DrawSquare(b2Vec2(x, y), inc, b2Color(0.0, 0.8, 0.0, 0.15));
                }
            }
        }
    }
    // yingfei

/* 
    if (1 & sp.gui.shape)
    {

        float as2 = sp.arena_size / 2;
        float inc = 0.05;
        float i2 = inc / 2;
        int r = 0;
        for (float x = -as2; x < as2; x+= inc)
        {
            for (float y = -as2; y < as2; y+= inc)
            {
                // Vector2f p = robots[r]->senses.position.mu - to_eigen(robots[r]->body->GetPosition());
                Vector2f p = system_bias;
                r = (r + 1) % sp.num_robots;
                float ax = x - p(0);
                float ay = y - p(1);
                if (in_shape(Vector2f(x, y)))
                {
                    // dd->DrawShadedCircle(b2Vec2(ax, ay), 0.05, b2Color(0.2,0.2,0.2));
                    // b2Vec2 v[4] = {{ax - i2, ay - i2}, {ax + i2, ay - i2}, {ax + i2, ay + i2}, {ax - i2, ay - i2}};
                    dd->DrawSquare(b2Vec2(ax, ay), inc, b2Color(0.2, 0.2, 0.2, 0.2));
                }
            }
        }
    } */
    // if (sp.gui.fshape)
    // {
    //     float as2 = sp.arena_size / 2;
    //     float inc = 0.05;
    //     float i2 = inc / 2;
    //     int r = 0;
    //     for (float x = -as2; x < as2; x+= inc)
    //     {
    //         for (float y = -as2; y < as2; y+= inc)
    //         {
    //             // Vector2f p = robots[r]->senses.position.mu - to_eigen(robots[r]->body->GetPosition());
    //             Vector2f p = system_bias;
    //             r = (r + 1) % sp.num_robots;
    //             float ax = x - p(0);
    //             float ay = y - p(1);
    //             int px = x * 50 + 128;
    //             int py = y * 50 + 128;
    //             if (px >= 0 && px < 256 && py >= 0 && py < 256)
    //             {
    //                 uint8_t *dp = sp.shape_data + py * 1024 + px * 4;
    //                 uint8_t r = dp[0];
    //                 uint8_t g = dp[1];
    //                 uint8_t b = dp[2];
    //                 dd->DrawSquare(b2Vec2(ax, ay), inc, 
    //                     b2Color((float)r / 256 , (float)g / 256, (float)b / 256, 0.5));
    //             }
    //         }
    //     }

    // }

    //
    int av_len = 600;
    se = sp.max_error;
    re = sp.max_error;
    if (system_algo2_error.size())
    {
        int num = system_algo2_error.size() > av_len ? av_len : system_algo2_error.size();
        se = std::accumulate(system_algo2_error.end() - num, system_algo2_error.end(), 0.0) / (float)num;
        re = std::accumulate(system_rpos_error.end() - num, system_rpos_error.end(), 0.0) / (float)num;

    }
    // float scale     = cam->m_zoom * 1.5;
    // float as2       = sp.arena_size / 2;
    // dd->DrawFont(b2Vec2(-as2, as2 - scale), scale, string_format("t    %6.1f", total_time).c_str());
    // dd->DrawFont(b2Vec2(-as2, as2 - 2 * scale), scale, string_format("s_err %.3f", se).c_str());
    // dd->DrawFont(b2Vec2(-as2, as2 - 3 * scale), scale, string_format("r_err %.3f", re).c_str());

    
    world.DebugDraw();
}

void Sim::reset()
{
    // -------------------------------------------------------
    // PERSON B: Export detector logs to CSV on reset/exit
    // -------------------------------------------------------
    if (sp.export_csv && robots.size() > 0)
    {
        FILE *f = fopen(sp.csv_path.c_str(), "w");
        if (f)
        {
            // Header
            fprintf(f, "time,robot,s_lambda,s_dkl,s_temp,s_inst,flagged,"
                       "r_error,attack_active\n");
            for (auto &r : robots)
            {
                for (auto &rec : r->detector_log)
                {
                    // Find matching r_error index
                    int idx = (int)(rec.time / sp.dt);
                    float rerr = (idx < (int)system_rpos_error.size())
                                 ? system_rpos_error[idx] : -1.0f;
                    fprintf(f, "%.3f,%d,%.4f,%.4f,%.6f,%.4f,%d,%.4f,%d\n",
                        rec.time,
                        r->name,
                        rec.s_lambda,
                        rec.s_dkl,
                        rec.s_temp,
                        rec.s_inst,
                        (int)rec.flagged,
                        rerr,
                        (int)r->attack_active
                    );
                }
            }
            fclose(f);
            printf("[PERSON B] Detector log written to %s\n", sp.csv_path.c_str());
        }
    }
    // -------------------------------------------------------
}