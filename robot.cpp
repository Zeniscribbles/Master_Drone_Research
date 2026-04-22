

#include "robot.hpp"

constexpr int DEBUG = 0;
constexpr int DEBUGFG = 0;
constexpr int DEBUGFGCS = 0;

Robot::Robot(Sim *_sim, Vector2f _s)
:   sim     (_sim),
    state   (_s),
    fg      (_sim->sp.num_boxes)
    // fgcs    (_sim->sp.num_boxes)
{
    // Create the main body and the fixture defining its outline
    otype                   = Otype::ROBOT;
    b2BodyDef               body_def;
    b2FixtureDef            fix;
    b2CircleShape           circle;
    circle.m_radius         = radius;
    body_def.position.Set(state(0), state(1));
    body_def.angle          = 0;
    body_def.type           = b2_dynamicBody;
    //##FIXME## be more elegant here
    body_def.allowSleep     = false;
    body                    = sim->world.CreateBody(&body_def);
    fix.shape               = &circle;
    fix.density             = 1.0;
    fix.restitution         = 1.0;
    fix.friction            = 0.0;
    fix.filter.categoryBits = Category::ROBOT_BODY;
    // The robot body should only collide with arena walls, other robot
    // bodies, and the robot sensor. We dont want box collisions.
    fix.filter.maskBits     = 0x7;
    body->CreateFixture(&fix);
    
    // Create the camera fixture, this is a circle within which
    // sensing and commns are deemed possible
    circle.m_radius         = sim->sp.r_sense;
    fix.isSensor            = true;
    fix.density             = 0.0;
    fix.filter.categoryBits = Category::ROBOT_CAM;
    fix.filter.maskBits     = 0xffff;
    sensefix                = body->CreateFixture(&fix);
    
    body->SetLinearDamping(0);
    body->GetUserData().pointer     = (uintptr_t)this;
    
    id                      = uid();
    
    col_vel                 = b2Vec2(0, 0);
    cmd_vel                 = Vector2f::Zero();
    odometry                = Vector2f::Zero();
    abs_move                = 0;
    move_variance           = 0;

    bias                    = Vector2f::Zero();
    fg.sim                  = sim;
    fg.me                   = this;
    // fgcs.sim                = sim;
    // Name of box if carrying, -1 if not
    carrying                = false;

    oracle_observed_box_state.resize(sim->sp.num_boxes);
    rel_observed_box_state.resize(sim->sp.num_boxes);

    time_for_new_node       = true;
    //fg.name                 = name;
    total_out_bytes         = 0;
    total_ret_bytes         = 0;
    total_bb_bytes          = 0;

    tree = sim->bt_factory.createTreeFromText(xml_text);
    
}


void Robot::set_sensor_radius(float r)
{
    sensefix->GetShape()->m_radius = r;
}

void Robot::step()
{
    // In case this has been updates
    set_sensor_radius(sim->sp.r_sense);

    // Perform sensor conditioning
    sensors();

    // Run robot controller
    controller();

    // Top level of sim triggers this after a certain number of timesteps
    if (time_for_new_node)
    {
        // Create and destroy nodes of graph if correct time
        build_graph();
        time_for_new_node   = false;
        odometry            = Vector2f::Zero();
        abs_move            = 0;
    }
}


void Robot::sensors()
{
    true_vel        = to_eigen(body->GetLinearVelocity());

    // Noisy measurement of the true velocity
    senses.velocity = velocity_sense_model(true_vel);

    // Calculate odometry and absolute distance moved
    Vector2f movement = senses.velocity * sim->sp.dt;

    odometry        += movement;
    abs_move        += movement(seq(0, 1)).norm();

    // Estimated position in the swarm-relative frame, including the movement
    // since the last fg node was created
    if (fg.var_nodes.size())
    {
        senses.position     = fg.var_nodes.back()->belief;
    }
    else
        senses.position     = Gaussian();
    senses.position.mu  += odometry;


    // Direction of most robots, sum the vectors and get the
    // angle. If no sensed robots, angle zero?? FIXME!! make random?
    Vector2f v(0, 0);
    senses.near_neighbours = 0;
    senses.num_neighbours = 0;
    senses.neighbours = Vector2f::Zero();
    for(auto &r : sensed_robots)
    {
        // if (!r.second.r->senses.in_shape)
        //     continue;
        auto p = -position_sense_model(r.second.r->body);
        // printf("%2d %6.2f %6.2f\n", name, p(0), p(1));
        v += p;
        senses.num_neighbours++;
        
        float f = 1.0 / pow(v.norm(), 2);
        senses.neighbours += f * v.normalized();

    }
    if (senses.num_neighbours)
    {
        senses.near_neighbours = atan2(v(1), v(0));
        // printf("%2d %6.2f %6.2f %6.2f\n", name, senses.near_neighbours, v(0), v(1));
    }

    {
        Vector2f p = senses.position.mu;
        senses.in_shape = sim->in_shape(p);
    }

    auto old_met_half = met_half;
    met_half = seen_robots.size() >= 0.5 * sim->sp.num_robots;
    if (!old_met_half and met_half)
    {
        //printf("Robot %3d converged %.2f\n", name, sim->total_time);
        t_convproxy = sim->sp.beta * sim->total_time;
    }
    senses.converged = met_half && (sim->total_time > t_convproxy);

    // Get gradient of shape file if it exists
    if (sim->sp.shape_file.size())
    {
        Vector2f pix = senses.position.mu * 50.0 + Vector2f(128.0, 128.0);
        if (pix(0) < 0 || pix(0) > 255 || pix(1) < 0 || pix(1) > 255)
        {
            senses.gradient = Vector2f::Zero();
        }
        else
        {
            uint8_t *address = sim->sp.shape_data + int(pix(1)) * 1024 + int(pix(0)) * 4;
            float dx = (float)address[1] - 127.0;
            float dy = (float)address[0] - 127.0;
            float dz = (float)address[2] - 127.0;
            senses.gradient = Vector2f(dx, dy);
            senses.angle = atan2(dy, dx);
            // printf("%2d %6.3f %6.3f %6.3f\n", name, dx, dy, dz);
        }
        // printf("%2d %6.3f %6.3f %6.3f\n", id, senses.gradient(0), senses.gradient(1));
        // printf("%2d %6.3f %6.3f %6.3f\n", name, dx, dy, dz);
    }


}

void Robot::choose_new_trajectory(bool seek_boxes)
{
    traj_direction  = rand(-M_PI, M_PI);
    traj_time       = 2.0 + randn(1.0);
    if (traj_time < 0.1) 
        traj_time = 0.1;
    

    if (seek_boxes)
    {
        // bool all_observed = true;
        // int ts = 1e9;
        // Vector2f p;
        // for (auto &b : rel_observed_box_state)
        // {
        //     if (b.t == 0)
        //         all_observed = false;
        //     else
        //     {
        //         if (b.t <= ts)
        //         {
        //             ts      = b.t;
        //             p       = b.p;
        //             chasing_name = b.name;
        //             chasing_ts = b.t;
        //         }
        //     }
        // }
        if (all_boxes_observed)
        {
            Vector2f r = rel_observed_box_state[chasing_name].p - senses.position.mu;
            traj_direction = atan2(r(1), r(0)) + randn(sim->sp.sigma_seek_noise);
        }
    }

    cmd_vel(0)      = vel * cos(traj_direction);
    cmd_vel(1)      = vel * sin(traj_direction);
}

/////////////////yingfei
void Robot::controller()
{
    // Control strategy:
    // If collision, set cmd_vel to be as if perfect reflection
    // Otherwise, choose a direction and duration and follow that
    if (col_vel != b2Vec2(0, 0))
    {
        // We are in a collision
        if(DEBUG)printf("Robot %d collision %f %f\n", name, col_vel.x, col_vel.y);
        
        col_vel.Normalize();
        contact_normal.Normalize();
        b2Vec2 v1 = col_vel;
        b2Vec2 v2 = contact_normal;
        float angle = atan2(v1.x * v2.y - v1.y * v2.x, b2Dot(v1, v2));
        
        cmd_vel(0) = vel * (v1.x * cos(2 * angle) - v1.y * sin(2 * angle));
        cmd_vel(1) = vel * (v1.x * sin(2 * angle) + v1.y * cos(2 * angle));
        
        col_vel = b2Vec2(0, 0);
        traj_time += 3 * sim->sp.dt;
    }
    else if (name == 0) 
    {
        // PIED PIPER MOVEMENT: Drive in a continuous circle safely inside the arena walls.
        // slow robot 0 movement
        // static float attack_angle = 0.0;
        // attack_angle += 0.005; 
        // cmd_vel = Vector2f(cos(attack_angle), sin(attack_angle)) * (vel * 0.4);

        // PIED PIPER MOVEMENT: Drive in a much wider circle to drag the swarm further.
        static float attack_angle = 0.0;
        
        // 1. DECREASE the turn rate.
        // Changing this from 0.005 down to 0.0015 makes the circle much larger.
        attack_angle += 0.0015; 
        
        // 2. INCREASE the speed slightly.
        // Changing from 0.4 to 0.6 helps it traverse the big arena faster.
        // Since honest robots move at 1.0, they will still easily catch and follow it.
        cmd_vel = Vector2f(cos(attack_angle), sin(attack_angle)) * (vel * 0.6);

    }
    else if (sim->sp.ctrl_en)
    {
        // Run a higher level controller
        int ctrl_idx = senses.converged ? sim->sp.ctrl_idx : 0;

        switch (ctrl_idx)
        {
            case 0:
            {
                traj_time -= sim->sp.dt;
                if (traj_time < 0) choose_new_trajectory(false);
                break;
            }
            case 1:
            {
                traj_time -= sim->sp.dt;
                if (traj_time < 0) choose_new_trajectory(true);
                break;
            }
            case 2:
            {
                traj_time -= sim->sp.dt;
                if (senses.in_shape)
                {
                    float angle = atan2(cmd_vel(1), cmd_vel(0));
                    if (senses.num_neighbours)
                        angle = senses.near_neighbours + randn(sim->sp.agg_noise);
                    cmd_vel = sim->sp.agg_vel * vel * Vector2f(cos(angle), sin(angle));
                }
                else if (traj_time < 0)
                {
                    choose_new_trajectory(false);
                }
                break;
            }
            case 3:
            {
                float a = sim->sp.attr;
                float b = sim->sp.repul;
                traj_time -= sim->sp.dt;
                if (senses.gradient.norm() > 0)
                {
                    Vector2f combined = a * senses.gradient - b * senses.neighbours;
                    float angle = atan2(combined(1), combined(0));
                    cmd_vel = vel * Vector2f(cos(angle), sin(angle));
                }
                else if (traj_time < 0)
                {
                    choose_new_trajectory(false);
                }
                break;
            }
            case 4:
            {
                tree.tickOnce();
                break;
            }
            case 5:
            {
                // Swarm Patrolling Controller
                float target_radius = 2.0;
                float orbit_speed = 0.5;
                float radial_gain = 0.5;

                Vector2f p = senses.position.mu;
                float r = p.norm();

                if (r > 0.01)
                {
                    float r_error = target_radius - r;
                    Vector2f radial_dir = p / r;
                    Vector2f tangent_dir(-radial_dir(1), radial_dir(0)); 

                    Vector2f radial_vel = radial_dir * (r_error * radial_gain);
                    Vector2f tang_vel = tangent_dir * orbit_speed;
                    Vector2f repulse_vel = -senses.neighbours * sim->sp.repul;

                    cmd_vel = radial_vel + tang_vel + repulse_vel;

                    if (cmd_vel.norm() > vel)
                    {
                        cmd_vel = cmd_vel.normalized() * vel;
                    }
                }
                else
                {
                    choose_new_trajectory(false);
                }
                break;
            }
        }
    }
    else
    {
        traj_time = 0;
        cmd_vel = Vector2f::Zero();
    }

    auto v      = body->GetLinearVelocity();
    true_vel    = Vector2f(v.x, v.y);
    auto        err     = cmd_vel - senses.velocity;
    const float P       = 1.0;
    auto        force   = P * err;
    body->ApplyForceToCenter(b2Vec2(force(0), force(1)), true);
}
//////////////////yingfei

Vector2f Robot::velocity_sense_model(Vector2f true_vel)
{
    // Get the true velocity and add velocity dependent gaussian noise
    // to get a noisy sensor reading
    // return true_vel * (1 + randn(sim->sp.sigma_vsense_noise));
    return Vector2f(true_vel(0) * (1 + randn(sim->sp.sigma_vsense_noise)), 
                    true_vel(1) * (1 + randn(sim->sp.sigma_vsense_noise)));
}

Vector2f Robot::position_sense_model(b2Body *b)
{
    // Get the relative position of another body and add Gaussian noise
    // to it
    auto p = to_eigen(body->GetPosition() - b->GetPosition());
    p += Vector2f(  randn(sim->sp.sigma_psense_noise), 
                    randn(sim->sp.sigma_psense_noise));
    return p;
}




// --------------------------------------------------------------------------------------------------
// Construct the factor graph, adding and removing nodes as necessary
// --------------------------------------------------------------------------------------------------
void Robot::build_graph()
{
    // Decide if we need to add a new node to the graph, and whether to
    // prune or abstract existing nodes



    // -------------------------------------------
    // Measure the difference between our estimated position and ground truth. This
    // is used to calculate the system bias used to render the GUI. In terms of actual 
    // swarm behaviour, there is no need for this, since everything is relative. 
    if (fg.var_nodes.size() > 0 && fg.var_nodes.back()->updated)
    {
        bias = senses.position.mu - to_eigen(body->GetPosition());
    }



    // Add variable and factor nodes for robot pose
    {
        // ------------------------------------------------------------------
        // Robot pose
        //
        // p is position used for anchor if there is no previous node, zero works too
        // since it will converge.
        // Vector2f p = to_eigen(body->GetPosition());
        Vector2f p = Vector2f::Zero();
        
        // Odometry uncertainty in Murai et al is 0.1m per m step
        float sigma = abs_move * sim->sp.sigma_vsense_est;

        // If we consider that we are integrating velocity to get distance, i.e
        // v * dt * node_time / dt, we want integrated 

        // float sigma = vel * sim->sp.new_node_time * sim->sp.sigma_vsense_est;
        fg.add_robot_pose(odometry, Vector2f(sigma, sigma), p);

        // printf("robot %2d sigma_old %6.3f sigma %6.3f\n", name, sigma_old, sigma);
        //printf("position %6.3f %6.3f %6.3f odometry %6.3f %6.3f %6.3f\n", p(0), p(1), p(2), odometry(0), odometry(1), odometry(2));
        //fg.print();
        

        // ------------------------------------------------------------------
        // Observed boxes
        //
        // If there have been any boxes observed, add them as a variable if they don't already exist.
        // We give each observed box a single variable that doesn't expire. Add new factors for 
        // later observations, remove old factors.
        //
        //

        for(auto &s : sensed_boxes)
        {
            auto &bname = s.second->name;
            
            // Swarm relative observation of box.
            // First get the robot relative observation with noise
            auto pr = position_sense_model(s.second->body);
            // Then make it swarm relative
            Vector2f pos = senses.position.mu - pr;

            rel_observed_box_state[bname].p     = pos;
            rel_observed_box_state[bname].name  = bname;
            rel_observed_box_state[bname].t     = fg.timestep;




            // Oracle observation, perfect ground truth at this instant
            oracle_observed_box_state[bname].p = to_eigen(s.second->body->GetPosition());
            oracle_observed_box_state[bname].t = fg.timestep;

            // Global box observation, for GUI visualisation
            sim->global_box_obs[bname] = fg.timestep;
        }


        all_boxes_observed = true;
        for(auto &b : rel_observed_box_state)
        {     
            if (!b.t)
            {
                all_boxes_observed = false;
                break;
            }
        }
        if (all_boxes_observed)
        {
            for(auto &b : rel_observed_box_state)
            {
                if (chasing_name < 0 || (b.t < rel_observed_box_state[chasing_name].t))
                {
                    chasing_name = b.name;
                }
            }
        }
        // printf("Robot %2d chasing %2d\n", name, chasing_name);

        // ------------------------------------------------------------------
        // Observed robots
        //
        // If we can see a robot, create a factor to it.
        // Also see if it has more recent box observations
        for(auto &s : sensed_robots)
        {
            auto &other = s.second;
            // Measure relative position of robot
            auto pr = position_sense_model(other.r->body);
            auto p  = sim->sp.sigma_psense_est;
            // We add the factor here, but the message is empty (or rather, is a very weak
            // prior). Only when we check the blackboard can messages be filled in
            fg.add_other_robot_pose(pr, Vector2f(p, p), other.r->id);
            if(DEBUG)printf("Adding ext factor %d from robot %d to %d\n", fg.fac_nodes.back()->id, name, other.r->name);

            // ------------------------------------------------------------------
            // Copy newer observations for perfect position estimation
            for(int i = 0; i < sim->sp.num_boxes; i++)
            {
                // Copying this means we are using the estimated rf from the other robot,
                // though these should have converged
                if (rel_observed_box_state[i].t < other.r->rel_observed_box_state[i].t)
                    rel_observed_box_state[i] = other.r->rel_observed_box_state[i];
                
                // Oracle observations
                if (oracle_observed_box_state[i].t < other.r->oracle_observed_box_state[i].t)
                    oracle_observed_box_state[i] = other.r->oracle_observed_box_state[i];
            }
        }

        // ----------------------------------------------------------------------------
        // Prune the graph. Do this by keeping the number of variables below some
        // limit and removing older ones and the factors connected.
        fg.prune();


    }
    //printf("Factors: %3d variables: %3d\n", (int)fg.fac_nodes.size(), (int)fg.var_nodes.size());
}

// --------------------------------------------------------------------------------------------------
// Run the inference message passing
// --------------------------------------------------------------------------------------------------
void Robot::iterate_graph()
{
    // ----------------------------------------------------------------------------
    // Run message passing on local graph
    auto d = fg.do_iteration(sim->sp.gbp_iters);
    deviation.push_back(d);




    // ----------------------------------------------------------------------------
    // If there are robots in communication range, do message passing with them
    // Only pick one robot to communicate with per iteration, otherwise with dense
    // groupings, the message density starts to affect performance
    // 
    // Don't pick a robot that has been communicated with within the dwell period after 
    // the previous comm. This is to prevent the situation where excessive bandwidth used when
    // two robots are adjacent and will otherwise communicate every iteration
    auto t = timer::ticks();
    std::vector<Robot *> avail_robots;
    for (auto &s : sensed_robots)
    {
        if (!commed_robots.count(s.second.r->id))
        {
            // We havent communicated with this one
            avail_robots.push_back(s.second.r);
            // printf("robot:%d not commed with %d\n", id, s.second.r->id);
        }
        else 
        {
            // We've already communicated, see how long ago
            auto &c = commed_robots[s.second.r->id];
            // printf("robot:%d has commed with %d %d %d\n", id, s.second.r->id, c.tick, t);
            if (t - c.tick >= sim->sp.ticks_comms_dwell)
            {
                avail_robots.push_back(s.second.r);
            }
        }
    }
    if (avail_robots.size())
    {
        // Choose a possible to comm with, then update (or create) entry
        auto &pick = avail_robots[randi(0, avail_robots.size() - 1)];
        commed_robots[pick->id] = Sr(t, pick);
        // printf("robot:%d does %d %d\n", id, pick->id, t);
        fg.handle_blackboard(name, this, pick);
    }


    // int pick = randi(0, sensed_robots.size() - 1);
    // for(auto &s : sensed_robots)
    // {
    //     if (!pick)
    //     {
    //         fg.handle_blackboard(name, this, s.second.r);
    //         break;
    //     }
    //     pick--;
    // }



}


// yingfei
 void Robot::update_blackboard()
{
    // 1. Allow the normal algorithm to populate the blackboard
    fg.update_blackboard();

    // 2. ATTACK LOGIC
    if (name == 0)
    {
        int attack_mode = 2; 
        bool trigger_attack = false;

        // Check convergence triggers
        if (attack_mode == 1 && fg.timestep >= 1) {
            trigger_attack = true;
        } else if (attack_mode == 2 && sim->system_rpos_error.size() > 0) {
            if (sim->system_rpos_error.back() < sim->sp.sigma_psense_noise * 2.0) {
                trigger_attack = true;
            }
        }

        // Execute the Pied Piper Spoof
        if (trigger_attack) 
        {
            float fake_precision = 10000.0;  

            for(auto* msg : fg.bb.pose_msgs)
            {
                if (msg) 
                {
                    // Force the swarm to believe the attacker is exactly the center (0,0)
                    msg->mu(0) = 0.0;
                    msg->mu(1) = 0.0;
                    
                    // Apply massive precision to overpower honest sensors
                    msg->lambda << fake_precision, 0.0, 
                                   0.0, fake_precision;
                    
                    // Update the GBP information vector
                    msg->eta = msg->lambda * msg->mu;
                }
            }
        }
    }

    // 3. Debugging output
    if(DEBUGFG || sim->sp.debug_fg){
        printf("Robot %d:%d Timestep %d blackboard\n%s", name, id, fg.timestep, fg.bb.repr().c_str());
        printf("Robot %d graph-------------------------------\n", name);
        fg.print();
    }
}
// yingfei

/* void Robot::update_blackboard()
{
    // ----------------------------------------------------------------------------
    // Populate our blackboard based on local graph
    fg.update_blackboard();

    if(DEBUGFG || sim->sp.debug_fg){
    printf("Robot %d:%d Timestep %d blackboard\n%s", name, id, fg.timestep, fg.bb.repr().c_str());
    printf("Robot %d graph-------------------------------\n", name);
    fg.print();
    }

}
 */

void Robot::add_sensed_object(uintptr_t sensed)
{
    sensed_objects.push_back(sensed);
    auto o =(Object*)sensed;
    if (o->otype == Otype::BOX)
        sensed_boxes[o->id] = (Static_box*)o;
    else if (o->otype == Otype::ROBOT)
    {
        sensed_robots[o->id] = Sr(timer::ticks(), (Robot*)o);
        seen_robots.insert(o->id);
    }
}

void Robot::remove_sensed_object(uintptr_t sensed)
{
    sensed_objects.erase(std::find(sensed_objects.begin(), sensed_objects.end(), sensed));
    auto o =(Object*)sensed;
    if (o->otype == Otype::BOX)
        sensed_boxes.erase(o->id);
    else if (o->otype == Otype::ROBOT)
        sensed_robots.erase(o->id);
}

void Robot::list_sensed(int type)
{
    for(auto s : sensed_boxes)
    {
        printf("rid:%2d sid:%2d\n", id, s.second->name);
    }

}



void Robot::render(DebugDraw *dd, Vector2f bias)
{
    b2Vec2 b(bias(0), bias(1));
    auto rgt = body->GetPosition();
    // float m = senses.in_shape ? 2.0 : 1.0;
    float m = 1.0;
    bool ccol = sim->sp.gui.aim_oldest && sim->sortbox.size() == sim->sp.num_boxes;
    b2Color c = ccol ? b2Color(0.6, 0.6, 0.6) : b2Color(m * colour.r, m * colour.g, m * colour.b);
    if (sim->sp.gui.robots)
    {   
        dd->DrawSolidCircle(rgt, radius, b2Vec2(cos(body->GetAngle()), sin(body->GetAngle())), c);
        // if(gbp_error > 0.2)
        //     dd->DrawShadedCircle(rgt, radius * 0.5, b2Color(0.1,0.1,0.1));
        if (sim->sp.gui.sensor)
        {
            dd->DrawCircle(rgt, sim->sp.r_sense, b2Color(0.3,0.7,0.6));
        }
        if (sim->sp.gui.sense_line)
        {
            for (auto s : sensed_robots)
            {
                dd->DrawSegment(body->GetPosition(), s.second.r->body->GetPosition(), b2Color(1.0, 0.0, 0.5));
            }
            for(auto s : sensed_boxes)
            {
                dd->DrawSegment(body->GetPosition(), s.second->body->GetPosition(), b2Color(1.0, 0.5, 0.0));
            }
        }

        if (senses.converged)
        {   
            dd->DrawCircle(rgt, 0.8 * radius, c);
        }

        if (sim->sp.gui.aim_oldest && sim->sortbox.size() == sim->sp.num_boxes)
        {
            int max = sim->sp.num_boxes > 10 ? 10 : sim->sp.num_boxes;
            for(int i = 0; i < max; i++)
            {
                if (chasing_name == sim->popbox[i].second)
                {
                    dd->DrawShadedCircle(rgt, 0.3, hsl2rgb(360 * chasing_name / sim->sp.num_boxes, 1.0, 0.8));
                }
            }

            auto &p = rel_observed_box_state[chasing_name].p - sim->system_bias;
            // dd->DrawThickSegment(body->GetPosition(), b2Vec2(p(0), p(1)), b2Color(0.2,0.2,0.2), 0.02);
            // dd->DrawShadedCircle(b2Vec2(p(0), p(1)), 0.3, b2Color(0.9,0.9,0.1));
        }

        // if (sim->sp.gui.erpos)
        // {
        //     MatrixXf cov = senses.position.get_cov();
        //     float r = sqrt(cov(0, 0));
        //     Vector2f est_pos = senses.position.mu - bias;
        //     // if (r > 1)
        //     //     return;
        //     dd->DrawCircle(b2Vec2(est_pos(0), est_pos(1)), r, colour);
        //     dd->DrawSegment(body->GetPosition(), b2Vec2(est_pos(0), est_pos(1)), colour);
        // }
        // if (sim->sp.gui.grad)
        // {
        //     float dx = 0.2 * cos(senses.angle);
        //     float dy = 0.2 * sin(senses.angle);
        //     dd->DrawSegment(body->GetPosition(), body->GetPosition() + b2Vec2(dx, dy), b2Color(1.0,1.0,0.0));
        //     dd->DrawSegment(body->GetPosition(), body->GetPosition() 
        //         + b2Vec2(senses.neighbours(0), senses.neighbours(1)), b2Color(1.0,1.0,0.0));
        // }


        if (sim->sp.gui.name)
        {
            dd->DrawFont(body->GetPosition() - b2Vec2(0, 0.02), 0.22, string_format("%d", name).c_str(), 1, b2Color(0,0,0));
        }
    }


    fg.render(dd, colour, senses.position.mu, to_eigen(rgt));

}


