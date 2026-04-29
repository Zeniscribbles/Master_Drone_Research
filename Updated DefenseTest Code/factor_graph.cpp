#include "factor_graph.hpp"
#include "robot.hpp"

constexpr int DEBUG = 0;
constexpr int DEBUGFG = 0;
constexpr int DEBUGFGCS = 0;

bool valid_outward_factor(Factor *f, int oid, int t)
{
    return (f->edges.size() == 2) && !f->edges[1].v && (f->other_id == oid) && (f->edges[0].v->otype == t);
}
// --------------------------------------------------------------------------------------------------
// Handle blackboard of another robot
// --------------------------------------------------------------------------------------------------
void Factor_graph::handle_blackboard(int name, Robot *me, Robot *other, bool handshake)
{
    // Look at the blackboard of the other robot.
    // The blackboard consists of three sets of data:
    //  1   Pose variable beliefs, one for each variable
    //  2   Factor to variable messages, one for each external factor
    //  3   Box variable beliefs
    // Outward facing factors have a NULL pointer in edge 1 var ptr. The uid of the robot, the oid 
    // of the factor message and the timestep in the factor provide the necessary information 
    // to connect the factors and place incoming messages on variables
    //
    // Variables that connect to a factor on another robot will have an edge created to hold the 
    // message from the factor.


    // Each variable can connect to many external factors, and is identified by category and timestep,
    // there can only be a single variable of a given category in a timestep.
    //
    // Each external factor can connect to only one variable on the other robot.

    // Bandwidth calculation
    // Assume we have handshakes, not a blackboard which we read.
    // The blackboard has:
    //  1) A set of pose variables, that may be destined for many factors
    //      A variable matches a factor if:
    //          a) the factor points to that robot
    //          b) their timestep is the same
    //      Send list of list of factor timesteps, ticks
    //      Receive list of matching messages
    //
    //  2) A set of factor outward messages, each destined for a single variable
    //      A factor matches a variable if:
    //          a) the factor points to this robot
    //          b) their timestep is the same
    //      Send list of variable timesteps, ticks
    //      Receive list of matching messages
    // 


    msg_start   = me;
    msg_end     = other;

    if (!handshake)
    {
        // ----------------------------------------------------------------------------
        // Find external variables that match with factors on this robot. An external
        // variable can connect with many factors on different robots
        int out_bytes = out_msg_overhead;
        int ret_bytes = ret_msg_overhead;
        int oid = other->id;
        for(auto &f : fac_nodes)
        {
            if (valid_outward_factor(f, oid, Otype::ROBOT))
            {
                // This factor connects to the other robot, so look on the other robot
                // for a matching variable, i.e. has the same timestamp, since, for a given outward factor,
                // only a single variable can be valid.
                out_bytes += out_msg_size;
                ret_bytes += ret_msg_size;
                auto t = f->timestep;
                for(auto &m : other->fg.bb.pose_msgs)
                {
                    if (m->timestep == t)
                    {
                        // This is the correct timestep, we can use this message
                        if (m->tick > f->edges[1].var_to_factor_msg.tick)
                        {
                            // Don't get the message unless it is newer than the current message
                            // from this source
                            //printf("robot %2d Extern var %d %d\n", name, f->edges[1].var_to_factor_msg.tick, m->tick);
                            f->edges[1].var_to_factor_msg = *m;
                            if(DEBUG)printf("Timestep %4d fac %d gets message %s\n", timestep, f->id, m->repr().c_str());
                            break;
                        }
                    }
                }
            }
        }
        // ----------------------------------------------------------------------------
        // Find external factors that match with variables on this robot. Each factor on the other
        // robot connects only one variable
        //out_bytes += var_nodes.size() * out_msg_size;
        for(auto &m : other->fg.bb.factor_msgs)
        {
            if (m->otype == Otype::ROBOT)
            {
                // Factors facing robot pose vars
                for(auto &v : var_nodes)
                {
                    if ((v->timestep == m->timestep) && (m->oid == me->id))
                    {
                        // Found a variable that matches with outwards factor message, now
                        // find the edge on the variable that connects to the factor, create the 
                        // edge if necessary
                        bool found = false;
                        for(auto &e : v->edges)
                        {
                            if (!e->f && e->robot_id == oid)
                            {
                                ret_bytes += ret_msg_size;
                                // An edge to this factor exists, put the message in it if its new
                                if (m->tick > e->factor_to_var_msg.tick)
                                {
                                    //printf("robot %2d Extern fac %d %d\n", name, e->factor_to_var_msg.tick, m->tick);
                                    e->factor_to_var_msg = *m;
                                    e->fresh = true;
                                }
                                found = true;
                                break;
                            }
                        }
                        if (!found)
                        {
                            // There is no edge, add one
                            v->edges.push_back(new Edge(nullptr, v));
                            v->edges.back()->robot_id           = oid;
                            v->edges.back()->factor_to_var_msg  = *m;
                            if(DEBUG)printf("Timestep %4d var %d robot %d gets edge to oid %d\n", timestep, v->id, name, oid);
                        }
                        if(DEBUG)printf("Timestep %4d var %d ts %d gets message %s\n", timestep, v->id, m->timestep, m->repr().c_str());

                        // Update the belief of this variable
                        v->update_belief();

                        // -------------------------------------------------------
                        // PERSON B: Signal 1 — Precision anomaly score S_lambda
                        // -------------------------------------------------------
                        {
                            float lambda_baseline = 1.0f / (sim->sp.sigma_psense_est
                                                          * sim->sp.sigma_psense_est);
                            float lambda_in = m->lambda(0, 0);
                            float s_lambda  = lambda_in / (lambda_baseline + 1.0f);
                            // Keep the max seen this timestep across all incoming messages
                            if (s_lambda > me->s_lambda)
                                me->s_lambda = s_lambda;
                        }

                        // -------------------------------------------------------
                        // PERSON B: Signal 2 — Belief divergence D_KL
                        // Measures inconsistency between incoming message and
                        // the local variable's current belief
                        // -------------------------------------------------------
                        if (v->belief.lambda(0,0) > 0 && m->lambda(0,0) > 0)
                        {
                            float lambda_q  = 1.0f / (sim->sp.sigma_psense_est
                                                     * sim->sp.sigma_psense_est);
                            float lx        = m->lambda(0, 0);
                            float ly        = m->lambda(1, 1) > 0 ? m->lambda(1,1) : lx;
                            Vector2f dmu    = v->belief.mu - m->mu;
                            float dkl       = 0.5f * (
                                lambda_q / lx +
                                lambda_q / ly +
                                lambda_q * dmu.squaredNorm() -
                                2.0f +
                                logf(lx / lambda_q) +
                                logf(ly / lambda_q)
                            );
                            if (dkl < 0) dkl = 0;
                            if (dkl > me->s_dkl)
                                me->s_dkl = dkl;
                        }
                        // -------------------------------------------------------


                        // Stop looking through vars, only one can match
                        break;
                    }
                }
            }
        }
        // 
        int bb_bytes = (other->fg.bb.factor_msgs.size() + other->fg.bb.pose_msgs.size()) * ret_msg_size;

        me->total_out_bytes += out_bytes;
        me->total_ret_bytes += ret_bytes;
        me->total_bb_bytes += bb_bytes;
    }
    else
    {
        // Handshake packet exchange method.
        //
        // We send a set of outgoing factor messages, each of which connects to
        // a single variable. 
        // The other robot replies with the set of beliefs of the variables
        // that connect to those factors.
        // The other robot also replies with 
        //

    }
    // if (out_bytes > 0 || ret_bytes > 0)
    //     printf("Robot %2d : robot %2d sent %4d recv %4d tot %4d (%4d)\n", me->id, oid, out_bytes, ret_bytes, out_bytes + ret_bytes, bb_bytes);
}


// --------------------------------------------------------------------------------------------------
// Update our blackboard
// --------------------------------------------------------------------------------------------------
void Factor_graph::update_blackboard()
{

    //bb.factor_msgs.clear();
    int last_name = -1;
    int last_oid = -1;
    int index = 0;
    for(auto &f : fac_nodes)
    {
        if (f->edges.size() == 2 && !f->edges[1].v)
        {
            // Outward facing factor, generate message
            auto m = &(f->edges[1].factor_to_var_msg);
            if (m->lambda(0, 0) == 0)
            {
                // This message has never been set
                flops_create += f->send_message(0, sim->sp.damping);
            }
            m->otype    = f->edges[0].v->otype;
            m->oid      = f->other_id;
            m->id       = f->id;
            m->timestep = f->edges[0].v->timestep;
            m->box      = f->edges[0].v->name;

            if (f->edges[0].v->otype == Otype::BOX)
            {
                if(DEBUG)printf("Adding box factor bb message %f %f name %d oid %d\n",m->mu(0), m->mu(1) , f->edges[0].v->name, f->other_id);
            }
            else
            {
                if(DEBUG)printf("Adding robot factor bb message %f %f %d\n", m->mu(0), m->mu(1), m->oid);
            }

            // Avoid expensive operations once message buffer is big enough
            if (bb.factor_msgs.size() > index)
                bb.factor_msgs[index] = m;
            else
                bb.factor_msgs.push_back(m);
            
            index++;
        }
    }
    bb.factor_msgs.resize(index);

    // bb.pose_msgs.clear();
    index = 0;
    for(auto &v : var_nodes)
    {
        if (v->otype == Otype::ROBOT)
        {
            v->belief.id = v->id;
            if (bb.pose_msgs.size() > index)
                bb.pose_msgs[index] = &(v->belief);
            else
                bb.pose_msgs.push_back(&(v->belief));
            index++;
        }
    }
    bb.pose_msgs.resize(index);


}

void Factor_graph::add_robot_pose(Vector2f meas, Vector2f sigma, Vector2f prior)
{
    // Create a robot pose variable and link it to the last pose if it exists.
    // Link using a measurement factor. If there is no previous pose, use a weak
    // anchor factor
    // auto v = new Variable(timestep, Gaussian(prior, sim->sp.sigma_prior));
    // auto v = new Variable(timestep, Gaussian(prior, sigma));
    auto v = new Variable(timestep, Gaussian(Vector2f(0,0), sim->sp.sigma_prior));
    v->otype = Otype::ROBOT;
    var_nodes.push_back(v);
    if (last_robot_pose)
    {
        auto f = new Meas_factor(uid(), timestep, {last_robot_pose, v}, meas, sigma);
        fac_nodes.push_back(f);

        if (sim->sp.debug_msgs)
        {
            printf("New meas id:%5d %5d %5d % 6.3f % 6.3f % 6.3f\n", 
                f->id, last_robot_pose->id, v->id, meas(0), meas(1), sigma(0));
            if (sim->sp.fp)fprintf(sim->sp.fp, "New meas id:%5d %5d %5d % 6.3f % 6.3f % 6.3f\n", 
                f->id, last_robot_pose->id, v->id, meas(0), meas(1), sigma(0));
        }


        // flops += f->send_and_get_message(last_robot_pose->id, sim->sp.damping);
        flops_create += f->send_and_get_message(v->id, sim->sp.damping);
    }
    else
    {
        auto f = new Pose_factor(uid(), timestep, {v}, prior, sim->sp.sigma_anchor_est);
        if (sim->sp.debug_msgs)
        {   printf("===> New pose id:%5d %5d % 6.3f % 6.3f % 6.3f % 6.3f % 6.3f\n", 
                f->id, v->id, meas(0), meas(1), prior(0), prior(1), sigma(0));
            if (sim->sp.fp)fprintf(sim->sp.fp, "===> New pose id:%5d %5d % 6.3f % 6.3f % 6.3f % 6.3f % 6.3f\n", 
                f->id, v->id, meas(0), meas(1), prior(0), prior(1), sigma(0));
        }
        fac_nodes.push_back(f);
        // flops += f->send_and_get_message(v->id, sim->sp.damping);
    }
    last_robot_pose = v;

   
}

void Factor_graph::add_other_robot_pose(Vector2f meas, Vector2f sigma, int id)
{
    // Create a factor to a foreign robot. This creates a edge from the
    // factor but leaves it unconnected, set at NULL. The incoming message
    // for the edge is filled in from the blackboard of the other robot
    auto f = new Meas_factor(uid(), timestep, {last_robot_pose, NULL}, -meas, sigma, id);

    // flops += last_robot_pose->update_belief();
    //f->send_and_get_message(last_robot_pose->id);
    //f->send_message(0);

    // Hack in an exact but weak belief, this will be overwritten when blackboard first evaluated
    Gaussian b(last_robot_pose->belief.mu - Vector2f(meas(0), meas(1)), 10.0);
    f->edges[1].var_to_factor_msg = b;
    fac_nodes.push_back(f);
}

void Factor_graph::delete_factors(std::vector<Factor *> &facs_to_delete)
{
    // Call this once the edge(s) on a factor have been removed from their variables
    while (facs_to_delete.size() > 0)
    {
        auto &f = facs_to_delete.back();
        facs_to_delete.pop_back();
        for(int i = 0; i < fac_nodes.size(); i++)
        {
            if (fac_nodes[i] == f)
            {
                fac_nodes.erase(fac_nodes.begin() + i);
                if(DEBUG)printf("Deleting factor %d\n", f->id);
                delete(f);
                break;
            }
        }
    }
}

// --------------------------------------------------------------------------------------------------
// Prune the factor graph to keep the number of robot pose variables constant
// --------------------------------------------------------------------------------------------------
void Factor_graph::prune()
{
    Matrix2f c(Matrix2f::Zero());
    std::vector<Factor *> facs_to_delete;
    while(var_nodes.size() > sim->sp.fg_window)
    {
        // Scan down list of nodes from oldest to youngest, removing nodes until
        // the size is small enough. 
        // 
        // To remove a node, we must first delete all factors connecting to it.
        // In removing a 2-edge factor, we need to remove the pointer to it at the other
        // end. When a var node is connected to a factor on another robot,
        // an edge is created with a nullptr where the factor pointer is, this needs
        // to be deleted here because it is not owned by a factor
        auto v = var_nodes[0];
        for(auto &e : v->edges)
        {
            auto &f = e->f;
            if (!f)
            {
                // Var owned edge that connects to a factor on another robot, delete
                if(DEBUG)printf("Deleting edge connecting to %d\n", e->robot_id);
                delete(e);
                continue;
            }
            else if (f->edges.size() == 1)
            {
                // Anchor node, capture the cov matrix
                c = f->factor.get_cov();
            }
            else if (f->edges.size() == 2)
            {
                // There are two cases for 2-edge factors:
                //  1)  connect to another var - need to delete edge on other
                //  2)  connect to another robot - can just delete
                //
                // If there is no anchor factor on the next var for case 1, we need to create a 
                // new anchor var that combines the anchor factor on the current oldest var with 
                // the measurement factor connecting oldest to next oldest.
                if (f->edges[1].v)
                {
                    // case 1, 2
                    auto n = f->edges[1].v;
                    if (n->otype == Otype::ROBOT)
                    {
                        // Case 1, we need to construct a new anchor, work out the 
                        // covariance here
                        c = c + f->factor.get_cov();
                    }

                    // Now delete the edge from any vars referencing it
                    auto ea = &(f->edges[1]);   
                    for(int i = 0; i < n->edges.size(); i++)
                    {
                        if (n->edges[i] == ea)
                        {
                            // Delete this edge from var node
                            n->edges.erase(n->edges.begin() + i);
                            break;
                        }
                    }
                }
            }
            // Can now delete the factor, save on the list
            facs_to_delete.push_back(f);
        }

        delete_factors(facs_to_delete);

        var_nodes.pop_front();
        if(DEBUG)printf("Deleting node %d\n", v->id);
        delete(v);

        // Make sure there is an anchor on now oldest node and on all boxes
        v = var_nodes[0];
        add_anchor(v);

        // Set the last factor-to-var message on connecting 2-edge factors to the belief of
        // the variable, this will otherwise be used to compute the implied var-to-factor message 
        // wrongly, causing oscillations
        for (auto &e : v->edges)
        {
            e->factor_to_var_msg = v->belief;
        }

        if(DEBUG)printf("Robot %d\n", name);

    }
}


Factor *Factor_graph::find_factor(Variable *v0, Variable *v1)
{
    for(auto f : fac_nodes)
    {
        if (f->edges.size() == 2 && f->edges[0].v == v0 && f->edges[1].v == v1)
            return f;
        if (f->edges.size() == 2 && f->edges[0].v == v1 && f->edges[1].v == v0)
            return f;
    }
    return nullptr;
}

void Factor_graph::add_anchor(Variable *v)
{
    // if (v->belief.lambda(0, 0) == 0)
    //     add_anchor(v, 0.1);
    // else
    // {
    Matrix2f c(v->belief.get_cov());
    float sigma = sqrt(c(0, 0));
    if (isnan(sigma))
        add_anchor(v, 0.1);
    else
        add_anchor(v, sigma);
}
void Factor_graph::add_anchor(Variable *v, float sigma, Vector2f bias)
{
    float s2 = sigma * sigma;
    Matrix2f c;
    c << s2, 0, 0, s2;
    add_anchor(v, c, bias);
}
void Factor_graph::add_anchor(Variable *v, Matrix2f cov, Vector2f bias)
{
    // Add anchor with current belief if there isn't one already. If there is,
    // update it to the current belief
    bool found = false;
    Vector2f prior(v->belief.mu - bias);
    for(auto &e : v->edges)
    {
        if (e->f && e->f->edges.size() == 1)
        {
            found = true;
            e->f->z = prior;
            e->f->factor.lambda = cov.inverse();
            e->f->edges[0].var_to_factor_msg = v->belief;
            break;
        }
    }
    if (!found)
    {
        // auto f = new Pose_factor(uid(), timestep, {v}, prior, sqrt(cov(0, 0)));
        //printf("Didn't find anchor!!=========================================\n");
        auto f = new Pose_factor(uid(), timestep, {v}, prior, sqrt(cov(0, 0)));
        fac_nodes.push_back(f); 
        f->edges[0].var_to_factor_msg = v->belief;
        // flops += f->send_message(v->id, sim->sp.damping);
    }
}

// --------------------------------------------------------------------------------------------------
// Perform a single iteration on the factor graph
// --------------------------------------------------------------------------------------------------
float Factor_graph::do_iteration(int iterations)
{
    if (!fac_nodes.size())
        return 0;
    for(int i = 0; i < iterations; i++)
    {
        //---------------------------------------------------------------
        // Choose a random factor and send messages to all edges
        auto f = fac_nodes[randi(0, fac_nodes.size() - 1)];

        for(auto &e : f->edges)
        {

            // Pre message debug output
            if (sim->sp.debug_msgs)
            {
                printf("fac  %4d %s|%s\n", f->id, f->repr().c_str(), f->fr().c_str());
                if (sim->sp.fp)fprintf(sim->sp.fp, "fac  %4d %s|%s\n", f->id, f->repr().c_str(), f->fr().c_str());
                if (e.v)
                {
                    printf("vpre %4d %s\n", e.v->id, e.v->belief.repr().c_str());
                        if (sim->sp.fp)fprintf(sim->sp.fp, "vpre %4d %s\n", e.v->id, e.v->belief.repr().c_str());
                }
            }

            //---------------------------------------------------------------
            // Send message and if there is a connecting variable on the edge, get
            // corresponding variable belief
            if (e.v)
                flops_update += f->send_and_get_message(e.v->id, sim->sp.damping);
            else
                flops_update += f->send_message(0, sim->sp.damping);

            f_iterated = f;
            //---------------------------------------------------------------
            

            // Post message debug output
            if (sim->sp.debug_msgs)
            {
                printf("robot %d fac id:%5d to var id:%5d %s  |  %s\n", name, f->id, e.v ? e.v->id : 0, 
                    e.factor_to_var_msg.repr().c_str(), e.var_to_factor_msg.repr().c_str());
                if (sim->sp.fp)
                    fprintf(sim->sp.fp, "robot %d fac id:%5d to var id:%5d %s  |  %s\n", name, f->id, e.v ? e.v->id : 0, 
                        e.factor_to_var_msg.repr().c_str(), e.var_to_factor_msg.repr().c_str());
                if (e.v)
                {
                    printf("vpst %4d %s\n", e.v->id, e.v->belief.repr().c_str());
                    if (fabs(e.v->belief.mu(0)) < 0.001 && fabs(e.v->belief.mu(1)) < 0.001)
                    {
                        printf("=============================================>\n");
                    }
                    if (sim->sp.fp)
                    {
                        fprintf(sim->sp.fp, "vpst %4d %s\n", e.v->id, e.v->belief.repr().c_str());
                        if (fabs(e.v->belief.mu(0)) < 0.001 && fabs(e.v->belief.mu(1)) < 0.001)
                        {
                            fprintf(sim->sp.fp, "=============================================>\n");
                        }
                    }
                }
                printf("fac  %4d %s|%s\n", f->id, f->repr().c_str(), f->fr().c_str());
                if (sim->sp.fp)fprintf(sim->sp.fp, "fac  %4d %s|%s\n", f->id, f->repr().c_str(), f->fr().c_str());
            }
        }
    }
    return 0;
}


void Factor_graph::print()
{
    for(int v = 0; v < var_nodes.size(); v++)
    {
        var_nodes[v]->print();
        auto &es = var_nodes[v]->edges;
        for(auto &e : es)
        {
            if (e->f)
                e->f->print();
            else
                printf("    Ext  id %5d\n", e->robot_id);
        }
    }
    printf("Variables: %4d Factors: %4d\n", (int)var_nodes.size(), (int)fac_nodes.size());
}


ThickLine::ThickLine(b2Vec2 _s, b2Vec2 _e, b2Color _c, float _t)
:   start       (_s),
    end         (_e),
    colour      (_c),
    thickness   (_t) 
    {
        ticks = timer::ticks();
    }
ThickLine::ThickLine(Robot *_s, Robot *_e, b2Color _c, float _t)
:   rs          (_s),
    re          (_e),
    colour      (_c),
    thickness   (_t) 
    {
        ticks = timer::ticks();
    }
void ThickLine::render(DebugDraw *dd, Sim *sim)
{
    int age = timer::ticks() - ticks;
    float alpha = colour.a * (1.0 - ((float)age / sim->sp.decay_time));
    b2Color tlc(
        colour.r * sim->sp.lighten, 
        colour.g * sim->sp.lighten, 
        colour.b * sim->sp.lighten, 
        alpha);
    if (rs)
    {
        start = rs->body->GetPosition();
        end = re->body->GetPosition();
    }
    dd->DrawThickSegment(start, end, tlc, 0.05);
}



void Factor_graph::render(DebugDraw *dd, b2Color colour, Vector2f est_pos, Vector2f rgt)
{
    // Beliefs may have a bias compared to ground truth, but it is global and irrelevant to
    // the robot. We adjust here for the purpose of visualisation.
    // auto bias = -sim->system_bias;
    if (!var_nodes.size() || !var_nodes.back()->updated)
        return;

    auto bias = sim->sp.gui.rbias ? -me->bias : -sim->system_bias;

    if (sim->sp.gui.fg)
    {
        for(auto &v : var_nodes)
        {
            if (!v->updated)
                continue;

            auto bl = v->belief.mu + bias;

            MatrixXf cov = v->belief.get_cov();

            float r = sqrt(cov(0,0));
            if (r < 0.5 * sim->sp.sigma_anchor_est)
            {
                dd->DrawCircle(b2Vec2(bl(0), bl(1)), r, colour);

                for(auto &e : v->edges)
                {
                    if (e->f && e->f->edges.size() == 1)
                    {
                        float s = r * 0.707;
                        auto p0 = bl + Vector2f(-s, -s);
                        auto p1 = bl + Vector2f( s,  s);
                        auto p2 = bl + Vector2f( s, -s);
                        auto p3 = bl + Vector2f(-s,  s);
                        dd->DrawSegment(b2Vec2(p0(0), p0(1)), b2Vec2(p1(0), p1(1)), colour);
                        dd->DrawSegment(b2Vec2(p2(0), p2(1)), b2Vec2(p3(0), p3(1)), colour);
                        break;
                    }
                }
            }


        }

        if(sim->sp.gui.factor)for(auto &f : fac_nodes)
        {
            if (!f->updated)
                continue;
            if (f->edges.size() == 2)
            {
                // auto c = me->colour;
                auto c = b2Color(0.9, 0.9, 0.9);
                if (!f->edges[1].v && f->edges[0].v->updated)
                {
                    // Foreign variable is NULL, draw a line to the belief in the message
                    auto p0 = f->edges[0].v->belief.mu + bias;
                    auto p1 = f->edges[1].var_to_factor_msg.mu + bias;
                    if (f->edges[0].v->otype != Otype::BOX)
                        dd->DrawSegment(b2Vec2(p0(0), p0(1)), b2Vec2(p1(0), p1(1)), c);
                }
                else if (f->edges[0].v->updated && f->edges[1].v->updated)
                {
                    // if (f->edges[0].v->belief.mu == Vector2f::Zero())
                    // {
                    //     printf("zero belief %5d\n", f->edges[0].v->id);
                    // }
                    auto p0 = f->edges[0].v->belief.mu + bias;
                    auto p1 = f->edges[1].v->belief.mu + bias;


                    b2Color c = colour;
                    c.a = sqrt(f->factor.lambda(0,0)) * 0.01;
                    dd->DrawSegment(b2Vec2(p0(0), p0(1)), b2Vec2(p1(0), p1(1)), c);
                }
            }
        }
    }


    // -------------------------------------------------------
    // Animate message send to other robot. msg_start and end point to robots
    // at each end of message transfer. Null pointer means no message.
    if (sim->sp.gui.msg)
    {
        // b2Color c = b2Color(0.0, 0.9, 0.7);
        if (msg_start)
        {
            // dd->DrawThickSegment(msg_start->body->GetPosition(), msg_end->body->GetPosition(), c, 0.05);

            b2Color c = b2Color(
                (msg_start->colour.r + msg_end->colour.r) / 2,
                (msg_start->colour.g + msg_end->colour.g) / 2,
                (msg_start->colour.b + msg_end->colour.b) / 2,
                (msg_start->colour.a + msg_end->colour.a) / 2
            );

            if (sim->sp.gui.mono)c = b2Color(0,0,0,1);
            c.a *= sim->sp.alpha;
            tlines.push_back(ThickLine(msg_start, msg_end, c, 0.05));
            msg_start = 0;
            msg_end = 0;
        }

        if (sim->sp.gui.fg && f_iterated)
        {
            auto c = me->colour;
            if (sim->sp.gui.mono)c = b2Color(0,0,0,1);
            c.a *= sim->sp.alpha;
            auto &f = f_iterated;
            if (f->edges.size() == 2)
            {
                if (!f->edges[1].v && f->edges[0].v->updated)
                {
                    // Foreign variable is NULL, draw a line to the belief in the message
                    auto p0 = f->edges[0].v->belief.mu + bias;
                    auto p1 = f->edges[1].var_to_factor_msg.mu + bias;
                    if (f->edges[0].v->otype != Otype::BOX)
                    {
                        // dd->DrawThickSegment(b2Vec2(p0(0), p0(1)), b2Vec2(p1(0), p1(1)), c, 0.05);
                        tlines.push_back(ThickLine(b2Vec2(p0(0), p0(1)), b2Vec2(p1(0), p1(1)), c, 0.05));
                    }
                }
                else if ( f->edges[0].v->updated && f->edges[1].v->updated)
                {
                    auto p0 = f->edges[0].v->belief.mu + bias;
                    auto p1 = f->edges[1].v->belief.mu + bias;

                    // dd->DrawThickSegment(b2Vec2(p0(0), p0(1)), b2Vec2(p1(0), p1(1)), c, 0.05);
                    tlines.push_back(ThickLine(b2Vec2(p0(0), p0(1)), b2Vec2(p1(0), p1(1)), c, 0.05));
                }
            }
            f_iterated = 0;
        }
        // Decay time is the number of sim ticks a drawn line takes to disappear
        // int decay_time = 15;
        // Remove old lines
        auto &decay_time = sim->sp.decay_time;
        tlines.erase(
            remove_if(
                tlines.begin(), 
                tlines.end(), 
                [&decay_time](ThickLine &a){return timer::ticks() - a.ticks > decay_time;}
            ), 
            tlines.end());
        // Draw the remaining lines with increasing alpha based on age
        for(auto &l : tlines)
        {#include "factor_graph.hpp"
#include "robot.hpp"

constexpr int DEBUG = 0;
constexpr int DEBUGFG = 0;
constexpr int DEBUGFGCS = 0;

bool valid_outward_factor(Factor *f, int oid, int t)
{
    return (f->edges.size() == 2) && !f->edges[1].v && (f->other_id == oid) && (f->edges[0].v->otype == t);
}
// --------------------------------------------------------------------------------------------------
// Handle blackboard of another robot
// --------------------------------------------------------------------------------------------------
void Factor_graph::handle_blackboard(int name, Robot *me, Robot *other, bool handshake)
{
    // Look at the blackboard of the other robot.
    // The blackboard consists of three sets of data:
    //  1   Pose variable beliefs, one for each variable
    //  2   Factor to variable messages, one for each external factor
    //  3   Box variable beliefs
    // Outward facing factors have a NULL pointer in edge 1 var ptr. The uid of the robot, the oid 
    // of the factor message and the timestep in the factor provide the necessary information 
    // to connect the factors and place incoming messages on variables
    //
    // Variables that connect to a factor on another robot will have an edge created to hold the 
    // message from the factor.


    // Each variable can connect to many external factors, and is identified by category and timestep,
    // there can only be a single variable of a given category in a timestep.
    //
    // Each external factor can connect to only one variable on the other robot.

    // Bandwidth calculation
    // Assume we have handshakes, not a blackboard which we read.
    // The blackboard has:
    //  1) A set of pose variables, that may be destined for many factors
    //      A variable matches a factor if:
    //          a) the factor points to that robot
    //          b) their timestep is the same
    //      Send list of list of factor timesteps, ticks
    //      Receive list of matching messages
    //
    //  2) A set of factor outward messages, each destined for a single variable
    //      A factor matches a variable if:
    //          a) the factor points to this robot
    //          b) their timestep is the same
    //      Send list of variable timesteps, ticks
    //      Receive list of matching messages
    // 


    msg_start   = me;
    msg_end     = other;

    if (!handshake)
    {
        // ----------------------------------------------------------------------------
        // Find external variables that match with factors on this robot. An external
        // variable can connect with many factors on different robots
        int out_bytes = out_msg_overhead;
        int ret_bytes = ret_msg_overhead;
        int oid = other->id;
        for(auto &f : fac_nodes)
        {
            if (valid_outward_factor(f, oid, Otype::ROBOT))
            {
                // This factor connects to the other robot, so look on the other robot
                // for a matching variable, i.e. has the same timestamp, since, for a given outward factor,
                // only a single variable can be valid.
                out_bytes += out_msg_size;
                ret_bytes += ret_msg_size;
                auto t = f->timestep;
                for(auto &m : other->fg.bb.pose_msgs)
                {
                    if (m->timestep == t)
                    {
                        // This is the correct timestep, we can use this message
                        if (m->tick > f->edges[1].var_to_factor_msg.tick)
                        {
                            // Don't get the message unless it is newer than the current message
                            // from this source
                            //printf("robot %2d Extern var %d %d\n", name, f->edges[1].var_to_factor_msg.tick, m->tick);
                            f->edges[1].var_to_factor_msg = *m;
                            if(DEBUG)printf("Timestep %4d fac %d gets message %s\n", timestep, f->id, m->repr().c_str());
                            break;
                        }
                    }
                }
            }
        }
        // ----------------------------------------------------------------------------
        // Find external factors that match with variables on this robot. Each factor on the other
        // robot connects only one variable
        //out_bytes += var_nodes.size() * out_msg_size;
        for(auto &m : other->fg.bb.factor_msgs)
        {
            if (m->otype == Otype::ROBOT)
            {
                // Factors facing robot pose vars
                for(auto &v : var_nodes)
                {
                    if ((v->timestep == m->timestep) && (m->oid == me->id))
                    {
                        // Found a variable that matches with outwards factor message, now
                        // find the edge on the variable that connects to the factor, create the 
                        // edge if necessary
                        bool found = false;
                        for(auto &e : v->edges)
                        {
                            if (!e->f && e->robot_id == oid)
                            {
                                ret_bytes += ret_msg_size;
                                // An edge to this factor exists, put the message in it if its new
                                if (m->tick > e->factor_to_var_msg.tick)
                                {
                                    //printf("robot %2d Extern fac %d %d\n", name, e->factor_to_var_msg.tick, m->tick);
                                    e->factor_to_var_msg = *m;
                                    e->fresh = true;
                                }
                                found = true;
                                break;
                            }
                        }
                        if (!found)
                        {
                            // There is no edge, add one
                            v->edges.push_back(new Edge(nullptr, v));
                            v->edges.back()->robot_id           = oid;
                            v->edges.back()->factor_to_var_msg  = *m;
                            if(DEBUG)printf("Timestep %4d var %d robot %d gets edge to oid %d\n", timestep, v->id, name, oid);
                        }
                        if(DEBUG)printf("Timestep %4d var %d ts %d gets message %s\n", timestep, v->id, m->timestep, m->repr().c_str());

                        // Update the belief of this variable
                        v->update_belief();

                        // -------------------------------------------------------
                        // PERSON B: Signal 1 — Precision anomaly score S_lambda
                        // -------------------------------------------------------
                        {
                            float lambda_baseline = 1.0f / (sim->sp.sigma_psense_est
                                                          * sim->sp.sigma_psense_est);
                            float lambda_in = m->lambda(0, 0);
                            float s_lambda  = lambda_in / (lambda_baseline + 1.0f);
                            // Keep the max seen this timestep across all incoming messages
                            if (s_lambda > me->s_lambda)
                                me->s_lambda = s_lambda;
                        }

                        // -------------------------------------------------------
                        // PERSON B: Signal 2 — Belief divergence D_KL
                        // Measures inconsistency between incoming message and
                        // the local variable's current belief
                        // -------------------------------------------------------
                        if (v->belief.lambda(0,0) > 0 && m->lambda(0,0) > 0)
                        {
                            float lambda_q  = 1.0f / (sim->sp.sigma_psense_est
                                                     * sim->sp.sigma_psense_est);
                            float lx        = m->lambda(0, 0);
                            float ly        = m->lambda(1, 1) > 0 ? m->lambda(1,1) : lx;
                            Vector2f dmu    = v->belief.mu - m->mu;
                            float dkl       = 0.5f * (
                                lambda_q / lx +
                                lambda_q / ly +
                                lambda_q * dmu.squaredNorm() -
                                2.0f +
                                logf(lx / lambda_q) +
                                logf(ly / lambda_q)
                            );
                            if (dkl < 0) dkl = 0;
                            if (dkl > me->s_dkl)
                                me->s_dkl = dkl;
                        }
                        // -------------------------------------------------------


                        // Stop looking through vars, only one can match
                        break;
                    }
                }
            }
        }
        // 
        int bb_bytes = (other->fg.bb.factor_msgs.size() + other->fg.bb.pose_msgs.size()) * ret_msg_size;

        me->total_out_bytes += out_bytes;
        me->total_ret_bytes += ret_bytes;
        me->total_bb_bytes += bb_bytes;
    }
    else
    {
        // Handshake packet exchange method.
        //
        // We send a set of outgoing factor messages, each of which connects to
        // a single variable. 
        // The other robot replies with the set of beliefs of the variables
        // that connect to those factors.
        // The other robot also replies with 
        //

    }
    // if (out_bytes > 0 || ret_bytes > 0)
    //     printf("Robot %2d : robot %2d sent %4d recv %4d tot %4d (%4d)\n", me->id, oid, out_bytes, ret_bytes, out_bytes + ret_bytes, bb_bytes);
}


// --------------------------------------------------------------------------------------------------
// Update our blackboard
// --------------------------------------------------------------------------------------------------
void Factor_graph::update_blackboard()
{

    //bb.factor_msgs.clear();
    int last_name = -1;
    int last_oid = -1;
    int index = 0;
    for(auto &f : fac_nodes)
    {
        if (f->edges.size() == 2 && !f->edges[1].v)
        {
            // Outward facing factor, generate message
            auto m = &(f->edges[1].factor_to_var_msg);
            if (m->lambda(0, 0) == 0)
            {
                // This message has never been set
                flops_create += f->send_message(0, sim->sp.damping);
            }
            m->otype    = f->edges[0].v->otype;
            m->oid      = f->other_id;
            m->id       = f->id;
            m->timestep = f->edges[0].v->timestep;
            m->box      = f->edges[0].v->name;

            if (f->edges[0].v->otype == Otype::BOX)
            {
                if(DEBUG)printf("Adding box factor bb message %f %f name %d oid %d\n",m->mu(0), m->mu(1) , f->edges[0].v->name, f->other_id);
            }
            else
            {
                if(DEBUG)printf("Adding robot factor bb message %f %f %d\n", m->mu(0), m->mu(1), m->oid);
            }

            // Avoid expensive operations once message buffer is big enough
            if (bb.factor_msgs.size() > index)
                bb.factor_msgs[index] = m;
            else
                bb.factor_msgs.push_back(m);
            
            index++;
        }
    }
    bb.factor_msgs.resize(index);

    // bb.pose_msgs.clear();
    index = 0;
    for(auto &v : var_nodes)
    {
        if (v->otype == Otype::ROBOT)
        {
            v->belief.id = v->id;
            if (bb.pose_msgs.size() > index)
                bb.pose_msgs[index] = &(v->belief);
            else
                bb.pose_msgs.push_back(&(v->belief));
            index++;
        }
    }
    bb.pose_msgs.resize(index);


}

void Factor_graph::add_robot_pose(Vector2f meas, Vector2f sigma, Vector2f prior)
{
    // Create a robot pose variable and link it to the last pose if it exists.
    // Link using a measurement factor. If there is no previous pose, use a weak
    // anchor factor
    // auto v = new Variable(timestep, Gaussian(prior, sim->sp.sigma_prior));
    // auto v = new Variable(timestep, Gaussian(prior, sigma));
    auto v = new Variable(timestep, Gaussian(Vector2f(0,0), sim->sp.sigma_prior));
    v->otype = Otype::ROBOT;
    var_nodes.push_back(v);
    if (last_robot_pose)
    {
        auto f = new Meas_factor(uid(), timestep, {last_robot_pose, v}, meas, sigma);
        fac_nodes.push_back(f);

        if (sim->sp.debug_msgs)
        {
            printf("New meas id:%5d %5d %5d % 6.3f % 6.3f % 6.3f\n", 
                f->id, last_robot_pose->id, v->id, meas(0), meas(1), sigma(0));
            if (sim->sp.fp)fprintf(sim->sp.fp, "New meas id:%5d %5d %5d % 6.3f % 6.3f % 6.3f\n", 
                f->id, last_robot_pose->id, v->id, meas(0), meas(1), sigma(0));
        }


        // flops += f->send_and_get_message(last_robot_pose->id, sim->sp.damping);
        flops_create += f->send_and_get_message(v->id, sim->sp.damping);
    }
    else
    {
        auto f = new Pose_factor(uid(), timestep, {v}, prior, sim->sp.sigma_anchor_est);
        if (sim->sp.debug_msgs)
        {   printf("===> New pose id:%5d %5d % 6.3f % 6.3f % 6.3f % 6.3f % 6.3f\n", 
                f->id, v->id, meas(0), meas(1), prior(0), prior(1), sigma(0));
            if (sim->sp.fp)fprintf(sim->sp.fp, "===> New pose id:%5d %5d % 6.3f % 6.3f % 6.3f % 6.3f % 6.3f\n", 
                f->id, v->id, meas(0), meas(1), prior(0), prior(1), sigma(0));
        }
        fac_nodes.push_back(f);
        // flops += f->send_and_get_message(v->id, sim->sp.damping);
    }
    last_robot_pose = v;

   
}

void Factor_graph::add_other_robot_pose(Vector2f meas, Vector2f sigma, int id)
{
    // Create a factor to a foreign robot. This creates a edge from the
    // factor but leaves it unconnected, set at NULL. The incoming message
    // for the edge is filled in from the blackboard of the other robot
    auto f = new Meas_factor(uid(), timestep, {last_robot_pose, NULL}, -meas, sigma, id);

    // flops += last_robot_pose->update_belief();
    //f->send_and_get_message(last_robot_pose->id);
    //f->send_message(0);

    // Hack in an exact but weak belief, this will be overwritten when blackboard first evaluated
    Gaussian b(last_robot_pose->belief.mu - Vector2f(meas(0), meas(1)), 10.0);
    f->edges[1].var_to_factor_msg = b;
    fac_nodes.push_back(f);
}

void Factor_graph::delete_factors(std::vector<Factor *> &facs_to_delete)
{
    // Call this once the edge(s) on a factor have been removed from their variables
    while (facs_to_delete.size() > 0)
    {
        auto &f = facs_to_delete.back();
        facs_to_delete.pop_back();
        for(int i = 0; i < fac_nodes.size(); i++)
        {
            if (fac_nodes[i] == f)
            {
                fac_nodes.erase(fac_nodes.begin() + i);
                if(DEBUG)printf("Deleting factor %d\n", f->id);
                delete(f);
                break;
            }
        }
    }
}

// --------------------------------------------------------------------------------------------------
// Prune the factor graph to keep the number of robot pose variables constant
// --------------------------------------------------------------------------------------------------
void Factor_graph::prune()
{
    Matrix2f c(Matrix2f::Zero());
    std::vector<Factor *> facs_to_delete;
    while(var_nodes.size() > sim->sp.fg_window)
    {
        // Scan down list of nodes from oldest to youngest, removing nodes until
        // the size is small enough. 
        // 
        // To remove a node, we must first delete all factors connecting to it.
        // In removing a 2-edge factor, we need to remove the pointer to it at the other
        // end. When a var node is connected to a factor on another robot,
        // an edge is created with a nullptr where the factor pointer is, this needs
        // to be deleted here because it is not owned by a factor
        auto v = var_nodes[0];
        for(auto &e : v->edges)
        {
            auto &f = e->f;
            if (!f)
            {
                // Var owned edge that connects to a factor on another robot, delete
                if(DEBUG)printf("Deleting edge connecting to %d\n", e->robot_id);
                delete(e);
                continue;
            }
            else if (f->edges.size() == 1)
            {
                // Anchor node, capture the cov matrix
                c = f->factor.get_cov();
            }
            else if (f->edges.size() == 2)
            {
                // There are two cases for 2-edge factors:
                //  1)  connect to another var - need to delete edge on other
                //  2)  connect to another robot - can just delete
                //
                // If there is no anchor factor on the next var for case 1, we need to create a 
                // new anchor var that combines the anchor factor on the current oldest var with 
                // the measurement factor connecting oldest to next oldest.
                if (f->edges[1].v)
                {
                    // case 1, 2
                    auto n = f->edges[1].v;
                    if (n->otype == Otype::ROBOT)
                    {
                        // Case 1, we need to construct a new anchor, work out the 
                        // covariance here
                        c = c + f->factor.get_cov();
                    }

                    // Now delete the edge from any vars referencing it
                    auto ea = &(f->edges[1]);   
                    for(int i = 0; i < n->edges.size(); i++)
                    {
                        if (n->edges[i] == ea)
                        {
                            // Delete this edge from var node
                            n->edges.erase(n->edges.begin() + i);
                            break;
                        }
                    }
                }
            }
            // Can now delete the factor, save on the list
            facs_to_delete.push_back(f);
        }

        delete_factors(facs_to_delete);

        var_nodes.pop_front();
        if(DEBUG)printf("Deleting node %d\n", v->id);
        delete(v);

        // Make sure there is an anchor on now oldest node and on all boxes
        v = var_nodes[0];
        add_anchor(v);

        // Set the last factor-to-var message on connecting 2-edge factors to the belief of
        // the variable, this will otherwise be used to compute the implied var-to-factor message 
        // wrongly, causing oscillations
        for (auto &e : v->edges)
        {
            e->factor_to_var_msg = v->belief;
        }

        if(DEBUG)printf("Robot %d\n", name);

    }
}


Factor *Factor_graph::find_factor(Variable *v0, Variable *v1)
{
    for(auto f : fac_nodes)
    {
        if (f->edges.size() == 2 && f->edges[0].v == v0 && f->edges[1].v == v1)
            return f;
        if (f->edges.size() == 2 && f->edges[0].v == v1 && f->edges[1].v == v0)
            return f;
    }
    return nullptr;
}

void Factor_graph::add_anchor(Variable *v)
{
    // if (v->belief.lambda(0, 0) == 0)
    //     add_anchor(v, 0.1);
    // else
    // {
    Matrix2f c(v->belief.get_cov());
    float sigma = sqrt(c(0, 0));
    if (isnan(sigma))
        add_anchor(v, 0.1);
    else
        add_anchor(v, sigma);
}
void Factor_graph::add_anchor(Variable *v, float sigma, Vector2f bias)
{
    float s2 = sigma * sigma;
    Matrix2f c;
    c << s2, 0, 0, s2;
    add_anchor(v, c, bias);
}
void Factor_graph::add_anchor(Variable *v, Matrix2f cov, Vector2f bias)
{
    // Add anchor with current belief if there isn't one already. If there is,
    // update it to the current belief
    bool found = false;
    Vector2f prior(v->belief.mu - bias);
    for(auto &e : v->edges)
    {
        if (e->f && e->f->edges.size() == 1)
        {
            found = true;
            e->f->z = prior;
            e->f->factor.lambda = cov.inverse();
            e->f->edges[0].var_to_factor_msg = v->belief;
            break;
        }
    }
    if (!found)
    {
        // auto f = new Pose_factor(uid(), timestep, {v}, prior, sqrt(cov(0, 0)));
        //printf("Didn't find anchor!!=========================================\n");
        auto f = new Pose_factor(uid(), timestep, {v}, prior, sqrt(cov(0, 0)));
        fac_nodes.push_back(f); 
        f->edges[0].var_to_factor_msg = v->belief;
        // flops += f->send_message(v->id, sim->sp.damping);
    }
}

// --------------------------------------------------------------------------------------------------
// Perform a single iteration on the factor graph
// --------------------------------------------------------------------------------------------------
float Factor_graph::do_iteration(int iterations)
{
    if (!fac_nodes.size())
        return 0;
    for(int i = 0; i < iterations; i++)
    {
        //---------------------------------------------------------------
        // Choose a random factor and send messages to all edges
        auto f = fac_nodes[randi(0, fac_nodes.size() - 1)];

        for(auto &e : f->edges)
        {

            // Pre message debug output
            if (sim->sp.debug_msgs)
            {
                printf("fac  %4d %s|%s\n", f->id, f->repr().c_str(), f->fr().c_str());
                if (sim->sp.fp)fprintf(sim->sp.fp, "fac  %4d %s|%s\n", f->id, f->repr().c_str(), f->fr().c_str());
                if (e.v)
                {
                    printf("vpre %4d %s\n", e.v->id, e.v->belief.repr().c_str());
                        if (sim->sp.fp)fprintf(sim->sp.fp, "vpre %4d %s\n", e.v->id, e.v->belief.repr().c_str());
                }
            }

            //---------------------------------------------------------------
            // Send message and if there is a connecting variable on the edge, get
            // corresponding variable belief
            if (e.v)
                flops_update += f->send_and_get_message(e.v->id, sim->sp.damping);
            else
                flops_update += f->send_message(0, sim->sp.damping);

            f_iterated = f;
            //---------------------------------------------------------------
            

            // Post message debug output
            if (sim->sp.debug_msgs)
            {
                printf("robot %d fac id:%5d to var id:%5d %s  |  %s\n", name, f->id, e.v ? e.v->id : 0, 
                    e.factor_to_var_msg.repr().c_str(), e.var_to_factor_msg.repr().c_str());
                if (sim->sp.fp)
                    fprintf(sim->sp.fp, "robot %d fac id:%5d to var id:%5d %s  |  %s\n", name, f->id, e.v ? e.v->id : 0, 
                        e.factor_to_var_msg.repr().c_str(), e.var_to_factor_msg.repr().c_str());
                if (e.v)
                {
                    printf("vpst %4d %s\n", e.v->id, e.v->belief.repr().c_str());
                    if (fabs(e.v->belief.mu(0)) < 0.001 && fabs(e.v->belief.mu(1)) < 0.001)
                    {
                        printf("=============================================>\n");
                    }
                    if (sim->sp.fp)
                    {
                        fprintf(sim->sp.fp, "vpst %4d %s\n", e.v->id, e.v->belief.repr().c_str());
                        if (fabs(e.v->belief.mu(0)) < 0.001 && fabs(e.v->belief.mu(1)) < 0.001)
                        {
                            fprintf(sim->sp.fp, "=============================================>\n");
                        }
                    }
                }
                printf("fac  %4d %s|%s\n", f->id, f->repr().c_str(), f->fr().c_str());
                if (sim->sp.fp)fprintf(sim->sp.fp, "fac  %4d %s|%s\n", f->id, f->repr().c_str(), f->fr().c_str());
            }
        }
    }
    return 0;
}


void Factor_graph::print()
{
    for(int v = 0; v < var_nodes.size(); v++)
    {
        var_nodes[v]->print();
        auto &es = var_nodes[v]->edges;
        for(auto &e : es)
        {
            if (e->f)
                e->f->print();
            else
                printf("    Ext  id %5d\n", e->robot_id);
        }
    }
    printf("Variables: %4d Factors: %4d\n", (int)var_nodes.size(), (int)fac_nodes.size());
}


ThickLine::ThickLine(b2Vec2 _s, b2Vec2 _e, b2Color _c, float _t)
:   start       (_s),
    end         (_e),
    colour      (_c),
    thickness   (_t) 
    {
        ticks = timer::ticks();
    }
ThickLine::ThickLine(Robot *_s, Robot *_e, b2Color _c, float _t)
:   rs          (_s),
    re          (_e),
    colour      (_c),
    thickness   (_t) 
    {
        ticks = timer::ticks();
    }
void ThickLine::render(DebugDraw *dd, Sim *sim)
{
    int age = timer::ticks() - ticks;
    float alpha = colour.a * (1.0 - ((float)age / sim->sp.decay_time));
    b2Color tlc(
        colour.r * sim->sp.lighten, 
        colour.g * sim->sp.lighten, 
        colour.b * sim->sp.lighten, 
        alpha);
    if (rs)
    {
        start = rs->body->GetPosition();
        end = re->body->GetPosition();
    }
    dd->DrawThickSegment(start, end, tlc, 0.05);
}



void Factor_graph::render(DebugDraw *dd, b2Color colour, Vector2f est_pos, Vector2f rgt)
{
    // Beliefs may have a bias compared to ground truth, but it is global and irrelevant to
    // the robot. We adjust here for the purpose of visualisation.
    // auto bias = -sim->system_bias;
    if (!var_nodes.size() || !var_nodes.back()->updated)
        return;

    auto bias = sim->sp.gui.rbias ? -me->bias : -sim->system_bias;

    if (sim->sp.gui.fg)
    {
        for(auto &v : var_nodes)
        {
            if (!v->updated)
                continue;

            auto bl = v->belief.mu + bias;

            MatrixXf cov = v->belief.get_cov();

            float r = sqrt(cov(0,0));
            if (r < 0.5 * sim->sp.sigma_anchor_est)
            {
                dd->DrawCircle(b2Vec2(bl(0), bl(1)), r, colour);

                for(auto &e : v->edges)
                {
                    if (e->f && e->f->edges.size() == 1)
                    {
                        float s = r * 0.707;
                        auto p0 = bl + Vector2f(-s, -s);
                        auto p1 = bl + Vector2f( s,  s);
                        auto p2 = bl + Vector2f( s, -s);
                        auto p3 = bl + Vector2f(-s,  s);
                        dd->DrawSegment(b2Vec2(p0(0), p0(1)), b2Vec2(p1(0), p1(1)), colour);
                        dd->DrawSegment(b2Vec2(p2(0), p2(1)), b2Vec2(p3(0), p3(1)), colour);
                        break;
                    }
                }
            }


        }

        if(sim->sp.gui.factor)for(auto &f : fac_nodes)
        {
            if (!f->updated)
                continue;
            if (f->edges.size() == 2)
            {
                // auto c = me->colour;
                auto c = b2Color(0.9, 0.9, 0.9);
                if (!f->edges[1].v && f->edges[0].v->updated)
                {
                    // Foreign variable is NULL, draw a line to the belief in the message
                    auto p0 = f->edges[0].v->belief.mu + bias;
                    auto p1 = f->edges[1].var_to_factor_msg.mu + bias;
                    if (f->edges[0].v->otype != Otype::BOX)
                        dd->DrawSegment(b2Vec2(p0(0), p0(1)), b2Vec2(p1(0), p1(1)), c);
                }
                else if (f->edges[0].v->updated && f->edges[1].v->updated)
                {
                    // if (f->edges[0].v->belief.mu == Vector2f::Zero())
                    // {
                    //     printf("zero belief %5d\n", f->edges[0].v->id);
                    // }
                    auto p0 = f->edges[0].v->belief.mu + bias;
                    auto p1 = f->edges[1].v->belief.mu + bias;


                    b2Color c = colour;
                    c.a = sqrt(f->factor.lambda(0,0)) * 0.01;
                    dd->DrawSegment(b2Vec2(p0(0), p0(1)), b2Vec2(p1(0), p1(1)), c);
                }
            }
        }
    }


    // -------------------------------------------------------
    // Animate message send to other robot. msg_start and end point to robots
    // at each end of message transfer. Null pointer means no message.
    if (sim->sp.gui.msg)
    {
        // b2Color c = b2Color(0.0, 0.9, 0.7);
        if (msg_start)
        {
            // dd->DrawThickSegment(msg_start->body->GetPosition(), msg_end->body->GetPosition(), c, 0.05);

            b2Color c = b2Color(
                (msg_start->colour.r + msg_end->colour.r) / 2,
                (msg_start->colour.g + msg_end->colour.g) / 2,
                (msg_start->colour.b + msg_end->colour.b) / 2,
                (msg_start->colour.a + msg_end->colour.a) / 2
            );

            if (sim->sp.gui.mono)c = b2Color(0,0,0,1);
            c.a *= sim->sp.alpha;
            tlines.push_back(ThickLine(msg_start, msg_end, c, 0.05));
            msg_start = 0;
            msg_end = 0;
        }

        if (sim->sp.gui.fg && f_iterated)
        {
            auto c = me->colour;
            if (sim->sp.gui.mono)c = b2Color(0,0,0,1);
            c.a *= sim->sp.alpha;
            auto &f = f_iterated;
            if (f->edges.size() == 2)
            {
                if (!f->edges[1].v && f->edges[0].v->updated)
                {
                    // Foreign variable is NULL, draw a line to the belief in the message
                    auto p0 = f->edges[0].v->belief.mu + bias;
                    auto p1 = f->edges[1].var_to_factor_msg.mu + bias;
                    if (f->edges[0].v->otype != Otype::BOX)
                    {
                        // dd->DrawThickSegment(b2Vec2(p0(0), p0(1)), b2Vec2(p1(0), p1(1)), c, 0.05);
                        tlines.push_back(ThickLine(b2Vec2(p0(0), p0(1)), b2Vec2(p1(0), p1(1)), c, 0.05));
                    }
                }
                else if ( f->edges[0].v->updated && f->edges[1].v->updated)
                {
                    auto p0 = f->edges[0].v->belief.mu + bias;
                    auto p1 = f->edges[1].v->belief.mu + bias;

                    // dd->DrawThickSegment(b2Vec2(p0(0), p0(1)), b2Vec2(p1(0), p1(1)), c, 0.05);
                    tlines.push_back(ThickLine(b2Vec2(p0(0), p0(1)), b2Vec2(p1(0), p1(1)), c, 0.05));
                }
            }
            f_iterated = 0;
        }
        // Decay time is the number of sim ticks a drawn line takes to disappear
        // int decay_time = 15;
        // Remove old lines
        auto &decay_time = sim->sp.decay_time;
        tlines.erase(
            remove_if(
                tlines.begin(), 
                tlines.end(), 
                [&decay_time](ThickLine &a){return timer::ticks() - a.ticks > decay_time;}
            ), 
            tlines.end());
        // Draw the remaining lines with increasing alpha based on age
        for(auto &l : tlines)
        {
            l.render(dd, sim);


            // int age = timer::ticks() - l.ticks;
            // float alpha = l.colour.a * (1.0 - ((float)age / sim->sp.decay_time));
            // b2Color tlc(
            //     l.colour.r * sim->sp.lighten, 
            //     l.colour.g * sim->sp.lighten, 
            //     l.colour.b * sim->sp.lighten, 
            //     alpha);
            // // printf("%d %d %f %f %f %f\n", timer::ticks(), l.ticks, tlc.r,tlc.g,tlc.b,tlc.a);
            // dd->DrawThickSegment(l.start, l.end, tlc, 0.05);
        }

    }


}
            l.render(dd, sim);


            // int age = timer::ticks() - l.ticks;
            // float alpha = l.colour.a * (1.0 - ((float)age / sim->sp.decay_time));
            // b2Color tlc(
            //     l.colour.r * sim->sp.lighten, 
            //     l.colour.g * sim->sp.lighten, 
            //     l.colour.b * sim->sp.lighten, 
            //     alpha);
            // // printf("%d %d %f %f %f %f\n", timer::ticks(), l.ticks, tlc.r,tlc.g,tlc.b,tlc.a);
            // dd->DrawThickSegment(l.start, l.end, tlc, 0.05);
        }

    }


}