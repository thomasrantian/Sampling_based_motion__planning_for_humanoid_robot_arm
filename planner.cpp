#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <openrave/planningutils.h>
#include <vector>
#include "RRT.h"
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>
#include <stdlib.h> 
#include <time.h>  
#include<vector>
using namespace OpenRAVE;
using namespace std;


// Inherit from the modulebase class in openrave
class planner : public ModuleBase{
public:
    // start and goal config
    vector<double> start_node_config_;
    vector<double> goal_node_config_;
    // joints limits
    vector<double> joints_limits_lower_;
    vector<double> joints_limits_upper_;

    EnvironmentBasePtr env; // ptr to the openrave environment
    RobotBasePtr pr2;       // ptr to the PR2 robot

    double goal_bias_;
    double step_size;

    // build the search tree
    NodeTree searchtree;

    bool goal_found;
    RRTNode* goal_ptr   = NULL;
    RRTNode* goal_ptr_s = NULL;
    RRTNode* goal_ptr_g = NULL;
    
    deque<RRTNode*> path_; // with the correct order of the node

    int nodes_sampled;
    vector<double> smooth_hist;
    vector<double> RRT_star_path_hist; // path length history for RRT star



public:
    planner(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("setInitialConfig",boost::bind(&planner::setInitialConfig,this,_1,_2),
                        "Setting the initial configuration of the robot.");
        RegisterCommand("setGoalConfig",boost::bind(&planner::setGoalConfig,this,_1,_2),
                        "Setting the goal configuration of the robot.");
        RegisterCommand("planner_config",boost::bind(&planner::planner_config,this,_1,_2),
                        "Setting the configuration of the planner.");
        RegisterCommand("plan_path",boost::bind(&planner::plan_path,this,_1,_2),
                        "Plan the path using Navive RRT.");
        RegisterCommand("plan_path_bi",boost::bind(&planner::plan_path_bi,this,_1,_2),
                        "Plan the path using the Bi-direction RRT.");
        RegisterCommand("smooth_path",boost::bind(&planner::smooth_path,this,_1,_2),
                        "Smooth the path using short cutting.");
        RegisterCommand("plan_path_rrtstar",boost::bind(&planner::plan_path_rrtstar,this,_1,_2),
                        "Plan the path using the RRT star.");
        RegisterCommand("count_nodes",boost::bind(&planner::count_nodes,this,_1,_2),
                        "Send the sampled nodes.");
        RegisterCommand("smooth_hist_print",boost::bind(&planner::smooth_hist_print,this,_1,_2),
                        "Sent the smoothing history.");
        RegisterCommand("RRT_star_hist_print",boost::bind(&planner::RRT_star_hist_print,this,_1,_2),
                        "Sent the RRT star smoothing history.");
        RegisterCommand("return_path",boost::bind(&planner::return_path,this,_1,_2),
                        "Return path");
        goal_found = false;
        nodes_sampled = 0;
    }

    virtual ~planner(){}

    // return the path based on the index
    bool return_path(ostream& sout,istream& sinput){
        int index_1;
        int index_2;
        sinput >> index_1;
        sinput >> index_2;
        if (index_2 > path_.size()){
            return false;
        }
        for (int i = index_1; i < index_2; ++i){
            for (int j = 0; j< 7; ++j){
                sout << (path_[i]->q_)[j] << ",";
            }
        }
    }

    bool count_nodes(ostream& sout,istream& sinput){
        sout << nodes_sampled;
        return true;
    }
    bool smooth_hist_print(ostream& sout,istream& sinput){
        for (int i = 0; i < (int) smooth_hist.size(); ++i){
            sout << smooth_hist[i] << ",";
        }
        //cout << smooth_hist.size();
        return true;
    }

    bool RRT_star_hist_print(ostream& sout,istream& sinput){
        for (int i = 0; i < (int) RRT_star_path_hist.size(); ++i){
            sout << RRT_star_path_hist[i] << ",";
        }
        //cout << smooth_hist.size();
        return true;
    }

    // Set the initial config
    bool setInitialConfig(ostream& sout,istream& sinput){
        string string_holder;
        sinput >> string_holder;
        if (string_holder == "start"){
            for (int i = 0; i < 7; ++i) {
                double temp_amgle;
                sinput >> temp_amgle;
                start_node_config_.push_back(temp_amgle);
            }
            return true;
        }else{
            sout << "Invalid start configuration input";
            return false;
        }
    }


    // Set the goal config
    bool setGoalConfig(ostream& sout,istream& sinput){
        string string_holder;
        sinput >> string_holder;
        if (string_holder == "goal"){
            // start to read the 7 configs
            for (int i = 0; i < 7; ++i) {
                double temp_amgle;
                sinput >> temp_amgle;
                goal_node_config_.push_back(temp_amgle);
            }
            return true;

        }else{
            sout << "Invalid goal configuration input";
            return false;
        }
    }

    // set the planning parameters: joints limit

    bool planner_config(ostream& sout,istream& sinput){
        // set the goal bias
        string string_holder;
        sinput >> string_holder;
        if (string_holder == "params"){
            sinput >> goal_bias_;
            sinput >> step_size;
        }else{
            cout << "Invalid params"<< endl;
            return false;
        }
        // set the joints limits
        env = GetEnv();
        pr2 = env -> GetRobot("PR2");

        pr2->GetActiveDOFLimits(joints_limits_lower_, joints_limits_upper_);
        joints_limits_lower_[4] = -M_PI;
        joints_limits_lower_[6] = -M_PI;
        joints_limits_upper_[4] = M_PI;
        joints_limits_upper_[6] = M_PI;
        return true;

    }


    // plan Birrt path
    bool plan_path_bi(ostream& sout,istream& sinput){
        //srand((int)time(0));
        srand(5);
        RRTNode* start_node_ptr = new RRTNode(start_node_config_);
        RRTNode* goal_node_ptr = new RRTNode(goal_node_config_);
        searchtree.add_node_to_tree(1, start_node_ptr);
        searchtree.add_node_to_tree(2,goal_node_ptr);
        int temp = 1;
        while (!goal_found){
            vector<double> random_config = generate_node_config();
            nodes_sampled++;
            if (temp % 2 == 0){// grow the start tree
                connect_tree(1, random_config);
                connect_tree(2, searchtree.tree_s_nodes_[searchtree.tree_s_nodes_.size()-1] -> q_);
            }else{// grow the goal tree
                connect_tree(2, random_config);
                connect_tree(1, searchtree.tree_g_nodes_[searchtree.tree_g_nodes_.size()-1] -> q_);
            }
            // check if two trees can be connected using L1 distance
            if (get_L1_distance(searchtree.tree_s_nodes_[searchtree.tree_s_nodes_.size()-1]->q_ ,searchtree.tree_g_nodes_[searchtree.tree_g_nodes_.size()-1]->q_) < 0.1){
                goal_found = true;
                goal_ptr_s =searchtree.tree_s_nodes_[searchtree.tree_s_nodes_.size()-1];
                goal_ptr_g =searchtree.tree_g_nodes_[searchtree.tree_g_nodes_.size()-1];
            }
            temp ++;
        }
        cout << "Goal found! Constructing path..." << endl;        
        RRTNode* ptr = goal_ptr_s;
        while (ptr != NULL){
            path_.push_front(ptr);
            ptr = ptr -> parent_;
        }
        ptr = goal_ptr_g;
        while (ptr != NULL){
            path_.push_back(ptr);
            ptr = ptr -> parent_;
        }
        for (int i = 0; i < (int) path_.size(); ++i){
            for (int j = 0; j< 7; ++j){
                sout << (path_[i]->q_)[j] << ",";
            }
        }
        return true;

    }

    // plan the RRT_star path
    bool plan_path_rrtstar(ostream& sout,istream& sinput){
        srand(5);
        // add the start node  into the tree
        RRTNode* start_node_ptr = new RRTNode(start_node_config_);
        searchtree.add_node(start_node_ptr);
        
        // set the start time
        time_t start = time(0); 
        int marktime = 100;
        while (true){
            // check the running time
            time_t now = time(0); 
            double dt = (double) now - start;
            //cout << dt << endl;
            if (dt  > 15*60){
                cout << "Searching complete! Time used: 900 s. "<<endl;
                break;
            }
            

            // generate a random node
            vector<double> random_config = generate_node_config();
            nodes_sampled = nodes_sampled + 1;

            // extend one step
            RRTNode* new_node = extend(random_config);
            if (new_node == NULL){
                continue;
            }

            // get the nearest neighbors
            vector<RRTNode*> neighbors;
            RRTNode* nn = searchtree.get_nearest(0,new_node->q_);
            double R = 2 * step_size;
            for (int i = 0; i < (int) searchtree.nodes_.size(); ++i){
                if (searchtree.get_weighted_distance(searchtree.nodes_[i]->q_, new_node->q_) <= R){
                    if (is_connectable(searchtree.nodes_[i], new_node)){
                        neighbors.push_back(searchtree.nodes_[i]);
                    }
                } 
            }

            if (neighbors.size() == 0){
                double d = searchtree.get_weighted_distance(new_node->q_, nn->q_);
                cout << "d between nn and nn:" << d<< endl;
            }

            // select the parent from the neighbors
            select_parent(neighbors, new_node);
            // re-wire the edges in the neighborhood
            rewire(neighbors, new_node);
            // record the current path length
            // record the best path length every 1 mins
            //cout << (int) dt << endl;
            // if ((int)dt == marktime){
            //     marktime = marktime + 100;
            //     double best_cost = 1000;
            //     for (int i = 0; i < (int) searchtree.nodes_.size(); ++ i){
            //         if (isgoalreached(searchtree.nodes_[i])){
            //             double temp_cost = get_final_cost(searchtree.nodes_[i]);
            //             if (temp_cost < best_cost){
            //                 best_cost = temp_cost;
            //                 cout << best_cost << endl;
            //             }
            //         }
            //     }
                
            //     RRT_star_path_hist.push_back(best_cost);
            // }
            

        }
        //cout << "Constructing path..." << endl;
        double best_cost_t = 1000;
        for (int i = 0; i < (int) searchtree.nodes_.size(); ++ i){
            if (isgoalreached(searchtree.nodes_[i])){
                double temp_cost = get_final_cost(searchtree.nodes_[i]);
                if (temp_cost < best_cost_t){
                    best_cost_t = temp_cost;
                    goal_ptr = searchtree.nodes_[i];
                }
            }
        }
        // build the path
        RRTNode* ptr = goal_ptr;
        while (ptr != NULL){
            path_.push_front(ptr);
            ptr = ptr -> parent_;
        }
        for (int i = 0; i < (int) path_.size(); ++i){
            for (int j = 0; j< 7; ++j){
                sout << (path_[i]->q_)[j] << ",";
            }
        }
        return true;
    }

    // update children nodes
    // the parent's g is updated
    void update_children(RRTNode* parent_in){
        // base case
        if (parent_in == NULL) return;
        for (int i = 0; i < (parent_in->childs_).size(); ++i){
            (parent_in->childs_)[0]->g_ = parent_in -> g_ + searchtree.get_weighted_distance((parent_in->childs_)[0]->q_, parent_in->q_);
            update_children((parent_in->childs_)[0]);
        }

    }

    // re-wire the edges in the neighbor
    void rewire(vector<RRTNode*>& neighbors, RRTNode* new_node){
        // check if the cost can be reduced if go from new_node to other nodes in the neighbor
        for (int i = 0; i < (int) neighbors.size(); ++i){
            if (neighbors[i] == new_node->parent_) continue;
            double new_g = searchtree.get_weighted_distance(new_node->q_, neighbors[i]->q_) + new_node->g_;
            if (new_g < neighbors[i]->g_){
                neighbors[i]->parent_ = new_node;
                new_node->childs_.push_back(neighbors[i]);
                neighbors[i] -> g_ = new_g;
                //update_children(neighbors[i]);
            }
        }
    }


    // steer phase in the paper, return the new node
    void select_parent(vector<RRTNode*>& neighbors, RRTNode* new_node){
        if (neighbors.size() == 0) {
            cout << "No neighbors around me!" << endl;
        }
        // loop every node in the parent, check which 
        double min_g = numeric_limits<double>::max();
        RRTNode* best_parent = NULL;
        double best_delta_g;
        //double best_g = 0;
        for (int i = 0; i < (int) neighbors.size(); ++i){
            // Do I need to check the collision here?

            double delta_g = searchtree.get_weighted_distance(neighbors[i]->q_, new_node->q_);
            //double current_g = get_final_cost(neighbors[i]);
            if (delta_g + neighbors[i]->g_ < min_g){
                min_g = delta_g + neighbors[i]->g_;
                best_parent = neighbors[i];
                best_delta_g = delta_g;
            }
        }

        // check if the parent is too far away from the q_new
        /*if (best_delta_g > step_size){ // I need to linear interpolate to connect
            int N = best_delta_g / step_size;
            // temp ptr used to set parent
            RRTNode* current_node_ptr = best_parent;
            // start to increment
            for (int n = 1; n <= N; ++n){
                vector<double> new_node_congig_temp;
                for (int j = 0; j < 7; ++ j){
                    double increment = (double) ((new_node->q_)[j] - (best_parent->q_)[j]) / N * n;
                    new_node_congig_temp.push_back((best_parent->q_)[j] + increment);
                } 
                // check collision
                //if (!check_collision(new_node_congig_temp)){
                    // build the node
                    RRTNode* new_node = new RRTNode(new_node_congig_temp);
                    new_node -> parent_ = current_node_ptr;
                    new_node->g_ = current_node_ptr->g_ + searchtree.get_weighted_distance(current_node_ptr->q_, new_node->q_);
                    current_node_ptr -> childs_.push_back(new_node);
                    searchtree.add_node( new_node);
                    current_node_ptr = new_node;

                //}else{
                    // collision, stop extend
                //    return;
                //}
            }

        }else{*/
            new_node -> parent_ = best_parent;
            best_parent -> childs_.push_back(new_node);
            new_node -> g_ = min_g;
            searchtree.add_node(new_node);
       // }
        return;
    }

    // back tracking to get the final cost
    double get_final_cost(RRTNode* ptr){
        double  cost = 0;
        //cout << "start" << endl;
        while (ptr->parent_ != NULL){
            cost = cost + get_L1_distance(ptr->q_, (ptr->parent_)->q_);
            ptr = ptr->parent_;
        }
        return cost;
        //cout << "end" << endl;
    }

    // function to check if the two node can be connected without collision
    bool is_connectable(RRTNode* node_1, RRTNode* node_2){
       
        double distance = searchtree.get_weighted_distance(node_1->q_, node_2->q_); 
        int N = distance / step_size;
        if (distance > step_size){
            // start to increment
            for (int n = 1; n <= N; ++n){
                vector<double> new_node_congig_temp;
                for (int j = 0; j < 7; ++ j){
                    double increment = (double) ((node_2->q_)[j] - (node_1->q_)[j]) / N * n;
                    new_node_congig_temp.push_back((node_1->q_)[j] + increment);
                } 
                // check collision
                if (check_collision(new_node_congig_temp)){
                    return false;
                }
            }
        }
        return true;
    }


    // plan the path
    bool plan_path(ostream& sout,istream& sinput){
        
        //srand((int)time(0));
        srand(5);
        // add the start node  into the tree
        RRTNode* start_node_ptr = new RRTNode(start_node_config_);
        searchtree.add_node(start_node_ptr);
        while (!goal_found){
            vector<double> random_config = generate_node_config();
            nodes_sampled++;
            connect(random_config);
        }
        cout << "Goal found! Constructing path..." << endl;
        // build the path
        RRTNode* ptr = goal_ptr;
        while (ptr != NULL){
            path_.push_front(ptr);
            ptr = ptr -> parent_;
        }
        for (int i = 0; i < (int) path_.size(); ++i){
            for (int j = 0; j< 7; ++j){
                sout << (path_[i]->q_)[j] << ",";
            }
        }
        return true;
    }

    bool smooth_path(ostream& sout,istream& sinput){
        double d_l = 0.1;
        for (int itr = 0; itr < 200; ++itr){
            int n_1_index = 0;
            int n_2_index = 0;
            while (n_2_index - n_1_index < 2){
                n_1_index = (int) ((double) rand()/RAND_MAX * path_.size());
                n_2_index = (int) ((double) rand()/RAND_MAX * path_.size());
            }// ensure n_2 is larger

            RRTNode* n_1 = path_[n_1_index];
            RRTNode* n_2 = path_[n_2_index];
            
            double l = get_L1_distance(n_1->q_, n_2->q_);
            
            int N = l / d_l;

            bool cutting_status = true;

            vector< vector<double> > temp_path_holder;
            for (int n = 1; n <= N; ++n){
                vector<double> new_node_congig_temp;
                for (int j = 0; j < 7; ++ j){
                    double increment = (double) ((n_2->q_)[j] - (n_1->q_)[j]) / N * n;
                    new_node_congig_temp.push_back((n_1->q_)[j] + increment);
                }
                if (check_collision(new_node_congig_temp)){
                    cutting_status = false;
                    break;   
                }
                temp_path_holder.push_back(new_node_congig_temp);
            }

            if (cutting_status){ // replace the path in between with the 
                // reset the path, delete the path from (n_1 n_2], 
                deque<RRTNode*> new_path;
                for (int i = 0; i <= n_1_index; ++ i){
                    new_path.push_back(path_[i]);
                }
                for (int i = 0 ; i < (int) temp_path_holder.size();++i){
                    RRTNode* new_node = new RRTNode(temp_path_holder[i]);
                    new_path.push_back(new_node);
                }
                for (int i = n_2_index+1; i < (int) path_.size();++i){
                    new_path.push_back(path_[i]);
                }
                // delete the un-used pointer
                for (int i = n_1_index+1; i < n_2_index+1; ++i){
                    delete path_[i];
                }
                path_ = new_path;
            }
            // record the cost
            double cost = 0;
            for (int i = 0; i < path_.size()-1; ++ i){
                cost = cost + get_L1_distance(path_[i]->q_, path_[i+1]->q_);
            }
            smooth_hist.push_back(cost);
        }
        for (int i = 0; i < (int) path_.size(); ++i){
            for (int j = 0; j< 7; ++j){
                sout << (path_[i]->q_)[j] << ",";
            }
        }
        return true;

    }

    // calculate the L1 distance between the node
    double get_L1_distance(vector<double>& n_1_c, vector<double>& n_2_c ){
        double distance = 0;
        for (int i = 0; i < 7; ++i){
            distance = distance + abs(n_1_c[i] - n_2_c[i]) ;
        }
        return distance;
    }

    // connect the random node with the start tree, stop if collision
    void connect_tree(int flag, vector<double>& random_config){
        // find the nearest neighbor

        RRTNode* my_neighbor = searchtree.get_nearest(flag,random_config);

        // L_1 distance between the two config
        double distance = searchtree.get_weighted_distance(my_neighbor->q_, random_config);
        
        int N = distance / step_size;
        
        // temp ptr used to set parent
        
        RRTNode* current_node_ptr = my_neighbor;
       
        if (distance > step_size){
            // start to increment
            for (int n = 1; n <= N; ++n){
                vector<double> new_node_congig_temp;
                for (int j = 0; j < 7; ++ j){
                    double increment = (double) (random_config[j] - (my_neighbor->q_)[j]) / N * n;
                    new_node_congig_temp.push_back((my_neighbor->q_)[j] + increment);
                } 
                // check collision
                if (!check_collision(new_node_congig_temp)){
                    // build the node
                    RRTNode* new_node = new RRTNode(new_node_congig_temp);
                    new_node -> parent_ = current_node_ptr;
                    searchtree.add_node_to_tree(flag, new_node);
                    current_node_ptr = new_node;
                }else{
                    // collision, stop extend
                    return;
                }

            }
        }

    }

    // extend the tree to the random config location for one step, if collision, return NULL
    RRTNode* extend(vector<double>& random_config){
        // generate a new node based on the neighbor, and the random config, using

        // find the nearest neighbor
        RRTNode* my_neighbor = searchtree.get_nearest(0,random_config);
        if (my_neighbor == NULL) {
            return NULL;
        }

        // distance between the neighbor and the random config
        double distance = searchtree.get_weighted_distance(my_neighbor->q_, random_config);
        int N = distance / step_size + 1;
       
        // if the new generated neighbor is alread 
        if (distance > step_size){
            // start to increment, only one step
            for (int n = 1; n <= 1; ++n){
                vector<double> new_node_congig_temp;
                for (int j = 0; j < 7; ++ j){
                    double increment = (double) (random_config[j] - (my_neighbor->q_)[j]) / N * n;
                    new_node_congig_temp.push_back((my_neighbor->q_)[j] + increment);
                } 
                // check collision
                if (!check_collision(new_node_congig_temp)){
                    // build the node
                    RRTNode* new_node = new RRTNode(new_node_congig_temp);
                    return new_node;
                }
                
                    // collision, stop extend
                return NULL;
            }
        }else{ // with in one step size, just use the random node as the new node
            // check collision
            if (!check_collision(random_config)){
                // build the node
                RRTNode* new_node = new RRTNode(random_config);
                return new_node;
            }
            return NULL;
        }
        return NULL;

    }

    // connect the random node with the rrt tree 
    void  connect( vector<double>& random_config){
        // generate a new node based on the neighbor, and the random config, using

        // find the nearest neighbor
        RRTNode* my_neighbor = searchtree.get_nearest(0,random_config);
        // distance between the neighbor and the random config
        double distance = searchtree.get_weighted_distance(my_neighbor->q_, random_config);
        int N = distance / step_size;
        // temp ptr used to set parent
        RRTNode* current_node_ptr = my_neighbor;
        
        // if the new generated neighbor is alread 
        if (distance > step_size){
            // start to increment
            for (int n = 1; n <= N; ++n){
                vector<double> new_node_congig_temp;
                for (int j = 0; j < 7; ++ j){
                    double increment = (double) (random_config[j] - (my_neighbor->q_)[j]) / N * n;
                    new_node_congig_temp.push_back((my_neighbor->q_)[j] + increment);
                } 
                // check collision
                if (!check_collision(new_node_congig_temp)){
                    // build the node
                    RRTNode* new_node = new RRTNode(new_node_congig_temp);
                    new_node -> parent_ = current_node_ptr;
                    searchtree.add_node(new_node);
                    current_node_ptr = new_node;
                }else{
                    // collision, stop extend
                    return;
                }
            }
        }

        // if the new node is already within one step distance or I reached the random node, check goal
        if (isgoalreached(current_node_ptr)){
            RRTNode* new_node = new RRTNode(goal_node_config_);
            new_node -> parent_ = current_node_ptr;
            searchtree.add_node(new_node);
            goal_found = true;
            goal_ptr = new_node;
            return;
        }
    }

    // check if goal reached
    bool isgoalreached(RRTNode* node){
        
        double distance = get_L1_distance(node->q_, goal_node_config_);
        //cout << distance << endl;
        return distance <= 0.1;
    }

    bool check_collision(vector<double> temp_config){
        // set the PR2 to that config


        pr2->SetActiveDOFValues(temp_config);
        
        return (GetEnv()->CheckCollision(pr2)||pr2->CheckSelfCollision());
    }


    vector<double> generate_node_config(){
        // generate the joint angle from -pi to pi

        double temp = (double) rand()/RAND_MAX; // random number between 0-1
        if (temp < goal_bias_){
            // return goal config as the new random config
            return goal_node_config_;

        }else{
            vector<double> new_config;
            for (int i = 0; i < 7; ++i){
                temp = (double) rand()/RAND_MAX;
                double angle_range = joints_limits_upper_[i] - joints_limits_lower_[i];
                double temp_angle = (   temp * angle_range + joints_limits_lower_[i]);
                new_config.push_back(temp_angle);
            }
            return new_config;
        }
    }



};


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "planner" ) {
        return InterfaceBasePtr(new planner(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("planner");
    
}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}

