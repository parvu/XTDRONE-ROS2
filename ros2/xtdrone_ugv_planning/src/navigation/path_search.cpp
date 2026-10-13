#include "hybrid_astar_test/navigation/path_search.hpp"

namespace HybridAStar {
HybridAStarGraphSearcher::HybridAStarGraphSearcher(){

    if (Constants::dubinsLookup) {
        Lookup::dubinsLookup(dubinsLookup);
    }

    Lookup::collisionLookup(collisionLookup);

    costmap_subscriber_ptr_ = std::make_shared<CostmapSubscriber>(
        xtd::node()->declare_parameter<std::string>("map_topic", "/map"), Constants::cellSize,
        static_cast<float>(xtd::node()->declare_parameter<double>("map_margin", 20.0)),
        xtd::node()->declare_parameter<bool>("unknown_occupied", true),
        static_cast<float>(xtd::node()->declare_parameter<double>("unknown_free_radius", 5.0)),
        static_cast<float>(xtd::node()->declare_parameter<double>("unknown_shadow_depth", 3.0)));
    if(HybridAStar::Constants::manual)
        est_start_subscriber_ptr_ = std::make_shared<EstStartSubscriber>("/initialpose");
    else
        start_subscriber_ptr_   = std::make_shared<StartSubscriber>(
            xtd::node()->declare_parameter<std::string>("base_frame", "ugv_0/base_link"), "map");
    goal_subscriber_ptr_ = std::make_shared<GoalSubscriber>(
        xtd::node()->declare_parameter<std::string>("goal_topic", "/goal_pose"));
    path_publisher_ptr_  = std::make_shared<Path>();
    front_end_path_publisher_ptr_ = std::make_shared<PathPublisher>("/front_end");
    replan_ = xtd::node()->declare_parameter<bool>("replan", false);
    smoother_ptr_        = std::make_shared<Smoother>();
    tf_broadcaster_ptr_  = std::make_shared<TfBroadcaster>("path", "map");
    start_point_publisher_ptr_ = std::make_shared<PointsPublisher>("start");
    goal_point_publisher_ptr_  = std::make_shared<PointsPublisher>("goal");

    grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();

}

HybridAStarGraphSearcher::~HybridAStarGraphSearcher(){

}

bool HybridAStarGraphSearcher::Run(){
    // replan:=false (default): one plan per goal, then wait for the next goal. XTDrone's ROS 1 node replanned from
    // the moving vehicle every 10 s (replan:=true); with slam_toolbox's growing map those plans often ran
    // backwards and the follower, restarting on each, wandered off.
    if (!replan_ && planned_) {
        if (!goal_subscriber_ptr_->HasGoal())
            return false;
        planned_ = false;
    }


    
    nav_msgs::msg::OccupancyGrid map;
    Eigen::Vector3f vehicle(NAN, NAN, 0.f);
    if (start_subscriber_ptr_ && !start_subscriber_ptr_->GetStart(vehicle))
        vehicle = Eigen::Vector3f(NAN, NAN, 0.f);
    if(!costmap_subscriber_ptr_->GetCostMap(map, x_origin_, y_origin_, vehicle.x(), vehicle.y()))
        return false;
    *grid = map;
    if(grid->info.resolution != Constants::cellSize)
        return false;

    configurationSpace.updateGrid(grid);
    int height = grid->info.height;
    int width  = grid->info.width;

    // The Voronoi diagram keeps this grid (DynamicVoronoi::initializeMap stores the pointer), and the start check
    // below reads it: it lives until the next map. The ROS 1 code freed it right after initializeMap.
    bool** binMap = new bool*[width];
    for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            binMap[x][y] = grid->data[y * width + x] ? true : false;
            int cost = grid->data[y * width + x];
            if (cost > 50) {
                binMap[x][y] = true;
            }
            else if(cost <=50){
                binMap[x][y] = false;
                continue;
            }
        }
    }
    voronoiDiagram.initializeMap(width, height, binMap);
    voronoiDiagram.update();
    // voronoiDiagram.visualize() wrote the diagram to result.ppm in the working directory at 5 Hz (a debug image)
    if (binMap_ != nullptr) {
        for (int x = 0; x < binMapWidth_; x++) { delete[] binMap_[x]; }
        delete[] binMap_;
    }
    binMap_ = binMap;
    binMapWidth_ = width;
    
    static Eigen::Vector3f start_point;
    static bool get_start_point = false;
    if(!get_start_point || !HybridAStar::Constants::manual) {
        static bool start_error_flag = true;
        if(HybridAStar::Constants::manual) {
            if (!est_start_subscriber_ptr_->GetStart(start_point)) {
                if (start_error_flag) {
                    start_error_flag = false;
                    std::cout << "\e[1;39;49m>>> please set the start\e[0m" << std::endl;
                }
                return false;
            } else
                get_start_point = true;
        }
        else{
            if (!start_subscriber_ptr_->GetStart(start_point)) {
                if (start_error_flag) {
                    start_error_flag = false;
//                    std::cout << "\e[1;39;49m>>> please set the starte[0m" << std::endl;
                }
                return false;
            } else
                get_start_point = true;
        }
        if(get_start_point) {
            std::cout << "\e[1;39;49m>>> got the start！！！\e[0m" << std::endl;
            start_point -= Eigen::Vector3f(x_origin_, y_origin_, 0);
        }
        start_error_flag = true;
    }
    float start_x = start_point.x() / Constants::cellSize;
    float start_y = start_point.y() / Constants::cellSize;
    float start_t = start_point.z();
    if ( !(grid->info.height >= start_y && start_y >= 0 && grid->info.width >= start_x && start_x >= 0 &&
            binMap[int(start_x)][int(start_y)] == false) ) {
        get_start_point = false;
        std::cout << "\e[1;31;49m>>> false start\e[0m" << std::endl;
        return false;
    }

    
    static Eigen::Vector3f goal_point = Eigen::Vector3f::Zero();
    static bool have_goal = false;
    if(!re_plan_flag) {
        static bool goal_error_flag = true;
        bool get_goal_flag;
        
        if (!(get_goal_flag = goal_subscriber_ptr_->GetGoal(goal_point)) || !arrived_flag) {
            if(get_goal_flag)
                have_goal = true;
            if(have_goal) {
                
                if (arrived_flag && !get_goal_flag) {
                    if (goal_error_flag) {
                        goal_error_flag = false;
                        std::cout << "\e[1;39;49m>>> please set the new goal\e[0m" << std::endl;
                        std::cout << "\e[1;38;49m************************************************ \e[0m" << std::endl;
                    }
                    arrived_flag = false;
                    return false;
                }
                
                Eigen::Vector3f judge_start_point = start_point + Eigen::Vector3f(x_origin_, y_origin_, 0);
                Eigen::Vector3f judge_goal_point  = goal_point  + Eigen::Vector3f(x_origin_, y_origin_, 0);
                if (!get_goal_flag && !arrived_flag) {
                    if ((judge_start_point - judge_goal_point).block<2, 1>(0, 0).norm() > 1.0)
                        std::cout << "\e[1;33;49m>>> continue\e[0m" << std::endl;
                    else {
                        if (goal_error_flag) {
                            goal_error_flag = false;
                            std::cout << "\e[1;39;49m>>> please set the goal\e[0m" << std::endl;
                        }
                        arrived_flag = true;
                        return false;
                    }
                }
            }
            else{
                if (goal_error_flag) {
                    goal_error_flag = false;
                    std::cout << "\e[1;39;49m>>> please set the goal\e[0m" << std::endl;
                }
                return false;
            }
            
        }
        if(get_goal_flag) {
            goal_world_ = goal_point;
            have_goal = true;
        }
        // the goal is kept in the map frame: the origin of slam_toolbox's map moves while the map grows
        goal_point = goal_world_ - Eigen::Vector3f(x_origin_, y_origin_, 0);
        goal_error_flag = true;
    }
    else
        goal_point = goal_point_;
    float goal_x = goal_point.x() / Constants::cellSize;
    float goal_y = goal_point.y() / Constants::cellSize;
    float goal_t = goal_point.z();
    if ( !(grid->info.height >= goal_y && goal_y >= 0 && grid->info.width >= goal_x && goal_x >= 0) ) {
        std::cout << "\e[1;31;49m>>> false goal\e[0m" << std::endl;
        return false;
    }
    std::cout << "\e[1;34;49m>>> start_point : \e[1;33;49m[" << start_point.x()  << "\t" <<
                                                                start_point.y() << "\t" <<
                                                                start_point.z() <<"]\e[0m" << std::endl;
    std::cout << "\e[1;34;49m>>> goal_point  : \e[1;33;49m[" << goal_point.x()   << "\t" <<
                                                                goal_point.y()  << "\t" <<
                                                                goal_point.z() <<"]\e[0m" << std::endl;
    start_point_publisher_ptr_->Publish(start_point + Eigen::Vector3f(x_origin_, y_origin_, 0));
    goal_point_publisher_ptr_ ->Publish(goal_point  + Eigen::Vector3f(x_origin_, y_origin_, 0));

    
    Eigen::Vector3f judge_start_point = start_point + Eigen::Vector3f(x_origin_, y_origin_, 0);
    Eigen::Vector3f judge_goal_point  = goal_point  + Eigen::Vector3f(x_origin_, y_origin_, 0);
    RCLCPP_WARN(xtd::node()->get_logger(), "Bug 2");
    if((judge_start_point - judge_goal_point).block<2,1>(0,0).norm() < 2.0) {
        start_point = goal_point;
        std::cout << "\e[1;32;49m>>> reached the goal\e[0m" << std::endl;
        return true;
    }
    if(!Plan(start_point, goal_point)) {
        std::cout << "\e[1;31;49m>>> failed\e[0m" << std::endl;
        return false;
    }
//    std::cout << vis_smooth_path.front().transpose() << ", start_point : " << start_point.transpose() << std::endl;
//    std::cout << vis_smooth_path.back().transpose() << ", goal_point : " << goal_point.transpose() << std::endl;

    if(Constants::manual)
        start_point = vis_smooth_path.back() - Eigen::Vector3f(x_origin_, y_origin_, 0);
    judge_start_point = start_point + Eigen::Vector3f(x_origin_, y_origin_, 0);
    
    if((judge_start_point - judge_goal_point).block<2,1>(0,0).norm() < 2.0) {
        start_point = goal_point;
        arrived_flag = true;
        if(vis_smooth_path.back() != judge_goal_point){
            vis_smooth_path.push_back(judge_goal_point);
        }
        std::cout << "\e[1;34;49m>>> Get Goal : " << vis_smooth_path.back().transpose() << "]\e[0m" << std::endl;
        PublishPath();
        planned_ = true;
        std::cout << "\e[1;32;49m>>> reached the goal\e[0m" << std::endl;
        return true;
    }
    else {
        // one plan per goal (replan:=false): the path ends at the goal, as the "reached" branch above does it
        // (only when the search got there: a path that stops short would get a jump to the goal)
        if(!replan_ && !vis_smooth_path.empty() && vis_smooth_path.back() != judge_goal_point &&
           (vis_smooth_path.back() - judge_goal_point).block<2,1>(0,0).norm() < 3.0){
            vis_smooth_path.push_back(judge_goal_point);
        }
        PublishPath();
        planned_ = true;
        std::cout << judge_start_point.transpose() << std::endl;
        std::cout << judge_goal_point.transpose() << std::endl;
        std::cout << (judge_start_point - judge_goal_point).block<2,1>(0,0).norm() << std::endl;
        std::cout << "\e[1;39;49m>>> continue\e[0m" << std::endl;
        if(!HybridAStar::Constants::manual && replan_)
            std::this_thread::sleep_for(std::chrono::seconds(10));
        return false;
    }

}

bool HybridAStarGraphSearcher::Plan(const Eigen::Vector3f& start_pt, const Eigen::Vector3f& goal_pt){
    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;

    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    // the search works in cells, start and goal are in metres from the map origin (the original passed metres:
    // right for 1 m cells only)
    const float cs = Constants::cellSize;
    const Node3D nGoal(goal_pt.x() / cs, goal_pt.y() / cs, Helper::normalizeHeadingRad(goal_pt.z()), 0, 0, nullptr);
    Node3D nStart(start_pt.x() / cs, start_pt.y() / cs, Helper::normalizeHeadingRad(start_pt.z()), 0, 0, nullptr);
    auto t0 = std::chrono::steady_clock::now();
    std::cout << "\e[1;38;49m************************************************ \e[0m" << std::endl;
    std::cout << "\e[1;32;49m>>> start!!! \e[0m" << std::endl;
    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization);
    GetPath(nSolution);
    auto t1 = std::chrono::steady_clock::now();
    std::cout << "\e[1;35;49m>>> time_cost : " << std::chrono::duration<double, std::milli>(t1 - t0).count() << " ms\e[0m" << std::endl;

    for(int i = 0 ; i < path.size(); i++){  // cells -> metres
        path[i].setX(path[i].getX() * cs);
        path[i].setY(path[i].getY() * cs);
    }
    std::reverse(path.begin(),path.end());
    DealPath(this->path, vis_path);
    visualization.publishNode3DCosts(nodes3D, width, height, depth);
    visualization.publishNode2DCosts(nodes2D, width, height);

    if(vis_path.empty())
        return false;

    
    smoother_ptr_->tracePath(nSolution);
    smoother_ptr_->smoothPath(voronoiDiagram);

    smooth_path.clear();
    vis_smooth_path.clear();
    smooth_path = smoother_ptr_->getPath();
    for(int i = 0 ; i < smooth_path.size(); i++){
        smooth_path[i].setX(smooth_path[i].getX() * cs + x_origin_);
        smooth_path[i].setY(smooth_path[i].getY() * cs + y_origin_);
    }
    smoothedPath.updatePath(smooth_path);
    std::reverse(smooth_path.begin(),smooth_path.end());
    DealPath(smooth_path, vis_smooth_path);

//    smoothedPath.publishPath();
//    smoothedPath.publishPathNodes();
    smoothedPath.publishPathVehicles();

    delete[] nodes2D;
    delete[] nodes3D;
//    delete nSolution;

    return true;

}

void HybridAStarGraphSearcher::GetPath(const Node3D* node, int i, std::vector<Node3D> path){
    if (node == nullptr) {
        this->path.clear();
        this->path = path;
        return;
    }
    i++;
    path.push_back(*node);
    GetPath(node->getPred(), i, path);
}

void HybridAStarGraphSearcher::DealPath(const std::vector<Node3D>& path_, std::vector<Eigen::Vector3f>& finish_path){

    finish_path.clear();
    if (path_.size() < 2) {  // the loop below runs to size() - 1 (unsigned: an empty path crashed)
        return;
    }

    std::vector<Eigen::Vector3f> temp_path;
    for(int i = 0; i < path_.size(); i++){
        temp_path.push_back(Eigen::Vector3f(path_[i].getX(),
                                            path_[i].getY(),
                                              path_[i].getT()));
    }

    static float history_yaw = 0.0;
    for(int i = 0 ; i < temp_path.size() -1; i++){
        double theta = temp_path[i].z();
        // (1.5*PI , 2.0*PI]
        if(theta > 1.5 * M_PI) {
            theta = (theta - 2 * M_PI);
            // std::cout << "(1.5*PI , 2.0*PI]theta : " << theta << ", z : " << temp_path[i].z() << std::endl;
        }
        // (1.0*PI , 1.5*PI]
        else if(theta <= 1.5 * M_PI && theta > M_PI) {
            theta = -(2 * M_PI - theta);
            // std::cout << "(1.0*PI , 1.5*PI]theta : " << theta << ", z : " << temp_path[i].z() << std::endl;
        }
        // (0.0*PI , 1.0*PI]
        else if(theta <=  M_PI && theta >= 0.0 * M_PI) {
            if(i != 0){
                float theta_ = theta - M_PI;
                // std::cout << " diff_1 : " << std::abs(Helper::calculation_yaw_difference(theta_, history_yaw)) << std::endl;
                // std::cout << " diff_2 : " << std::abs(Helper::calculation_yaw_difference(theta, history_yaw)) << std::endl;
                // std::cout << " theta_ : " << theta_ << std::endl;
                // std::cout << " theta  : " << theta << std::endl;
                // std::cout << " history_yaw : " << history_yaw << std::endl;
                if(std::abs(Helper::calculation_yaw_difference(theta_, history_yaw)) < 1.0)
                    theta = theta_;
                if(std::abs(Helper::calculation_yaw_difference(theta, history_yaw)) < 1.0)
                    theta = theta;
            }
            else
                theta = theta;
            // std::cout << "[0.0*PI , 1.0*PI]theta : " << theta << ", z : " << temp_path[i].z() << std::endl;
        }
        else{
            float theta_ = theta + M_PI;
            if(std::abs(Helper::calculation_yaw_difference(theta_, history_yaw)) < 1.0)
                theta = theta_;
            if(std::abs(Helper::calculation_yaw_difference(theta, history_yaw)) < 1.0)
                theta = theta;
            // std::cout << "-[0.0*PI , 0.5*PI]theta : " << theta << ", z : " << temp_path[i].z() << std::endl;
        }
        history_yaw = theta;
        finish_path.push_back(Eigen::Vector3f(temp_path[i].x(), temp_path[i].y(), theta));
    }

}


void HybridAStarGraphSearcher::PublishPath(){

    path_publisher_ptr_->updatePath(path);
    path_publisher_ptr_->publishPath();
    path_publisher_ptr_->publishPathNodes();
    path_publisher_ptr_->publishPathVehicles();

    front_end_path_publisher_ptr_->Publish(vis_smooth_path);

}

}
