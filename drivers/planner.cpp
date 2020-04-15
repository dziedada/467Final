// Planner Main
/*
 * LCM communication with EKF and Arm Controller
 */
#include <common/messages/arm_path_t.hpp>
#include <common/messages/ball_t.hpp>
#include <common/message_channels.hpp>
#include <common/point.hpp>
#include <planner/arm_planner.hpp>
#include <planner/TrackingVis.hpp>

#include <lcm/lcm-cpp.hpp>
#include <Eigen/Core>

#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <memory>
#include <vector>
#include <utility>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <string>
#include <csignal>
#include <functional>
#include <atomic>

using std::string;
using std::cout;
using std::endl;
using std::cerr;
using std::vector;
using std::pair;
using std::shared_ptr;
using std::mutex;
using std::condition_variable;
using std::function;
using std::atomic_bool;

const char* GUI_FLAG = "-g";

class Handler
{
public:
    shared_ptr<ArmPlanner> planner;
    Handler(shared_ptr<ArmPlanner> planner_in) : planner {planner_in} {}

    void handleEKFMessage( const lcm::ReceiveBuffer*, const std::string &channel, 
        const ball_detections_t *ball )
    {
		//cout << "Handling message" << endl;
        //cout << " update received at " << ball->utime << endl;

        planner->updateBalls( *ball );

        if(planner->balls.empty()) return;

        // REMOVED CALCULATE PLAN
        //std::vector<Vector2d> plan = planner->calculatePlan( );
        /*
        auto pos =  planner->balls[0].getPos();
        cout << pos << '\n';
        cout << " calculated Plan at " << endl;
        planner.publishPlan( plan );
        */
    }
    
private:
    int64_t prevTime = 0;
};

// Handles lcm in a loop until program no longer running
void handle_loop(lcm::LCM* lcm_ptr)
{
    while ( 0 == lcm_ptr->handle());
}

// Set up graceful exit code
function<void(int)> sigHandlerImpl;
void sigHandler(int sig) {sigHandlerImpl(sig);}

int main(int argc, char ** argv)
{
    // Set configuration values
    bool view_display = false;
    if (argc > 1 && !strcmp(argv[1], GUI_FLAG)) view_display = true;

    // Setup lcm
    lcm::LCM lcm;
    if ( !lcm.good( ) )
        return 1;

    // Create planner
    shared_ptr<ArmPlanner> planner = shared_ptr<ArmPlanner>(new ArmPlanner(&lcm));

    // Subscribe to detections and set up handler
    Handler handler(planner);
    lcm.subscribe(channel::BALL_DETECTIONS, &Handler::handleEKFMessage, &handler);

    // Set up signal handling
    shared_ptr<condition_variable> cond_var = planner->cond_var;
    shared_ptr<mutex> mtx = planner->mtx;
    atomic_bool running;
    running = true;
    sigHandlerImpl = [&] (int)
    {
        running = false;
        cond_var->notify_all();
    };
    signal(SIGINT, sigHandler);

    // Start lcm thread
    std::thread(handle_loop, &lcm).detach();

    if (view_display)
    {
        cout << "Running display\n";
        TrackingVis visualizer(planner->cond_var, planner->mtx, planner->getBalls());
        while (running)
        {
            std::unique_lock<std::mutex> lck(*mtx);
            cond_var->wait(lck);
            visualizer.update();
        }
    }
    else
    {
        while (running)
        {
            std::unique_lock<std::mutex> lck(*mtx);
            cond_var->wait(lck);
        }
    }

    /*
    auto publish_dummy_path = [&lcm]()
    {
        sleep( 2 );
        arm_path_t path;
        path.waypoints_num = 2;
        vector< vector< double > > waypoints( 2, vector< double >( 2, 0 ) );
        waypoints[0][0] = 0.1;
        waypoints[0][1] = 0.08;
        waypoints[1][0] = 0.08;
        waypoints[1][1] = 0.15;
        path.waypoints = waypoints;
        path.angles_num = 0;
        std::vector<std::vector<double>> vec( 0, vector<double>( 0, 0 ) );
        path.angles = vec;
        path.speed = 1;
        lcm.publish( "ARM_PATH", &path );
    };
    publish_dummy_path();
    */
}
