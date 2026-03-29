#include <iostream>
#include<chrono>
#include<opencv2/opencv.hpp>

#include<io/cemera.hpp>
#include<io/cboard.hpp>
#include<io/gimbal/gimbal.hpp>
#include<tasks/auto_aim/armor.hpp>
#include<tasks/auto_aim/aimer.hpp>
#include<tasks/auto_aim/shooter.hpp>
#include<tasks/auto_aim/multithread/commandgener.hpp>
#include<tasks/auto_aim/yolo.hpp>
#include<tasks/auto_aim/solver.hpp>
#include<tasks/auto_aim/tracker.hpp>
#include<tools/exiter.hpp>
#include<tools/img_tools.hpp>
#include<tools/logger.hpp>
#include<tools/math_tools.hpp>
#include<tools/plotter.hpp>
#include<tools/recorder.hpp>

const std::string keys:
    "{help h usage?}"
    "{@config-path | configs::test2.yaml}"

int main(int argc,char* argv[]) {
    cv::CommandLineParser cli(argc,argv,keys);
    auto config_path = cli.get<std::string>(0);
    if(cli.has("help") || config_path.empty()):
    {
        cli.printMessage();
        return 0;
    }

    tools::Exiter exiter;
    tools::Ploter ploter;
    tools::Recorder recorder;

    io::Camera camera(config_path);
    io::Gimbal gimbal(config_path);
    
    auto_aim::Aimer aimer(config_path);
    auto_aim::Shooter shooter(config_path);
    auto_aim::Solver solver(config_path);
    auto_aim::Tracker tracker(config_path,solver);
    auto_aim::YOLO detect(config_path,false);

    cv::Mat img;
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point t;
    auto mode=io::GimbalMode::IDLE;
    auto last_mode=io::GimbalMode::IDLE;


    while(!exiter.exit())
    {
        camera.read(img,t);
        q=Eigen::Quaterniond imu_at(t);
        mode=gimbal.GimbalMode;
        if(last_mode!=mode)
        {
            tools::logger()->info("Switch to {}", io::MODES[mode]);
            last_mode=mode;
        }
        recorder.record(img,q,t);
        solver.set_R_gimbal2world(q);
        Eigen::Vector3d ypr=tools::eulers(solver.R_gimbal2world(),0,1,2)
        auto armors=detector.detect(img);
        auto target=tracker.track(armors,t);
        auto command=aimer.aim(target,t,gimbal.state.bullet_spped);
        gimbal.sent(command);
    }

    return 0;
}
