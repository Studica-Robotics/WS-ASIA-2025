#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveTrain.h"
#include "RobotContainer.h"

enum class RobotState{
    CuretEnturenc,Idle,RotateRight,RotateLeft, Forward, FolloWall
};


class Navigation : public frc2::CommandHelper<frc2::CommandBase, Navigation>
{
    public:
        Navigation(DriveTrain* drive);
        void Initialize() override;
        void Execute() override;
        void End(bool interrupted) override;
        bool IsFinished() override;
        double CalculateLinearSpeed();
        double CalculateAngularSpeed(double lidar_point_1,double lidar_point_2);
        double CalculateSideSpeed();
        std::tuple<double,double,double> CalculateVelocityT();

        int rotation_counter = 0;
        bool robot_in_curt = false;
        bool isRotateRight = false;
        bool isRotateLeft = false;
        bool do_once_flag{true};
        bool is_current_state_done{false};
        double tolerance = 2.0;
        double currentAngle;

        void Kinematics( double x, double y, double z );

        void HandleForward( double delay, double speed );


        void HandleIdleState();
        void HandleCurtEnturence();
        void HandleFollowingWallState();
        void HandleRotateLeftState();
        void HandleRotateRightState();
        void HandleRotationStation(bool is_rotate_left);
        void ResetFlags();
        int CheckLidarAngle(int angel1,int angle2);
        std::pair<double,double> CalculateVelocity();

        RobotState IsRobotState = RobotState::Idle;
        RobotState NextState    = RobotState::Idle;


        std::chrono::time_point<std::chrono::high_resolution_clock> current_time = std::chrono::high_resolution_clock::now();
        std::chrono::time_point<std::chrono::high_resolution_clock> last_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> delta_time;
        double linear_speed;
        double side_speed;      
        double angular_speed;       
        int targetAngle;      


        /** L movement **/
        double linear_speed_L = 0;
        double delay_L = 0;
        int step_L = 0;  

        bool second_arc = false;


    private:    
        DriveTrain* drive;
};