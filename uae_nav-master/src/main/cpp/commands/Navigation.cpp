#include <commands/Navigation.h>

// Aligned Angle is 85


#define TimeToEnterCurt 2.5
#define SpeedToEnterCurt 0.3

#define AngularVelocityLeft1 0.23
#define AngularVelocityLeft2 0.26

#define AngularVelocityRight 0.5
#define LidarPoint1 160



static constexpr bool backward_recovery = true;




static constexpr bool arcMove = true;

/** L Movement **/
static constexpr double inplace_rotation_left = 0.4;   // rad/s

static constexpr double time_L_move_1 = 1;             // s
static constexpr double linear_L_speed_1 = 0.3;        // m/s 

static constexpr double time_L_move_2 = 0;             // s
static constexpr double linear_L_speed_2 = 0.0;        // m/s

/** Second Forward Second L in sequence **/
static constexpr double linear_L_speed_second = 0.0;   // m/s 
static constexpr double time_L_move_second = 0;        // s


/*
*   Move 1: First Forward move of FIRST Arc/L move
*   Move 2: Second Forward of any Arc/L move
*   Second: First Forward move of SECOND Arc/L move in sequence
*
*   Example: Forward(1) -> Arc -> CHECK -> Arc                                                 then: Move 1 != 0, Move 2 = 0, Second =  0
*   Example: Forward(1) -> Arc -> CHECK -> Forward (Second) -> Arc                             then: Move 1 != 0, Move 2 = 0, Second != 0
*   Example: Forward(1) -> Arc -> Forward(2) -> CHECK -> Forward (Second) -> Arc -> Forward(2) then: Move 1 != 0, Move 2 != 0, Second != 0
*/

/* L MODE with ARC Movement - Comment the IF in the HandleRotateLeftState() */




Navigation::Navigation(DriveTrain* drive):drive{drive}
{
AddRequirements({drive});
}
void Navigation::Initialize() 
{
    drive->ResetYaw();
    IsRobotState = RobotState::FolloWall;
    frc::SmartDashboard::PutBoolean("test",0);
    robot_in_curt = true;
    is_current_state_done = true;

}
void Navigation::Execute() 
{
    drive->frontLeftMotor.Set(0.2);
    drive->backLeftMotor.Set(0.2);
    drive->frontRightMotor.Set(0.2);
    drive->backRightMotor.Set(0.2); 
    // drive->XBotMotorControl(0,0.2,0); 
    // drive->MecanumMotorControl(0.2, 0, 0);


    frc::SmartDashboard::PutNumber("Angle",CheckLidarAngle(LidarPoint1,LidarPoint1+10));
    frc::SmartDashboard::PutNumber("front_mean_dist",drive->front_mean_dist);
    frc::SmartDashboard::PutNumber("left_mean_dist", drive->left_mean_dist);
    

    frc::SmartDashboard::PutBoolean("StartButton", drive->GetStartButton());
    frc::SmartDashboard::PutBoolean("StoptButton", drive->GetEStopButton());

    if( drive->backward && backward_recovery ){
        linear_speed_L = -0.3;      // Negative
        delay_L = 1.5;
        IsRobotState = RobotState::Forward;
    }
    else if(is_current_state_done&& !robot_in_curt && NextState == RobotState::Idle)
    {
        IsRobotState = RobotState::FolloWall;
        second_arc = false;

    }
    else {
        if((drive->left_obj_flag && !drive->front_obj_flag)  && is_current_state_done && NextState == RobotState::Idle)
        {
            IsRobotState = RobotState::CuretEnturenc;
            second_arc = false;

        }
        else if(((drive->left_obj_flag && drive->front_obj_flag) || (!drive->left_obj_flag && drive->front_obj_flag))&& is_current_state_done && NextState == RobotState::Idle)
        {
            isRotateRight = true;
            isRotateLeft = false;
            IsRobotState = RobotState::RotateLeft;
            second_arc = false;

        }
        else if(!drive->left_obj_flag && !drive->front_obj_flag && is_current_state_done && arcMove == true && NextState == RobotState::Idle)
        {
            isRotateRight = false;
            isRotateLeft = true;
            second_arc = false;
            IsRobotState = RobotState::RotateRight;
        }

        else if((!drive->left_obj_flag && !drive->front_obj_flag && is_current_state_done && arcMove == false && NextState == RobotState::Idle) || ( NextState == RobotState::RotateLeft && is_current_state_done ))
        {
            frc::SmartDashboard::PutNumber("step_L", step_L);

            if( IsRobotState == RobotState::Idle && NextState != RobotState::Idle ){ 
                step_L++; }

            if( step_L == 0 ){
                if( second_arc ){
                    linear_speed_L = linear_L_speed_second;
                    delay_L = time_L_move_second;
                    second_arc = false;
                }else{            
                    linear_speed_L = linear_L_speed_1;
                    delay_L = time_L_move_1;
                    second_arc = true;
                }
                IsRobotState = RobotState::Forward;
                NextState = RobotState::RotateLeft;
            }else if( step_L == 1 ){
                isRotateRight = false;
                isRotateLeft = true;
                IsRobotState = RobotState::RotateLeft;
                NextState = RobotState::RotateLeft;
            }else if( step_L == 2 ){
                linear_speed_L = linear_L_speed_2;
                delay_L = time_L_move_2;
                IsRobotState = RobotState::Forward;
                NextState = RobotState::RotateLeft;
            }else{
                step_L = 0;
                IsRobotState = RobotState::Idle;
                NextState = RobotState::Idle;
            }
        }
    }

    switch (IsRobotState)
    {
    case RobotState::CuretEnturenc: HandleCurtEnturence(); break;
    case RobotState::Idle: HandleIdleState(); break;
    case RobotState::FolloWall: HandleFollowingWallState(); break;
    case RobotState::RotateRight: HandleRotateRightState(); break;
    case RobotState::RotateLeft: HandleRotateLeftState(); break;
    case RobotState::Forward: HandleForward( delay_L, linear_speed_L ); break;
    // case RobotState::Forward: HandleForward( 2, 0.3 ); break;

    default:
        break;
    }
}
void Navigation::End(bool interrupted) 
{

}
bool Navigation::IsFinished() 
{
    return false;
}


void Navigation::HandleIdleState()
{
    frc::SmartDashboard::PutString("RobotState", "IdleState");
    is_current_state_done = false;
    if(do_once_flag)
    {
        ResetFlags();
        do_once_flag = false;
        last_time =  std::chrono::high_resolution_clock::now();
        drive->StopMotor();
    }

    current_time =  std::chrono::high_resolution_clock::now();

    delta_time = current_time - last_time;
    if(delta_time.count() >=0.1)
    {
        drive->StopMotor();
        is_current_state_done = true;
        do_once_flag = true;
    }
}

void Navigation::HandleCurtEnturence()

{
    frc::SmartDashboard::PutString("RobotState", "CurtEnturence");
    is_current_state_done = false;
    if(do_once_flag )
    {
        do_once_flag = false;
        last_time =  std::chrono::high_resolution_clock::now();

    }

    current_time =  std::chrono::high_resolution_clock::now();

    delta_time = current_time - last_time;
    if(delta_time.count() < TimeToEnterCurt)
    {
        Kinematics( SpeedToEnterCurt, 0, 0 );
    }
    else
    {
        do_once_flag = true;
        robot_in_curt = true;
        IsRobotState = RobotState::Idle;
    }
    
}

void Navigation::HandleFollowingWallState()

{
    frc::SmartDashboard::PutString("RobotState", "FollowingWallState");
    is_current_state_done = false;
    if (do_once_flag)
    {
        do_once_flag = false;
        last_time =  std::chrono::high_resolution_clock::now();
    }
    current_time =  std::chrono::high_resolution_clock::now();
    delta_time = current_time - last_time;

    if( drive->front_obj_flag || delta_time.count()>2)
    {
        do_once_flag = true;
        IsRobotState = RobotState::Idle;
    }
    else
    {
        auto velocity = CalculateVelocity();
        frc::SmartDashboard::PutNumber("angular",velocity.second);   
        Kinematics(velocity.first, 0, velocity.second);
    }
}

void Navigation::HandleRotateLeftState()
{
    frc::SmartDashboard::PutString("RobotState", "RotateLeftState");
    HandleRotationStation(true);
}

void Navigation::HandleRotateRightState()
{
    frc::SmartDashboard::PutString("RobotState", "RotateRightState");
    HandleRotationStation(false);
}
void Navigation::HandleRotationStation(bool is_rotate_left)
{
    is_current_state_done = false;
    currentAngle = drive->GetGlobalAngle();
    if(do_once_flag)
    {
        do_once_flag = false;

        if( arcMove == true ){
            targetAngle = int(currentAngle + (is_rotate_left? 45:-45) +360)%360;
        }else{
            targetAngle = int(currentAngle + (is_rotate_left? 90:-45) +360)%360;
        }

        if (is_rotate_left)
        {
            rotation_counter++;
        }
        else
        {
            rotation_counter = 0;
        }
        
    }

    if(std::abs(currentAngle-targetAngle)< 2)
    {
        isRotateLeft = false;
        isRotateRight = false;
        do_once_flag = true;
        IsRobotState = RobotState::Idle;
    }
    else
    {
        auto angular_compute = drive->ArcCompute(rotation_counter == 1 ? AngularVelocityLeft1 :AngularVelocityLeft2 , rotation_counter == 1 ? 1:1);
        
        if( arcMove == false ){
            angular_compute.first  = 0;
            angular_compute.second = inplace_rotation_left;
        }

        Kinematics(is_rotate_left ?angular_compute.first:0, 0, is_rotate_left?angular_compute.second:-AngularVelocityRight); 
        frc::SmartDashboard::PutNumber("angular_compute.first",angular_compute.first);
    }
    

}
void Navigation::ResetFlags()
{
    isRotateRight = false;
    isRotateLeft = false;    
}
std::pair<double,double> Navigation::CalculateVelocity()
{
    
    const double target_distance = 0.26; // Target distance to the wall in meters
    const double dist_tolorance = 0.04;
    const int target_angle = 100;
    double angle_tolerance = 5;

    
    double angle_diff_degree = CheckLidarAngle(LidarPoint1,LidarPoint1+10);


    double angular_speed = 0;
    double linear_speed = 0;

    if (((abs(angle_diff_degree-target_angle)<= angle_tolerance) && (abs(drive->left_mean_dist- target_distance) <dist_tolorance ))) 
    {
        double angle_speed_angle = std::clamp(-0.02 * angle_diff_degree +2 ,-0.3,0.3); 
        angular_speed = angle_speed_angle;
        linear_speed  = std::clamp((-0.0011*angle_diff_degree*angle_diff_degree) + (0.1943*angle_diff_degree)- 7.9,0.1,0.4);
    } 
    else {
        double angle_speed_angle = std::clamp(-0.02 * angle_diff_degree +2 ,-0.3,0.3); 
        double angle_speed_wall = std::clamp(2* drive->left_mean_dist -0.875,-0.1,0.1);

        angular_speed = std::clamp(angle_speed_angle+angle_speed_wall,-0.4,0.4);
        // angular_speed = std::max( std::min( angular_speed, 0.15 ), -0.15 );
        frc::SmartDashboard::PutNumber("angular_speed",angular_speed);
        linear_speed  = std::clamp((-0.0017*angle_diff_degree*angle_diff_degree) + (0.2914*angle_diff_degree)- 12.08,0.15,0.4);
    }

    return {linear_speed,angular_speed};
}


int Navigation::CheckLidarAngle(int angle1,int angle2)
{
    double lidar_angle_diff = angle1 - angle2;
    double a = drive->GetLidarDist(angle1);  // in meters
    double b = drive->GetLidarDist(angle2);  // in meters
    double C_degrees = abs(lidar_angle_diff);     
    double C_radians = C_degrees * M_PI / 180.0;
    double c = std::sqrt((a*a + b*b) - (2*a*b * std::cos(C_radians)));
    double A_radians = std::acos((b*b + c*c - a*a)/(2*c*b));
    double A_degree = (A_radians * (180 / M_PI)) ;
    return (A_degree);
}


void Navigation::HandleForward( double delay, double speed ){

    frc::SmartDashboard::PutString("RobotState", "Forward");
    is_current_state_done = false;
    if(do_once_flag )
    {
        do_once_flag = false;
        last_time =  std::chrono::high_resolution_clock::now();
    }

    current_time =  std::chrono::high_resolution_clock::now();

    delta_time = current_time - last_time;
    if(delta_time.count() < delay)
    {
        Kinematics(speed, 0, 0);
    }
    else
    {
        do_once_flag = true;
        IsRobotState = RobotState::Idle;
        drive->backward = false;
    }
    
}

// x = Linear, y = 0, z = Angular
void Navigation::Kinematics( double x, double y, double z ){
    drive->MecanumMotorControl(x, y, z);

    frc::SmartDashboard::PutNumber("x",x);
    frc::SmartDashboard::PutNumber("y",y);
    frc::SmartDashboard::PutNumber("z",z);

}
