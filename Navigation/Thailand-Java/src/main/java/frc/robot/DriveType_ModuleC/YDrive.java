/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.DriveType_ModuleC;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class YDrive {

  public static final int TITAN_ID = 42;
  public static final int MOTOR_FRONT = 0;    
  public static final int MOTOR_LEFT = 1;
  public static final int MOTOR_RIGHT = 2;    


  public static double speedY = 0.2 ;
  public static double speedZ = 0.08 ;

  public static double setpointDistanceY = 350; 
  public static double setpointDistanceZ = 400;

  public static double epsilonDistanceY = 50;
  public static double epsilonDistanceZ = 50;

  public static double ProtectDistance_Y = 350;
  public static double ProtectDistance_Left = 250;

  
  public static int FRONT_LIDAR = 270 ; 
  public static int LEFT_LIDAR = -45 ; 
  public static int RIGHT_LIDAR = +45 ; 
  public static int LEFT_BACK_LIDAR = -105 ; 
  public static int RIGHT_BACK_LIDAR = +135 ; 


  public static int FRONT_MIN_ANGLE_LEFT = FRONT_LIDAR - 30 ; 
  public static int FRONT_MAX_ANGLE_LEFT = FRONT_LIDAR ; 
  public static int FRONT_MIN_ANGLE_RIGHT = FRONT_LIDAR ; 
  public static int FRONT_MAX_ANGLE_RIGHT = FRONT_LIDAR + 30 ; 

  public static int LEFT_MIN_ANGLE_LEFT = FRONT_LIDAR + LEFT_LIDAR - 10;
  public static int LEFT_MAX_ANGLE_LEFT = FRONT_LIDAR + LEFT_LIDAR  ;
  public static int LEFT_MIN_ANGLE_RIGHT = FRONT_LIDAR + LEFT_LIDAR   ;
  public static int LEFT_MAX_ANGLE_RIGHT = FRONT_LIDAR + LEFT_LIDAR + 10 ; 

  public static int RIGHT_MIN_ANGLE_LEFT = FRONT_LIDAR + RIGHT_LIDAR - 10;
  public static int RIGHT_MAX_ANGLE_LEFT = FRONT_LIDAR + RIGHT_LIDAR  ;
  public static int RIGHT_MIN_ANGLE_RIGHT = FRONT_LIDAR + RIGHT_LIDAR   ;
  public static int RIGHT_MAX_ANGLE_RIGHT = FRONT_LIDAR + RIGHT_LIDAR + 10 ; 



  public static int LEFT_PROTECT_MIN_ANGLE_LEFT = FRONT_LIDAR + LEFT_BACK_LIDAR - 20;
  public static int LEFT_PROTECT_MAX_ANGLE_LEFT = FRONT_LIDAR + LEFT_BACK_LIDAR  ;
  public static int LEFT_PROTECT_MIN_ANGLE_RIGHT = FRONT_LIDAR + LEFT_BACK_LIDAR   ;
  public static int LEFT_PROTECT_MAX_ANGLE_RIGHT = FRONT_LIDAR + LEFT_BACK_LIDAR + 20 ; 

  public static int RIGHT_BACK_MIN_ANGLE_LEFT = FRONT_LIDAR + RIGHT_BACK_LIDAR - 10;
  public static int RIGHT_BACK_MAX_ANGLE_LEFT = FRONT_LIDAR + RIGHT_BACK_LIDAR  ;
  public static int RIGHT_BACK_MIN_ANGLE_RIGHT = FRONT_LIDAR + RIGHT_BACK_LIDAR   ;
  public static int RIGHT_BACK_MAX_ANGLE_RIGHT = FRONT_LIDAR + RIGHT_BACK_LIDAR + 10 ; 





    /**
   * Protect Back
   */

  public final int RIGHT_PROTECT_MIN_ANGLE_LEFT = 0;
  public final int RIGHT_PROTECT_MAX_ANGLE_LEFT = 10;
  public final int RIGHT_PROTECT_MIN_ANGLE_RIGHT = 11;
  public final int RIGHT_PROTECT_MAX_ANGLE_RIGHT = 31;

  /**
   * Protech Front
   */

  public final int LEFT_PROTECT_MIN_ANGLE_FRONT = 180;
  public final int LEFT_PROTECT_MAX_ANGLE_FRONT = 220;

  public final int RIGHT_PROTECT_MIN_ANGLE_FRONT = 319;
  public final int RIGHT_PROTECT_MAX_ANGLE_FRONT = 359;



  // public final int block_distance_Front = 180;
  public final int block_distance_Front = 350;
  public final int block_distance_Left =285;
  public final int block_distance_Right = 285;

  public final double front_protech = 310;
  public final double back_protech = 360;

  public final double left_protech = 200;
  public final double right_protech = 200;

  /**
   * Protect
   */
  public final double left_back_protech = 255;
  public final double right_back_protech = 255;

  public final double left_front_protech = 200;
  public final double right_front_protech = 200;

  public double lastTime;
  double currentTime;
  public boolean hasSavedFirstTime = false;
  public boolean timerStarted;

  public final double speed_x = 0.1; 
  public final double speed_y = 0.18;
  public final double speed_z = 0.1;
  public final double speed_z_state = 0.1; 
  public final double speed_y_protec = 0.15;
  public final double error_x_protec = 0.002; 
  public final double error_z_protec = 0.0015;
  public final double error_z_middle = 0.0015;
  public final double determine_protec = 0.08;
  public final double time_front_block = 0.7;
  public final double time_left_block = 0.1;
  public final double time_right_block = 0.2;


}
