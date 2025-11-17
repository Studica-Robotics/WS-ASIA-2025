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
public final class SixWheelDrive {

  public static final int TITAN_ID = 42;
  public static final int MOTOR_LEFT_FRONT = 2;
  public static final int MOTOR_LEFT_BACK = 3;
  public static final int MOTOR_RIGHT_FRONT = 0 ;    
  public static final int MOTOR_RIGHT_BACK = 1;


  public static double speedY = 0.16 ;
  public static double speedZ = 0.16 ;

  public static double setpointDistanceY = 300; 
  public static double setpointDistanceZ = 325;
  // public static double setpointDistanceZ = 325;

  public static double epsilonDistanceY = 50;
  public static double epsilonDistanceZ = 50;

  public static double ProtectDistance_Y = 325;
  public static double ProtectDistance_Left = 275;

  
  public static int FRONT_LIDAR = 270 ; 
  public static int LEFT_LIDAR = -75 ; 
  public static int RIGHT_LIDAR = +75 ; 
  public static int LEFT_BACK_LIDAR = -45 ; 
  public static int RIGHT_BACK_LIDAR = +135 ; 


  public static int FRONT_MIN_ANGLE_LEFT = FRONT_LIDAR - 60 ; 
  public static int FRONT_MAX_ANGLE_LEFT = FRONT_LIDAR ; 
  public static int FRONT_MIN_ANGLE_RIGHT = FRONT_LIDAR ; 
  public static int FRONT_MAX_ANGLE_RIGHT = FRONT_LIDAR  ; 

  public static int LEFT_MIN_ANGLE_LEFT = FRONT_LIDAR + LEFT_LIDAR - 10;
  public static int LEFT_MAX_ANGLE_LEFT = FRONT_LIDAR + LEFT_LIDAR  ;
  public static int LEFT_MIN_ANGLE_RIGHT = FRONT_LIDAR + LEFT_LIDAR   ;
  public static int LEFT_MAX_ANGLE_RIGHT = FRONT_LIDAR + LEFT_LIDAR + 10 ; 

  public static int RIGHT_MIN_ANGLE_LEFT = FRONT_LIDAR + RIGHT_LIDAR - 10;
  public static int RIGHT_MAX_ANGLE_LEFT = FRONT_LIDAR + RIGHT_LIDAR  ;
  public static int RIGHT_MIN_ANGLE_RIGHT = FRONT_LIDAR + RIGHT_LIDAR   ;
  public static int RIGHT_MAX_ANGLE_RIGHT = FRONT_LIDAR + RIGHT_LIDAR + 10 ; 

  public static int LEFT_PROTECT_MIN_ANGLE_LEFT = FRONT_LIDAR + LEFT_BACK_LIDAR - 30;
  public static int LEFT_PROTECT_MAX_ANGLE_LEFT = FRONT_LIDAR + LEFT_BACK_LIDAR  ;
  public static int LEFT_PROTECT_MIN_ANGLE_RIGHT = FRONT_LIDAR + LEFT_BACK_LIDAR   ;
  public static int LEFT_PROTECT_MAX_ANGLE_RIGHT = FRONT_LIDAR + LEFT_BACK_LIDAR + 30 ; 

  public static int RIGHT_BACK_MIN_ANGLE_LEFT = FRONT_LIDAR + RIGHT_BACK_LIDAR - 10;
  public static int RIGHT_BACK_MAX_ANGLE_LEFT = FRONT_LIDAR + RIGHT_BACK_LIDAR  ;
  public static int RIGHT_BACK_MIN_ANGLE_RIGHT = FRONT_LIDAR + RIGHT_BACK_LIDAR   ;
  public static int RIGHT_BACK_MAX_ANGLE_RIGHT = FRONT_LIDAR + RIGHT_BACK_LIDAR + 10 ; 


  
  public final int BACK_PROTECT_MIN_ANGLE_LEFT = 250;
  public final int BACK_PROTECT_MAX_ANGLE_LEFT = 280;
  public final int BACK_PROTECT_MIN_ANGLE_RIGHT = 281;
  public final int BACK_PROTECT_MAX_ANGLE_RIGHT = 301;



  // Back_Protect แทน Right_Protect
  public final int RIGHT_PROTECT_MIN_ANGLE_LEFT = 210; // Right_Front
  public final int RIGHT_PROTECT_MAX_ANGLE_LEFT = 220;
  public final int RIGHT_PROTECT_MIN_ANGLE_RIGHT = 221; // Right_Back
  public final int RIGHT_PROTECT_MAX_ANGLE_RIGHT = 241;
  //อ้างอิงจากกึ่งกลางหุ่นยนต์ด้านขวาถึงล้อหน้าด้านขวา 

  
  //L_F
  public final int LEFT_PROTECT_MIN_ANGLE_FRONT = 30;
  public final int LEFT_PROTECT_MAX_ANGLE_FRONT = 80;
  //R_F
  public final int RIGHT_PROTECT_MIN_ANGLE_FRONT = 100;
  public final int RIGHT_PROTECT_MAX_ANGLE_FRONT = 150;


  public final int block_distance_Front = 370;
  public final int block_distance_Left = 370;
  public final int block_distance_Right = 370;

  public final double front_protech = 340;
  public final double back_protech = 320;
  public final double left_protech = 320;
  public final double right_protech = 320;

  public final double left_back_protech = 340;
  public final double right_back_protech = 340;

  public final double left_front_protech = 360;
  public final double right_front_protech = 360;
  public final double differ_conned_and_mid = 0;

  public double lastTime;
  double currentTime;
  public boolean hasSavedFirstTime = false;
  public boolean timerStarted;

  public final double speed_x = 0.2; 
  public final double speed_y = 0.15;
  public final double speed_z = 0.2;
  public final double speed_z_state = 0.2; 
  public final double speed_y_protec = 0.12;
  public final double error_x_protec = 0.0022;
  public final double error_z_protec = 0.002;
  public final double error_z_middle = 0.0025;
  public final double determine_protec = 0.08;
  public final double time_front_block = 0.7;
  public final double time_left_block = 0.1;
  public final double time_right_block = 0.2;


}
