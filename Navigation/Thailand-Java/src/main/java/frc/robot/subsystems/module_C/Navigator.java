/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.module_C;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.DriveType_ModuleC.XDrive;
import frc.robot.subsystems.Sensor;

public class Navigator extends SubsystemBase {
  /**
   * Creates a new Navigator.
   */

  private static final Sensor sensor = RobotContainer.sensor;
  private static final XDrive DriveType = RobotContainer.xDrive;



   private ShuffleboardTab tab;

   private NetworkTableEntry Lidar_Distance_Front;
   private NetworkTableEntry Distance_front_Min;
   private NetworkTableEntry Angle_front_Min;
   
   private NetworkTableEntry Lidar_Distance_Left;
   private NetworkTableEntry Lidar_Distance_Right;
   
   private NetworkTableEntry Distance_left_Min;
   private NetworkTableEntry Angle_left_Min;

   private NetworkTableEntry Distance_right_Min;
   private NetworkTableEntry Angle_right_Min;

  public Navigator() {

    if (Config.Sensor) {
      tab = Shuffleboard.getTab("Sensor");
      Distance_front_Min = tab.add("Distance_front_Min", 0).withPosition(5, 1).getEntry();
      Angle_front_Min = tab.add("Angle_front_Min", 0).withPosition(4, 1).getEntry();
      
      
      Lidar_Distance_Front = tab.add("Lidar_Distance", 0).withPosition(5, 2).getEntry();
      Lidar_Distance_Left = tab.add("Lidar_Distance_Left", 0).withPosition(4, 3).getEntry();
      Lidar_Distance_Right = tab.add("Lidar_Distance_Right", 0).withPosition(6, 3).getEntry();


      Distance_left_Min = tab.add("Distance_left_Min", 0).getEntry();
      Angle_left_Min = tab.add("Angle_left_Min", 0).getEntry();

      Distance_right_Min = tab.add("Distance_right_Min", 0).getEntry();
      Angle_right_Min = tab.add("Angle_right_Min", 0).getEntry();

    }
  }


   /*
   * 
   * 
   * LIDAR
   * 
   * 
   * 
   */

  private final double DEFAULT_MAX_DISTANCE = 9999.0;
  private final int FRONT_MIN_ANGLE_LEFT = DriveType.FRONT_MIN_ANGLE_LEFT;
  private final int FRONT_MAX_ANGLE_LEFT = DriveType.FRONT_MAX_ANGLE_LEFT;
  private final int FRONT_MIN_ANGLE_RIGHT = DriveType.FRONT_MIN_ANGLE_RIGHT;
  private final int FRONT_MAX_ANGLE_RIGHT = DriveType.FRONT_MAX_ANGLE_RIGHT;

  private final int LEFT_MIN_ANGLE_LEFT = DriveType.LEFT_MIN_ANGLE_LEFT;
  private final int LEFT_MAX_ANGLE_LEFT = DriveType.LEFT_MAX_ANGLE_LEFT;
  private final int LEFT_MIN_ANGLE_RIGHT = DriveType.LEFT_MIN_ANGLE_RIGHT;
  private final int LEFT_MAX_ANGLE_RIGHT = DriveType.LEFT_MAX_ANGLE_RIGHT;

  private final int RIGHT_MIN_ANGLE_LEFT = DriveType.RIGHT_MIN_ANGLE_LEFT;
  private final int RIGHT_MAX_ANGLE_LEFT = DriveType.RIGHT_MAX_ANGLE_LEFT;
  private final int RIGHT_MIN_ANGLE_RIGHT = DriveType.RIGHT_MIN_ANGLE_RIGHT;
  private final int RIGHT_MAX_ANGLE_RIGHT = DriveType.RIGHT_MAX_ANGLE_RIGHT;



  // ** Front */
  public double[] findFrontMinDistance() {
      double minDistance = DEFAULT_MAX_DISTANCE;
      double minAngle = FRONT_MIN_ANGLE_LEFT;

      for (int angle = FRONT_MIN_ANGLE_LEFT; angle <= FRONT_MAX_ANGLE_LEFT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);

      if (distance == 9999.0) {
          minDistance = distance;
          minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                  // เพื่อป้องกันค่าผิดพลาด
          minDistance = distance;
          minAngle = angle;
      }
      }

      for (int angle = FRONT_MIN_ANGLE_RIGHT; angle <= FRONT_MAX_ANGLE_RIGHT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);

      if (distance == 9999.0) {
          minDistance = distance;
          minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                  // เพื่อป้องกันค่าผิดพลาด
          minDistance = distance;
          minAngle = angle;
      }
      }

      Distance_front_Min.setDouble(minDistance);
      Angle_front_Min.setDouble(minAngle);

      return new double[] { minDistance, minAngle };
  }

  
  
// ** LEFT */

public double[] findLeftMinDistance() {
  double minDistance = DEFAULT_MAX_DISTANCE;
  double minAngle = LEFT_MIN_ANGLE_LEFT;

  for (int angle = LEFT_MIN_ANGLE_LEFT; angle <= LEFT_MAX_ANGLE_LEFT; angle += 5) {
    final double distance = sensor.getDistionLidar(angle);

    if (distance == 9999.0) {
      minDistance = distance;
      minAngle = angle;

    } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                               // เพื่อป้องกันค่าผิดพลาด
      minDistance = distance;
      minAngle = angle;
    }
  }

  for (int angle = LEFT_MIN_ANGLE_RIGHT; angle <= LEFT_MAX_ANGLE_RIGHT; angle += 5) {
    final double distance = sensor.getDistionLidar(angle);

    if (distance == 9999.0) {
      minDistance = distance;
      minAngle = angle;

    } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                               // เพื่อป้องกันค่าผิดพลาด
      minDistance = distance;
      minAngle = angle;
    }
  }

  Distance_left_Min.setDouble(minDistance);
  Angle_left_Min.setDouble(minAngle);

  return new double[] { minDistance, minAngle };
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    findFrontMinDistance();
    findLeftMinDistance();
  }
}
