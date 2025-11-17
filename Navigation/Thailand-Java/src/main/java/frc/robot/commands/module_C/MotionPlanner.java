/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.module_C;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.module_C.AStarNavigator;
import frc.robot.subsystems.module_C.DWANavigator;
import frc.robot.subsystems.module_C.Navigate;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Sensor;

import java.util.List;

public class MotionPlanner extends CommandBase {
  /**
   * Creates a new MotionPlanner.
   */

  private static final DriveTrain drive = RobotContainer.driveTrain;
  private static final Sensor sensor = RobotContainer.sensor;
  private final Navigate navigate = RobotContainer.navigate; // ใช้ navigate จาก RobotContainer
  private final AStarNavigator aStarNavigator =RobotContainer.aStarNavigator;
  private final DWANavigator dwaNavigator = RobotContainer.dwaNavigator;

  public MotionPlanner() {
    addRequirements(drive, sensor);
  }



     /**
     * ตัวอย่างการใช้ AStar เพื่อหา movement vector
     */
    public double[] getAStarMovement(int[][] grid, int startX, int startY, int goalX, int goalY) {
      // ตรวจสอบว่า speed_x, speed_y, speed_z เป็น public ใน Navigate.java
      double speed_x = navigate.speed_x;
      double speed_y = navigate.speed_y;
      double speed_z = navigate.speed_z;
  
      return aStarNavigator.calculateAStarMovement(grid, startX, startY, goalX, goalY, speed_x, speed_y, speed_z);
    }

  /**
   * ตัวอย่างการใช้ DWA เพื่อหา velocity ที่ปลอดภัย
   */
  public DWANavigator.Velocity getDWAMovement(
          double robotX, double robotY, double robotTheta,
          double robotVx, double robotVy, double robotOmega,
          double goalX, double goalY,
          List<double[]> obstacles) {

      DWANavigator.RobotState state = new DWANavigator.RobotState(robotX, robotY, robotTheta, robotVx, robotVy, robotOmega);
      return dwaNavigator.plan(state, goalX, goalY, obstacles);
  }

  /**
   * ตัวอย่างการสั่งงานมอเตอร์ (ควรเรียกใน command หรือ subsystem)
   */
  public void driveWithAStar(int[][] grid, int startX, int startY, int goalX, int goalY, DriveTrain drive) {
      double[] movement = getAStarMovement(grid, startX, startY, goalX, goalY);
      // movement[0]=cal_x, movement[1]=cal_y, movement[2]=cal_z
      drive.setDriveMotorSpeeds(movement[0], movement[1]);
      // สามารถปรับให้เหมาะกับประเภท drive ได้
  }

  public void driveWithDWA(
    double robotX, double robotY, double robotTheta,
    double robotVx, double robotVy, double robotOmega,
    double goalX, double goalY,
    List<double[]> obstacles,
    DriveTrain drive) {

DWANavigator.Velocity vel = getDWAMovement(robotX, robotY, robotTheta, robotVx, robotVy, robotOmega, goalX, goalY, obstacles);

// สร้าง gradualSpeeds สำหรับแต่ละแกน (ตัวอย่าง: vx=Z, vy=Y, omega=X)
double[] gradualSpeeds = new double[3];
gradualSpeeds[0] = vel.omega; // Z
gradualSpeeds[1] = vel.vy;    // Y
gradualSpeeds[2] = vel.vx;    // X

String type = navigate.TypeDrive();

// double[] gradualSpeeds = getGradualMotorSpeeds();
// navigate

if(type.equals("Differance")){
    drive.twoWheelMotorControl(gradualSpeeds[0], gradualSpeeds[1]); // Z,Y
}
else if(type.equals("TheStack")){
    drive.FourWheelMotorControl(gradualSpeeds[0], gradualSpeeds[1]); // Z,Y
}
else if(type.equals("SixWheelDrive")){
    drive.FourWheelMotorControl(gradualSpeeds[0], -(gradualSpeeds[1])); // Z,Y
}
else if(type.equals("XDrive") || type.equals("Mecanum")){
    drive.XDriveMotorControl(gradualSpeeds[2], gradualSpeeds[1], gradualSpeeds[0]); // X,Y,Z
}
else if(type.equals("YDrive")){
    drive.TriangleMotorControl(gradualSpeeds[2], gradualSpeeds[1], gradualSpeeds[0]); // X,Y,Z
}

// ส่งค่าความเร็วที่ปรับแล้วไปยังมอเตอร์
drive.Runninggreen(true);
drive.Runningred(false);
}
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      int[][] grid = new int[][] {
          {0, 0, 0, 1, 0},
          {0, 1, 0, 1, 0},
          {0, 1, 0, 0, 0},
          {0, 0, 0, 1, 0},
          {1, 1, 0, 0, 0}
      };
      int startX = 0;
      int startY = 0;
      int goalX = 4;
      int goalY = 4;
      DriveTrain drive = frc.robot.RobotContainer.driveTrain; // ใช้ RobotContainer
  
      driveWithAStar(grid, startX, startY, goalX, goalY, drive);
  
      // ตัวอย่างสำหรับ DWA (ถ้าต้องการทดสอบ DWA ให้ comment ส่วน AStar แล้ว uncomment ด้านล่าง)
      /*
      double robotX = 0.0;
      double robotY = 0.0;
      double robotTheta = 0.0;
      double robotVx = 0.0;
      double robotVy = 0.0;
      double robotOmega = 0.0;
      double dwaGoalX = 4.0;
      double dwaGoalY = 4.0;
      List<double[]> obstacles = List.of(
          new double[]{2.0, 2.0},
          new double[]{1.0, 3.0}
      );
      driveWithDWA(robotX, robotY, robotTheta, robotVx, robotVy, robotOmega, dwaGoalX, dwaGoalY, obstacles, drive);
      */

      
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
