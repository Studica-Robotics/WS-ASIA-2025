/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.module_C;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Config;
import frc.robot.RobotContainer;
import frc.robot.DriveType_ModuleC.YDrive;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Sensor;

public class TestRobotMovement extends CommandBase {
  /**
   * Creates a new TestRobotMovement.
   */
  private static final DriveTrain drive = RobotContainer.driveTrain;
  private static final Sensor sensor = RobotContainer.sensor;
  // private static final YDrive DriveType = RobotContainer.yDrive;


  public TestRobotMovement() {
    // Use addRequirements() here to declare subsystem dependencies.
 

  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.TriangleMotorControl(0,0,0);
    drive.XDriveMotorControl(0, 0, 0);
    drive.FourWheelMotorControl(0,0);
    drive.twoWheelMotorControl(0, 0);
    // drive.
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  drive.XDriveMotorControl(
    0.0, 
    0.2, // + Forward
    0.0  // + TurnRight
    );

  // drive.twoWheelMotorControl(
  //   0.0, // + Forward
  //   0.2); // + TurnRight


  // drive.SixWheelMotorControl(
  //   0.2, // + Forward
  //   0.0 // + TurnRight
  //   );

  // drive.FourWheelMotorControl(
  //   0.0,  // + Forward
  //   0.2 // TurnRight 
  //   );

  // drive.TriangleMotorControl(
  //   0.0, 
  //   0.2, 
  //   0.0
  //   );
    
  }

 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.TriangleMotorControl(0,0,0);
    drive.XDriveMotorControl(0, 0, 0);
    drive.FourWheelMotorControl(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
