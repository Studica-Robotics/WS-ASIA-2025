/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.module_C;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Sensor;

public class XDriveTest extends CommandBase {
  /**
   * Creates a new XDriveTest.
   */

  private static final DriveTrain drive = RobotContainer.driveTrain;
  private static final Sensor sensors = RobotContainer.sensor;
  
  public XDriveTest() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive,sensors);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drive.XDriveMotorSpeeds(0, 0); 
    drive.XDriveMotorControl(0, 0, 0);
    drive.FourWheelMotorControl(0.0, 0.0);
    drive.Runninggreen(true);
    drive.Runningred(false);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // drive.XDriveMotorSpeeds(0.3, 0.3); 
    // drive.XDriveMotorControl(0.0, 0.2,0.0);
    drive.FourWheelMotorControl(0.2, 0.0);
    // drive.setTriangleMotorSpeeds(0.0,0.2,-0.2);
    // drive.TriangleMotorControl(0.0,0.1,0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    drive.XDriveMotorSpeeds(0, 0);
    drive.XDriveMotorSpeeds(0,0);
    drive.XDriveMotorControl(0, 0, 0);
    drive.XDriveMotorControl(0, 0, 0);
    drive.XDriveMotorControl(0, 0, 0);
    drive.FourWheelMotorControl(0.0, 0.0);
    drive.Runninggreen(false);
    drive.Runningred(true);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
