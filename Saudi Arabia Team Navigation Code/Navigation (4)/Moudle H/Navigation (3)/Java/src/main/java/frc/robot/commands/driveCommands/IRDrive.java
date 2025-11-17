package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class IRDrive extends CommandBase {

    private static final DriveTrain drive = RobotContainer.drive;
    private double SetDistance;

    public IRDrive(double SetDistance) 
    {
    addRequirements(drive);
    this.SetDistance   =   SetDistance;
    SetDistance += 7;
    }

  @Override
  public void initialize() 
  {

  }

  @Override
  public void execute() {
    drive.setTwoMotorSpeed(0.6 ,0.6);
  }

  @Override
  public void end(boolean interrupted) {
  drive.setTwoMotorSpeed(0, 0);
 }
  

  @Override
public boolean isFinished() {
  double distance    = drive.getDistance();
  return distance    < SetDistance         ;
}


}
  




