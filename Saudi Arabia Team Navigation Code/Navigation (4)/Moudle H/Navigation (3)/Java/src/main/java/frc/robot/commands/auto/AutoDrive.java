package frc.robot.commands.auto;
import frc.robot.commands.driveCommands.DrivePID;
import frc.robot.commands.driveCommands.Lidar;
import frc.robot.commands.driveCommands.Rotation;
// import the SimpleDrive command
import frc.robot.commands.driveCommands.SimpleDrive;
//import frc.robot.commands.driveCommands.WallTrace;

/**
 * DriveMotor class
 * <p>
 * This class creates the inline auto command to drive the motor
 */
public class AutoDrive extends AutoCommand
{
    /**
     * Constructor
     */
    public AutoDrive()
    {
        /**
         * Calls the SimpleDrive command and adds a 5 second timeout
         * When the timeout is complete it will call the end() method in the SimpleDrive command
         */
        super(
          new DrivePID(1000)
          
        );

    }
}