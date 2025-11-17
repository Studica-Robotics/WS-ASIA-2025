package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class DrivePID extends CommandBase {
    private static final DriveTrain drive = RobotContainer.drive;

    private double setPointDistance;
    private PIDController yAxis;
    private PIDController zAxis;

    public DrivePID(double setPointDistance)
    {
        this.setPointDistance = setPointDistance;
        yAxis = new PIDController(1, 0, 0);
        yAxis.setTolerance(10);

        zAxis = new PIDController(0.1, 0, 0);
        zAxis.setTolerance(1);

        addRequirements(drive); 
    }
     
    @Override
    public void initialize()
    {
        yAxis.reset();
        zAxis.reset();
        drive.resetEncoders();
        drive.resetNavX();
    }

    @Override
    public void execute()
    {


        double forwardSpeed = MathUtil.clamp(yAxis.calculate(drive.getAverageEncoderDistance(), setPointDistance), -0.3, 0.3);
        double rotationSpeed = MathUtil.clamp(zAxis.calculate(drive.getNavXAngle(), 0.3), -0.3, 0.3);

        double leftSpeed = forwardSpeed + rotationSpeed;
        double rightSpeed = forwardSpeed - rotationSpeed;
        
        drive.setTwoMotorSpeed(leftSpeed, rightSpeed);

        
    }

    @Override
    public void end(boolean interrupted)
    {
        drive.setTwoMotorSpeed(0.0, 0.0); // Stop motor
        
    }

    @Override
    public boolean isFinished()
    {
        return yAxis.atSetpoint();
    }

}