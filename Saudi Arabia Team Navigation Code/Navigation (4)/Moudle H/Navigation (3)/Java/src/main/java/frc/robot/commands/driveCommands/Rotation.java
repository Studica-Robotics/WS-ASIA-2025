package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class Rotation extends CommandBase {
    private static final DriveTrain dr = RobotContainer.drive;

    private double rotationAngle;
    private PIDController zAxis;

    public Rotation(double rotationAngle)
    {
        this.rotationAngle = rotationAngle;
        zAxis = new PIDController(0.1, 0, 0);
        zAxis.setTolerance(1);

        addRequirements(dr); 
    }

    @Override
    public void initialize()
    {
        zAxis.reset();
        dr.resetNavX();
    }

    @Override
    public void execute()
    {
        double speed = 0.2;
        double rotationSpeed = MathUtil.clamp(zAxis.calculate(dr.getNavXAngle(), rotationAngle), -1, 1);
        double rotation = speed * rotationSpeed;
        
        dr.setTwoMotorSpeed(rotation, -rotation);
    }

    @Override
    public void end(boolean interrupted)
    {
        dr.setTwoMotorSpeed(0.0, 0.0); // Stop motor
    }

    @Override
    public boolean isFinished()
    {
        return zAxis.atSetpoint();
    }

}