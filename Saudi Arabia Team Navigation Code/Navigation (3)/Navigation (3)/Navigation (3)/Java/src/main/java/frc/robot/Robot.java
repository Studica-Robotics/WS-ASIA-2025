
package frc.robot;
import com.studica.frc.MockDS; //comment if not using mokds
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.AutoDrive;
//import edu.wpi.cscore.UsbCamera;
//import edu.wpi.first.cameraserver.CameraServer;



public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

   private MockDS ds; // Create the object //comment if not using mokds
  private boolean active = false; // active flag prevents calling mockds more than once.*/ //comment if not using mokds


  private RobotContainer m_robotContainer;

  
  @Override
  public void robotInit() {
    
    m_robotContainer = new RobotContainer();

     ds = new MockDS(); // Create the instance     //startbutton //comment if not using mokds


    // new Thread(() -> {
    //     UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    //     camera.setResolution(640, 480);
    // }).start();
}
  

  
    @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    // if (DriverStation.getInstance().isDisabled()) {
    //   m_robotContainer.drive.setStoppedLED(true);
    //   m_robotContainer.drive.setRunningLED(false);
    // } else {
    //   m_robotContainer.drive.setStoppedLED(false);
    //   m_robotContainer.drive.setRunningLED(true);
    // }

     // If the start button is pushed and the system is not active //comment if not using mokds
  if (!RobotContainer.drive.getStartButton() && !active)
  {
      ds.enable(); // enable the robot.
      active = true;
  }
  // If the e-stop is pushed and the system is active
  if (RobotContainer.drive.getEmergencyStop() && active)
  {
      ds.disable(); // disable the robot.
      active = false;
  }

  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    //Check to see if autoChooser has been created
    if(null == RobotContainer.autoChooser)
    {
      RobotContainer.autoChooser = new SendableChooser<>();
    }
    // //Add the default auto to the auto chooser
    RobotContainer.autoChooser.setDefaultOption("Drive Motor", "Drive Motor");
    RobotContainer.autoMode.put("Drive Motor", new AutoDrive());
    SmartDashboard.putData(RobotContainer.autoChooser);

  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
