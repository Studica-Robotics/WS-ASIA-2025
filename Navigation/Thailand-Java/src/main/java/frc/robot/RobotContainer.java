/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.DriveType_ModuleC.Differance;
// import frc.robot.DriveType_ModuleC.Differance_2;
import frc.robot.DriveType_ModuleC.Mecanum;
import frc.robot.DriveType_ModuleC.SixWheelDrive;
import frc.robot.DriveType_ModuleC.TheStack;
// import frc.robot.DriveType_ModuleC.TheStack_MyBot;
import frc.robot.DriveType_ModuleC.YDrive;
import frc.robot.DriveType_ModuleC.XDrive;
import frc.robot.commands.Autocommand;
import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.Navigator;
import frc.robot.subsystems.Sensor;
import frc.robot.subsystems.module_C.AStarNavigator;
import frc.robot.subsystems.module_C.DWANavigator;
import frc.robot.subsystems.module_C.Navigate;
// import frc.robot.subsystems.module_C.Navigate;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static DriveTrain driveTrain;
  public static Sensor sensor;



  public static Differance differance;
  public static XDrive xDrive;
  public static Mecanum mecanum;
  public static TheStack theStack;
  public static YDrive yDrive;
  public static SixWheelDrive sixWheelDrive;

  public static Navigate navigate;
  public static DWANavigator dwaNavigator;
  public static AStarNavigator aStarNavigator;



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    // configureButtonBindings();

    
    driveTrain = new DriveTrain(); // Create Object
    sensor = new Sensor();




    differance = new Differance();
    xDrive = new XDrive();
    mecanum = new Mecanum();
    theStack = new TheStack();
    yDrive = new YDrive();
    sixWheelDrive = new SixWheelDrive();

    
    navigate = new Navigate();
    dwaNavigator = new DWANavigator();
    aStarNavigator = new AStarNavigator();

  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new Autocommand();
  }


public class navigate {
}
}
