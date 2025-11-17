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
import frc.robot.DriveType_ModuleC.Differance;
import frc.robot.DriveType_ModuleC.Mecanum;
import frc.robot.DriveType_ModuleC.SixWheelDrive;
import frc.robot.DriveType_ModuleC.TheStack;
import frc.robot.DriveType_ModuleC.XDrive;
import frc.robot.DriveType_ModuleC.YDrive;
import frc.robot.RobotContainer.navigate;
import frc.robot.subsystems.DriveTrain;
// import frc.robot.subsystems.Navigator;
import frc.robot.subsystems.Sensor;

public class DriveTest extends CommandBase {
  /**
   * Creates a new WallFollow.
   */
  private static final DriveTrain drive = RobotContainer.driveTrain;
  private static final Sensor sensor = RobotContainer.sensor; 


  // private static final YDrive DriveType = RobotContainer.yDrive;
  // private static final XDrive DriveType = RobotContainer.xDrive;
  // private static final SixWheelDrive DriveType = RobotContainer.sixWheelDrive;
  // private static final Mecanum DriveType = RobotContainer.mecanum;
  private static final Differance DriveType = RobotContainer.differance;
  // private static final TheStack DriveType = RobotContainer.theStack;



  private double speedY = DriveType.speedY ;
  private double speedZ = DriveType.speedZ ;

  private double setpointDistanceY = DriveType.setpointDistanceY; 


  PIDController pidYAxis;




  int count = 0;
   
  public DriveTest() {

    double epsilonDistanceY = DriveType.epsilonDistanceY;

    pidYAxis = new PIDController(0.0120,0.07,0.0005);
     pidYAxis.setTolerance(epsilonDistanceY);

  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.TriangleMotorControl(0,0,0);
    drive.XDriveMotorControl(0, 0, 0);
    drive.SixWheelMotorControl(0, 0);
    drive.twoWheelMotorControl(0, 0);

  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(sensor.findFrontMinDistance()[0] > 4000){
  
      if(Config.YDrive_Type){
        drive.TriangleMotorControl(
        0.0,
        -speedY * 5,
        0.0
        );
    }

    else if(Config.XDrive_Type){
        drive.XDriveMotorControl(
          0.0, 
          -speedY * 5, 
          0.0
          );
    }

    else if(Config.SixWheelDrive_Type){
      drive.SixWheelMotorControl(
        -speedY * 3, 
        0.0
        );
    }

    else if(Config.MecanumDrive_Type){
      drive.XDriveMotorControl(
        0.0,
        -speedY * 3, 
        0.0
        );
    }

    else if(Config.DifferanceDrive_Type){
      drive.twoWheelMotorControl(
        -speedY * 3, 
        0.0
        );
    }

    else if(Config.TheStackDrive_Type){
      drive.FourWheelMotorControl(
        -speedY * 3, 
        0.0
        );
    }



  }
  
  else if(sensor.findFrontMinDistance()[0] < DriveType.ProtectDistance_Y){

    if(sensor.findFrontMinDistance()[0] > DriveType.ProtectDistance_Y ){

        count ++;

          if(Config.YDrive_Type && count < 100){
            drive.TriangleMotorControl(
            -speedY,
            speedY,
            0.0
            );
        }


        
        else if(Config.XDrive_Type && count < 100){
          drive.XDriveMotorControl(
            speedY * 2,
            speedY,
            0.0
            );
      }



      else if(Config.SixWheelDrive_Type){
        drive.SixWheelMotorControl(
          0.0 , 
          speedZ * 3
          );
    }

      else if(Config.MecanumDrive_Type && count < 100 ){
          drive.XDriveMotorControl(
            speedY,
            speedY,
            0.0
            );
      }

      



      else if(Config.DifferanceDrive_Type && count < 100){
          drive.twoWheelMotorControl(
            speedY , 
            0.0
            );
      }


      else if(Config.TheStackDrive_Type){
        drive.FourWheelMotorControl(
          0.0 , 
          speedZ * 4
          );
    }


    }

    else{
        
        if(Config.YDrive_Type){
            drive.TriangleMotorControl(
            0.0,
            0.0,
            speedZ * 5
            );
        }

        else if(Config.XDrive_Type){
            drive.XDriveMotorControl(
              0.0, 
              0.0, 
              speedZ * 8
              );
        }

        else if(Config.SixWheelDrive_Type){
            drive.SixWheelMotorControl(
              0.0 , 
              speedZ * 3
              );
        }

        else if(Config.MecanumDrive_Type){
            drive.XDriveMotorControl(
              0.0 ,
              0.0 , 
              speedZ * 3
              );
        }

        else if(Config.DifferanceDrive_Type){
            drive.twoWheelMotorControl(
              0.0 , 
              speedZ * 3
              );
        }

        else if(Config.TheStackDrive_Type){
            drive.FourWheelMotorControl(
              0.0 , 
              speedZ * 4
              );
        }

      }
  
  }

  else{

    Test();

  }
  
}

  public void Test(){

    count = 0;

    double speedY_PID = -MathUtil.clamp(pidYAxis.calculate(sensor.findFrontMinDistance()[0], setpointDistanceY), -speedY, speedY);
    


    if(Config.YDrive_Type){
      drive.TriangleMotorControl(
        0.0,
        speedY,
        speedZ
        );
    }

    else if(Config.XDrive_Type){
        drive.XDriveMotorControl(
          0.0, 
          speedY, 
          speedZ
          );
    }

    else if(Config.SixWheelDrive_Type){
        drive.SixWheelMotorControl(
          speedY, 
          speedZ
          );
    }

    else if(Config.MecanumDrive_Type){
        drive.XDriveMotorControl(
          0.0 ,
          speedY, 
          speedZ
          );
    }

    else if(Config.DifferanceDrive_Type){
        drive.twoWheelMotorControl(
          speedY_PID, 
          speedZ
          );
    }

    else if(Config.TheStackDrive_Type){
        drive.FourWheelMotorControl(
          speedY, 
          speedZ
          );
    }
    

  }

  


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.TriangleMotorControl(0,0,0);
    drive.XDriveMotorControl(0, 0, 0);
    drive.SixWheelMotorControl(0, 0);
    drive.twoWheelMotorControl(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
