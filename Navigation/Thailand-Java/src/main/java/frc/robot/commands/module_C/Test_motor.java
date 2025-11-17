/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.module_C;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Sensor;
import frc.robot.subsystems.module_C.AStarNavigator;
import frc.robot.subsystems.module_C.DWANavigator;
import frc.robot.subsystems.module_C.Navigate;

public class Test_motor extends CommandBase {

  private static final DriveTrain drive = RobotContainer.driveTrain;
  private static final Sensor sensor = RobotContainer.sensor;
  private static final Navigate navigate = RobotContainer.navigate;
  private static final  DWANavigator dwaNavigator = RobotContainer.dwaNavigator;
  private static final AStarNavigator aStarNavigator = RobotContainer.aStarNavigator;


  private final double SPEED_INCREMENT = 0.015; // ค่าการเพิ่มความเร็วทีละครั้ง
  private double current_motor_z = 0.0; // ความเร็วปัจจุบันของแกน Z
  private double current_motor_y = 0.0; // ความเร็วปัจจุบันของแกน Y
  private double current_motor_x = 0.0; // ความเร็วปัจจุบันของแกน Y

  private boolean isRotating = false;

  /**
   * Creates a new test_motor.
   */
  public Test_motor() {
    addRequirements(drive, sensor);


  }

  /**
   * ฟังก์ชันสำหรับการปรับความเร็วแบบค่อยๆ
   * @param current ความเร็วปัจจุบัน
   * @param target ความเร็วเป้าหมาย
   * @return 
   */
  private double gradualSpeedAdjust(double current, double target) {
    double difference = target - current;
    
    if (Math.abs(difference) <= SPEED_INCREMENT) {
      return target;
    }
    if (difference > 0) {
      return current + SPEED_INCREMENT; // เพิ่มขึ้น
    } else {
      return current - SPEED_INCREMENT; // ลดลง
    }
  }


  /**
   * ฟังก์ชันหลักสำหรับการควบคุมมอเตอร์แบบค่อยๆ
   * @return array ของความเร็วที่ปรับแล้ว [z_speed, y_speed]
   */
  private double[] getGradualMotorSpeeds() {

    double[] targetSpeeds = dwaNavigator.getMovementValues();
    // double[] targetSpeeds = navigate.getMovementValues();
    // double[] targetSpeeds = aStarNavigator.getMovementValues();
    double target_z = targetSpeeds[0];
    double target_y = targetSpeeds[1];
    // double target_x = navigate.protec()[0];
    double target_x = dwaNavigator.protec()[0];
    // double target_x = aStarNavigator.protec()[0];

    current_motor_z = gradualSpeedAdjust(current_motor_z, target_z);
    current_motor_y = gradualSpeedAdjust(current_motor_y, target_y);
    current_motor_x = gradualSpeedAdjust(current_motor_x, target_x);
  

    return new double[] { current_motor_z, current_motor_y , current_motor_x };
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setDriveMotorSpeeds(0, 0);
    drive.Runninggreen(true);
    drive.Runningred(false);
    isRotating = false;
    current_motor_z = 0.0;
    current_motor_y = 0.0;
    current_motor_x = 0.0;
  }

  @Override
  public void execute() {
    // ดึงค่าความเร็วที่ปรับแบบค่อย ๆ
    // double[] speeds = getGradualMotorSpeeds();
    // double z = speeds[0];
    // double y = speeds[1];
    // double x = speeds[2];

//     double[] gradualSpeeds = getGradualMotorSpeeds();
// // navigate

    double z = 0.1;
    double y = 0.0;
  
    drive.FourWheelMotorControl(z, y);

//   // drive.XDriveMotorSpeeds(z, y); 
// // drive.XDriveMotorControl(z, y,x);
// // drive.setTriangleMotorSpeeds(z, y, -x);
// // drive.TriangleMotorControl(z, y,x);
  
    drive.Runninggreen(true);
    drive.Runningred(false);
  }
 

  // Called once the command ends or is interrupted.
  @Override
  public void end(final boolean interrupted) {
    // หยุดมอเตอร์แบบทันที (ในกรณีฉุกเฉิน)
    drive.setDriveMotorSpeeds(0, 0);
    drive.Runninggreen(false);
    drive.Runningred(true);
    
    // รีเซ็ตค่าความเร็วปัจจุบัน
    current_motor_z = 0.0;
    current_motor_y = 0.0;
    current_motor_x = 0.0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}