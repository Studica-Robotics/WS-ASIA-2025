package frc.robot.subsystems;

// import com.studica.frc.Servo;
import com.studica.frc.TitanQuad;

// import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalOutput;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;


import frc.robot.DriveType_ModuleC.Differance;
import frc.robot.DriveType_ModuleC.SixWheelDrive;
import frc.robot.DriveType_ModuleC.TheStack;
// import frc.robot.DriveType_ModuleC.TheStack_MyBot;
import frc.robot.DriveType_ModuleC.YDrive;
import frc.robot.DriveType_ModuleC.XDrive;

// import java.util.Map;

public class DriveTrain extends SubsystemBase
{
    private final DigitalOutput LedRun;
    private final DigitalOutput LedStop;
    /**
     * Motors
     */
    private final TitanQuad motorLeft;
  private final TitanQuad motorRight;

  //   private final TitanQuad motorLeft_S;
  // private final TitanQuad motorRight_S;

  
  private final TitanQuad motorLeft_front;
  private final TitanQuad motorRight_front;
  private final TitanQuad motorLeft_back;
  private final TitanQuad motorRight_back;
  
  private final TitanQuad motorLeft_front_S;
  private final TitanQuad motorRight_front_S;
  private final TitanQuad motorLeft_back_S;
  private final TitanQuad motorRight_back_S;
  
  private final TitanQuad motorLeft_front_Six;
  private final TitanQuad motorRight_front_Six;
  private final TitanQuad motorLeft_back_Six;
  private final TitanQuad motorRight_back_Six;

  private final TitanQuad motorFront_t;
  private final TitanQuad motorLeft_t;
  private final TitanQuad motorRight_t;

  // Constructor
  public DriveTrain() {
    /**
     * Motors
     */
    motorLeft = new TitanQuad(Differance.TITAN_ID, Differance.MOTOR_LEFT);
    motorRight = new TitanQuad(Differance.TITAN_ID, Differance.MOTOR_RIGHT);

    // motorLeft_S = new TitanQuad(Differance.TITAN_ID, TheStack_MyBot.MOTOR_LEFT);
    // motorRight_S = new TitanQuad(Differance.TITAN_ID, TheStack_MyBot.MOTOR_RIGHT);

    motorFront_t = new TitanQuad(YDrive.TITAN_ID, YDrive.MOTOR_FRONT);
    motorLeft_t = new TitanQuad(YDrive.TITAN_ID, YDrive.MOTOR_LEFT);
    motorRight_t = new TitanQuad(YDrive.TITAN_ID, YDrive.MOTOR_RIGHT);

    motorLeft_front = new TitanQuad(XDrive.TITAN_ID, XDrive.MOTOR_LEFT_FRONT);
    motorRight_front = new TitanQuad(XDrive.TITAN_ID, XDrive.MOTOR_RIGHT_FRONT);
    motorLeft_back = new TitanQuad(XDrive.TITAN_ID, XDrive.MOTOR_LEFT_BACK);
    motorRight_back = new TitanQuad(XDrive.TITAN_ID, XDrive.MOTOR_RIGHT_BACK);

    motorLeft_front_S = new TitanQuad(TheStack.TITAN_ID, TheStack.MOTOR_LEFT_FRONT);
    motorRight_front_S = new TitanQuad(TheStack.TITAN_ID, TheStack.MOTOR_RIGHT_FRONT);
    motorLeft_back_S = new TitanQuad(TheStack.TITAN_ID, TheStack.MOTOR_LEFT_BACK);
    motorRight_back_S = new TitanQuad(TheStack.TITAN_ID, TheStack.MOTOR_RIGHT_BACK);

    motorLeft_front_Six = new TitanQuad(SixWheelDrive.TITAN_ID, SixWheelDrive.MOTOR_LEFT_FRONT);
    motorRight_front_Six = new TitanQuad(SixWheelDrive.TITAN_ID, SixWheelDrive.MOTOR_RIGHT_FRONT);
    motorLeft_back_Six = new TitanQuad(SixWheelDrive.TITAN_ID, SixWheelDrive.MOTOR_LEFT_BACK);
    motorRight_back_Six = new TitanQuad(SixWheelDrive.TITAN_ID, SixWheelDrive.MOTOR_RIGHT_BACK);

    LedRun = new DigitalOutput(Constants.RUNNING_LED);
    LedStop = new DigitalOutput(Constants.STOPPED_LED);

    motorLeft.setInverted(false);
    motorRight.setInverted(true);

    // motorLeft_S.setInverted(false);
    // motorRight_S.setInverted(true);

    motorLeft_front.setInverted(false);
    motorRight_front.setInverted(true);
    motorLeft_back.setInverted(false);
    motorRight_back.setInverted(true);

    motorLeft_front_S.setInverted(false);
    motorRight_front_S.setInverted(true);
    motorLeft_back_S.setInverted(false);
    motorRight_back_S.setInverted(true);

    motorLeft_front_Six.setInverted(false);
    motorRight_front_Six.setInverted(true);
    motorLeft_back_Six.setInverted(false);
    motorRight_back_Six.setInverted(true);

  }

  /**
   * 
   * @param on
   */
  public void Runningred(final boolean on) {
    LedStop.set(on);
  }

  /**
   * 
   * @param on  
   */
  public void Runninggreen(final boolean on) {
    LedRun.set(on);
  }

  /**
   * Two Drive Motor
   * 
   * @param motorkleftSpeed
   * @param motorrightSpeed
   */

  public void setDriveMotorSpeeds(final double motorkleftSpeed, final double motorrightSpeed) {
    motorLeft.set(motorkleftSpeed);
    motorRight.set(motorrightSpeed);
  }


  // public void TheStack_MyBotMotorControl(final double z, final double y){
  //   motorLeft_S.set((y + z));
  //   motorRight_S.set((y - z));
  // }

  /**
   * Controls the two wheel robot
   * 
   * @param x - movement in the x axis
   * @param y - movement in the y axis
   */

  // หน้าหุ่นยนต์แกน Y
  public void twoWheelMotorControl(final double y, final double x) {
    motorLeft.set((y + x));
    motorRight.set((y - x));
  }

  /**
   * 
   * Triangle
   */

  public void setTriangleMotorSpeeds(double motorfrontSpeed ,double motorleftSpeed, double motorrightSpeed) {
    motorFront_t.set(motorfrontSpeed);
    motorLeft_t.set(motorleftSpeed);
    motorRight_t.set(motorrightSpeed);
  }

  /**
   * Controls the two wheel robot
   * 
   * @param x - movement in the x axis
   * @param y - movement in the y axis
   */

  // หน้าหุ่นยนต์แกน Y
  public void TriangleMotorControl(double x, double y, double z) {

    double rightSpeed = ((x / 3) - (y / Math.sqrt(3)) + z) * Math.sqrt(3);
    double leftSpeed = ((x / 3) + (y / Math.sqrt(3)) + z) * Math.sqrt(3);
    double backSpeed = (-3 * x / 3) + z;

    double max = Math.abs(rightSpeed);
    if (Math.abs(leftSpeed) > max) max = Math.abs(leftSpeed);
    if (Math.abs(backSpeed) > max) max = Math.abs(backSpeed);

    if (max > 1)
    {
        rightSpeed /= max;
        leftSpeed /= max;
        backSpeed /= max;
    }

    motorLeft_t.set(leftSpeed);
    motorRight_t.set(rightSpeed);
    // motorLeft_t.set(rightSpeed);
    // motorRight_t.set(leftSpeed);
    motorFront_t.set(backSpeed);
    }
  

  /**
   * THE STACK
   * 
   * @param motorkleftSpeed
   * @param motorrightSpeed
   */

  public void setDriveMotorSpeeds_TheStack(final double motorkleft_frontSpeed, final double motorright_frontSpeed,
      final double motorkleft_backSpeed, final double motorright_backSpeed) {
    motorLeft_front_S.set(motorkleft_frontSpeed);
    motorRight_front_S.set(motorright_frontSpeed);
    motorLeft_back_S.set(motorkleft_backSpeed);
    motorRight_back_S.set(motorright_backSpeed);
  }

  /**
   * Controls the four wheel robot
   * 
   * @param x - movement in the x axis
   * @param y - movement in the y axis
   */

  // หน้าหุ่นยนต์แกน Y
  public void FourWheelMotorControl(final double y, final double z) {
    motorLeft_front_S.set(y + z);
    motorRight_front_S.set(y - z);
    motorLeft_back_S.set(y + z);
    motorRight_back_S.set(y - z);
  }

  /**
   * Controls the four wheel robot
   * 
   * @param x - movement in the x axis
   * @param y - movement in the y axis
   */

  // หน้าหุ่นยนต์แกน Y
  public void SixWheelMotorControl(final double y, final double z) {
    motorLeft_front_Six.set(y + z);
    motorRight_front_Six.set(y - z);
    motorLeft_back_Six.set(y + z);
    motorRight_back_Six.set(y - z);
  }

  /**
   * XDrive Motor
   * 
   * @param motorkleftSpeed
   * @param motorrightSpeed
   */

  public void XDriveMotorSpeeds(final double motorkleftSpeed, final double motorrightSpeed) {
    motorLeft_front.set(motorkleftSpeed);
    motorRight_front.set(motorrightSpeed);
    motorLeft_back.set(motorkleftSpeed);
    motorRight_back.set(motorrightSpeed);
  }

  public void XDriveMotorControl(final double x, final double y, final double z) {
    // คำนวณค่า denominator เพื่อป้องกันความเร็วเกิน
    final double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(z), 1.0);

        // กำหนดความเร็วให้มอเตอร์แต่ละตัว
        motorLeft_front.set((y + x + z) / denominator);
        motorRight_front.set((y - x - z) / denominator);
        motorLeft_back.set((y - x + z) / denominator);
        motorRight_back.set((y + x - z) / denominator);
    }
      

    @Override
    public void periodic()
    {
    }

}