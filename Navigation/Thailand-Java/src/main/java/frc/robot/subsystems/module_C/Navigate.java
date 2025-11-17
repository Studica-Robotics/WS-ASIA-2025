/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.module_C;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.RobotContainer;
import frc.robot.DriveType_ModuleC.Mecanum;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Sensor;

public class Navigate extends SubsystemBase {

  private static final Sensor sensor = RobotContainer.sensor;
  private static final DriveTrain driveTrain = RobotContainer.driveTrain;

  //////////////////////////////////////////////////////////////////////////////////////////

  /**
   * ตั้งค่าตรงนี้ ห้ามลืมเด็ดขาด
   * 
   * @return
   */

  public static String TypeDrive() {
    final String Drive = "Mecanum";
    // final String Drive = "TheStack_MyBot";
    return Drive;
  }

  // private static final Differance driveType = RobotContainer.differance;
  // private static final Differance_2 driveType = RobotContainer.differance_2;
  // private static final XDrive driveType = RobotContainer.xDrive;
  private static final Mecanum driveType = RobotContainer.mecanum;
  // private static final TheStack driveType = RobotContainer.theStack;
  // private static final TheStack_MyBot driveType = RobotContainer.theStack_MyBot;
  // private static final Triangle driveType = RobotContainer.triangle;
  // private static final SixWheelDrive driveType = RobotContainer.sixWheelDrive;

  // XDrive
  // Differance
  // Mecanum
  // TheStack
  // Triangle
  // SixWheelDrive

  ////////////////////////////////////////////////////////////////////////////////////////////

  private final int FRONT_MIN_ANGLE_LEFT = driveType.FRONT_MIN_ANGLE_LEFT;
  private final int FRONT_MAX_ANGLE_LEFT = driveType.FRONT_MAX_ANGLE_LEFT;
  private final int FRONT_MIN_ANGLE_RIGHT = driveType.FRONT_MIN_ANGLE_RIGHT;
  private final int FRONT_MAX_ANGLE_RIGHT = driveType.FRONT_MAX_ANGLE_RIGHT;

  private final int LEFT_MIN_ANGLE_LEFT = driveType.LEFT_MIN_ANGLE_LEFT;
  private final int LEFT_MAX_ANGLE_LEFT = driveType.LEFT_MAX_ANGLE_LEFT;
  private final int LEFT_MIN_ANGLE_RIGHT = driveType.LEFT_MIN_ANGLE_RIGHT;
  private final int LEFT_MAX_ANGLE_RIGHT = driveType.LEFT_MAX_ANGLE_RIGHT;

  private final int RIGHT_MIN_ANGLE_LEFT = driveType.RIGHT_MIN_ANGLE_LEFT;
  private final int RIGHT_MAX_ANGLE_LEFT = driveType.RIGHT_MAX_ANGLE_LEFT;
  private final int RIGHT_MIN_ANGLE_RIGHT = driveType.RIGHT_MIN_ANGLE_RIGHT;
  private final int RIGHT_MAX_ANGLE_RIGHT = driveType.RIGHT_MAX_ANGLE_RIGHT;

  private final int BACK_PROTECT_MIN_ANGLE_LEFT = driveType.BACK_PROTECT_MIN_ANGLE_LEFT;
  private final int BACK_PROTECT_MAX_ANGLE_LEFT = driveType.BACK_PROTECT_MAX_ANGLE_LEFT;
  private final int BACK_PROTECT_MIN_ANGLE_RIGHT = driveType.BACK_PROTECT_MIN_ANGLE_RIGHT;
  private final int BACK_PROTECT_MAX_ANGLE_RIGHT = driveType.BACK_PROTECT_MAX_ANGLE_RIGHT;

  private final int LEFT_PROTECT_MIN_ANGLE_LEFT = driveType.LEFT_PROTECT_MIN_ANGLE_LEFT;
  private final int LEFT_PROTECT_MAX_ANGLE_LEFT = driveType.LEFT_PROTECT_MAX_ANGLE_LEFT;
  private final int LEFT_PROTECT_MIN_ANGLE_RIGHT = driveType.LEFT_PROTECT_MIN_ANGLE_RIGHT;
  private final int LEFT_PROTECT_MAX_ANGLE_RIGHT = driveType.LEFT_PROTECT_MAX_ANGLE_RIGHT;
  
  private final int RIGHT_PROTECT_MIN_ANGLE_LEFT = driveType.RIGHT_PROTECT_MIN_ANGLE_LEFT;
  private final int RIGHT_PROTECT_MAX_ANGLE_LEFT = driveType.RIGHT_PROTECT_MAX_ANGLE_LEFT;
  private final int RIGHT_PROTECT_MIN_ANGLE_RIGHT = driveType.RIGHT_PROTECT_MIN_ANGLE_RIGHT;
  private final int RIGHT_PROTECT_MAX_ANGLE_RIGHT = driveType.RIGHT_PROTECT_MAX_ANGLE_RIGHT;
  
  private final int LEFT_PROTECT_MIN_ANGLE_FRONT = driveType.LEFT_PROTECT_MIN_ANGLE_FRONT;
  private final int LEFT_PROTECT_MAX_ANGLE_FRONT = driveType.LEFT_PROTECT_MAX_ANGLE_FRONT;

  private final int RIGHT_PROTECT_MIN_ANGLE_FRONT = driveType.RIGHT_PROTECT_MIN_ANGLE_FRONT;
  private final int RIGHT_PROTECT_MAX_ANGLE_FRONT = driveType.RIGHT_PROTECT_MAX_ANGLE_FRONT;

  private final int block_distance_Front = driveType.block_distance_Front;
  private final int block_distance_Left = driveType.block_distance_Left;
  private final int block_distance_Right = driveType.block_distance_Right;

  private final double front_protech = driveType.front_protech;
  private final double back_protech = driveType.back_protech;
  private final double left_protech = driveType.left_protech;
  private final double right_protech = driveType.right_protech;
  private final double left_back_protech = driveType.left_back_protech;
  private final double right_back_protech = driveType.right_back_protech;
  private final double left_front_protech = driveType.left_front_protech;
  private final double right_front_protech = driveType.right_front_protech;

  double currentTime;

  public final double speed_x = driveType.speed_x;
  public final double speed_y = driveType.speed_y;
  public final double speed_z = driveType.speed_z;
  private final double speed_z_state = driveType.speed_z_state;
  private final double speed_y_protec = driveType.speed_y_protec;
  private final double error_x_protec = driveType.error_x_protec;
  private final double error_z_protec = driveType.error_z_protec;
  private final double error_z_middle = driveType.error_z_middle;
  private final double determine_protec = driveType.determine_protec;

  private final double DEFAULT_MAX_DISTANCE = 9999.0;

  private boolean rotation = false;
  private boolean rotation2 = false;
  private boolean protection = false;
  private int block_Count = 0;
  private int block_Count_Protect = 0;
  private int count = 0;

  private int count_start = 0;

  private int turnDurationCounter = 0;
  private boolean isTurning = false;

  private RobotState currentState = RobotState.MIDDLE; // เริ่มต้นที่ MIDDLE

  private int r;
  private int l;

  private final static ShuffleboardTab tab = Shuffleboard.getTab("Navigate");
  
  // private final NetworkTableEntry Go = tab.add("Go", false).getEntry();
  
  private final NetworkTableEntry swStart = tab.add("Start_SW",false).withPosition(0,0).getEntry();
  private final NetworkTableEntry SwStop = tab.add("Stop_SW",false).withPosition(1,0).getEntry();
  private final NetworkTableEntry SwReset = tab.add("SwReset", false).withPosition(0,1).getEntry();
  private final NetworkTableEntry SWStop = tab.add("SWStop", false).withPosition(1,1).getEntry();
  
  private final NetworkTableEntry block_boolean_left = tab.add("is_Block_Left", false).withPosition(0,4).getEntry();
  private final NetworkTableEntry Angle_left_Min = tab.add("Ang_left_Min", 0).withPosition(1,4).getEntry();
  private final NetworkTableEntry Distance_left_Min = tab
  .add(LEFT_MIN_ANGLE_LEFT + " to " + LEFT_MAX_ANGLE_RIGHT + " Distance_left", 0).withSize(2, 1).withPosition(2,4).getEntry();
  
  private final NetworkTableEntry block_boolean_front = tab.add("is_Block_Front", false).withPosition(5,4).getEntry();
  private final NetworkTableEntry Angle_front_Min = tab.add("Ang_front_Min", 0).withPosition(6,4).getEntry();
  private final NetworkTableEntry Distance_front_Min = tab
  .add(FRONT_MIN_ANGLE_LEFT + " to " + FRONT_MAX_ANGLE_RIGHT + " Distance_front", 0).withSize(2, 1).withPosition(7,4).getEntry();
  
  
  private final NetworkTableEntry block_boolean_right = tab.add("is_Block_Right", false).withPosition(9,4).getEntry();
  private final NetworkTableEntry Angle_right_Min = tab.add("Ang_right_Min", 0).withPosition(10,4).getEntry();
  private final NetworkTableEntry Distance_right_Min = tab
  .add(RIGHT_MIN_ANGLE_LEFT + " to " + RIGHT_MAX_ANGLE_RIGHT + " Distance_right", 0).withSize(2, 1).withPosition(11,4).getEntry();
  
  
  private final NetworkTableEntry Block_Distance_left_Min_Protect = tab.add("Block_left_Back", false).withPosition(0,5).getEntry();
  private final NetworkTableEntry Angle_left_Min_Protect = tab.add("Ang_left_Min_Protect", 0).withPosition(1,5).getEntry();
  private final NetworkTableEntry Distance_left_Min_Protect = tab
  .add(LEFT_PROTECT_MIN_ANGLE_LEFT + " to " + LEFT_PROTECT_MAX_ANGLE_RIGHT + " Distance_Left_Back_Protect", 0)
  .withSize(2, 1).withPosition(2,5).getEntry();
  
  private final NetworkTableEntry Block_Distance_back_Min_Protect = tab.add("Block_back", false).withPosition(5,5).getEntry();
  private final NetworkTableEntry Angle_back_Min_Protect = tab.add("Ang_Back_Min_Protect", 0).withPosition(6,5).getEntry();
  private final NetworkTableEntry Distance_back_Min_Protect = tab  
  .add(BACK_PROTECT_MIN_ANGLE_LEFT + " to " + BACK_PROTECT_MAX_ANGLE_RIGHT + " Distance_Back_Protect", 0)
  .withSize(2, 1).withPosition(7,5).getEntry();
  

  private final NetworkTableEntry Block_Distance_right_Min_Protect = tab.add("Block_right_Back", false).withPosition(9,5).getEntry();
  private final NetworkTableEntry Angle_right_Min_Protect = tab.add("Ang_right_Min_Protect", 0).withPosition(10,5).getEntry();
  private final NetworkTableEntry Distance_right_Min_Protect = tab
  .add(RIGHT_PROTECT_MIN_ANGLE_LEFT + " to " + RIGHT_PROTECT_MAX_ANGLE_RIGHT + " Distance_right_Back_Protect", 0)
  .withSize(2, 1).withPosition(11,5).getEntry();


  private final NetworkTableEntry Block_Distance_left_Min_Protect_front = tab.add("Block_Left_Front", false).withPosition(0,3).getEntry();
  private final NetworkTableEntry Angle_left_Min_Protect_front = tab.add("Ang_left_Min_Protect_front", 0).withPosition(1,3).getEntry();
  private final NetworkTableEntry Distance_left_Min_Protect_front = tab
  .add(LEFT_PROTECT_MIN_ANGLE_FRONT + " to " + LEFT_PROTECT_MAX_ANGLE_FRONT + " Distance_left_Protect_front", 0)
  .withSize(2, 1).withPosition(2,3).getEntry();
  

  private final NetworkTableEntry Block_Distance_right_Min_Protect_front = tab.add("Block_right_front", false).withPosition(9,3).getEntry();
  private final NetworkTableEntry Angle_right_Min_Protect_front = tab.add("Ang_right_Min_Protect_front", 0).withPosition(10,3).getEntry();
  private final NetworkTableEntry Distance_right_Min_Protect_front = tab
  .add(RIGHT_PROTECT_MIN_ANGLE_FRONT + " to " + RIGHT_PROTECT_MAX_ANGLE_FRONT + " Distance_right_Protect_front", 0)
  .withSize(2, 1).withPosition(11,3).getEntry();
  
  private final NetworkTableEntry State_Start_Number = tab.add("State_Start_Number", 0).withPosition(2,0).getEntry();
  private final NetworkTableEntry State_Start_count = tab.add("State_Start_count", 0).withPosition(2,1).getEntry();
  private final NetworkTableEntry Start_Choose_way = tab.add("Start_Choose_way", false).withPosition(3,0).getEntry();


  private final NetworkTableEntry Turn_count = tab.add("Turn_Counts", 0).withPosition(5,1).getEntry();
  private final NetworkTableEntry Turn_count_Protect = tab.add("Turn_Counts_Protect", 0).withPosition(6,1).getEntry();
  
  private final NetworkTableEntry Cal_X = tab.add("Cal_X", 0).withPosition(5,2).getEntry();
  private final NetworkTableEntry Cal_Y = tab.add("Cal_Y", 0).withPosition(6,2).getEntry();
  private final NetworkTableEntry Cal_Z = tab.add("Cal_Z", 0).withPosition(7,2).getEntry();
  
  private final NetworkTableEntry R = tab.add("r", 0).withPosition(7,3).getEntry();
  private final NetworkTableEntry L = tab.add("l", 0).withPosition(5,3).getEntry();
  private final NetworkTableEntry Middle = tab.add("Middle", 0).withPosition(6,3).getEntry();
  
  public final static NetworkTableEntry TypeRobot = tab.add("DriveType", "").withPosition(5,0).getEntry();
  private final NetworkTableEntry state = tab.add("state", "").withSize(2, 1).withPosition(6,0).getEntry();
  
  private final NetworkTableEntry YAW = tab.add("Yaw", 0).withPosition(7,1).getEntry();
  private NetworkTableEntry navx_getPitch = tab.add("navx_getPitch",0).withPosition(8,1).getEntry();
  private NetworkTableEntry navx_getTempC = tab.add("navx_getTempC",0).withPosition(9,1).getEntry();


  public Navigate() {

  }

  // กำหนด state เป็น enum
  public enum RobotState {
    PROTECTION, MIDDLE 
  }

  public void reset(){

    boolean Emergency = sensor.getSwitchEmergency();
    if (Emergency){

      rotation = false;
      rotation2 = false;
      protection = false;
      block_Count = 0;
      block_Count_Protect = 0;
      turnDurationCounter = 0;
      isTurning = false;

    }

  }

  /**
   * 
   * CAL SPEED
   * 
   */

  public double[] getMovementValues() {
    double cal_z = 0.0;
    double cal_y = speed_y;
    TypeRobot.setString(TypeDrive());

    final boolean front_block = is_Block()[0];
    final boolean left_block = is_Block()[1];
    final boolean right_block = is_Block()[2];

    final double front = findFrontMinDistance()[0];
    final double left = findLeftMinDistance()[0];
    final double right = findRightMinDistance()[0];
    final double back = findBackMinDistance_Protect()[0];
    final double left_back = findLeftMinDistance_Protect()[0];
    final double right_back = findRightMinDistance_Protect()[0];
    final double left_front = findLeftMinDistance_Protect_Front()[0];
    final double right_front = findRightMinDistance_Protect_Front()[0];

    final double Left_error = Math.max(0, left_protech - left);
    final double Right_error = Math.max(0, right_protech - right);
    final double Left_back_error = Math.max(0, left_back_protech - left_back);
    final double Right_back_error = Math.max(0, right_back_protech - right_back);
    final double Left_front_error = Math.max(0, left_front_protech - left_front);
    final double Right_front_error = Math.max(0, right_front_protech - right_front);
    final double Front_error = Math.max(-speed_y, front_protech - front);

    Block_Distance_back_Min_Protect.setBoolean(back <= back_protech);
    Block_Distance_right_Min_Protect.setBoolean(right_back <= right_back_protech);
    Block_Distance_left_Min_Protect.setBoolean(left_back <= left_back_protech);
    Block_Distance_right_Min_Protect_front.setBoolean(right_front <= right_front_protech);
    Block_Distance_left_Min_Protect_front.setBoolean(left_front <= left_front_protech);

    

    // กำหนด state ตามเงื่อนไข
    determineState();
    // reset();   

    // switch-case ตาม state
    switch (currentState) {
    case PROTECTION:
      cal_z = protec()[2];
      cal_y = protec()[1];
      state.setString("Protection");
      determineState();


if (!protection) {
    block_Count_Protect++;
    Turn_count_Protect.setDouble(block_Count_Protect);
    protection = true; // ตั้งแฟล็ก
}

// เงื่อนไขใหม่: เช็คว่า block_Count_Protect หาร 5 ลงตัวหรือไม่
else if ((block_Count_Protect % 5 == 0 && block_Count_Protect > 0) || isTurning) {
    if (!isTurning) {
        isTurning = true;
        turnDurationCounter++; // เพิ่มตัวนับทิศทางการหมุน
    }

    // กำหนดทิศทางการหมุนตามลำดับ (คู่ = ซ้าย, คี่ = ขวา)
    if (turnDurationCounter % 2 == 1) {
        cal_z = -(speed_z_state);
        cal_y = 0.0;
        state.setString("Protect%5==0_Turning_Left_");
    } else {
        cal_z = speed_z_state;
        cal_y = 0.0;
        state.setString("Protect%5==0_Turning_Right_");
    }

    turnDurationCounter++;

    if (turnDurationCounter >= 100) {
        isTurning = false;
        rotation = true;
        r = 1;
        R.setDouble(r);
        Turn_count_Protect.setDouble(block_Count_Protect);
    }
}

      else if (front_block && left_block && right_block) {
        cal_z = -(speed_z_state);
        cal_y = 0.0;
        rotation = true;
        r = 1;
        R.setDouble(r);
        state.setString("All_block_turnning_Left");
      }

      break;


      case MIDDLE:
      boolean start = sensor.getSwitchStart();
      if(start){
        cal_y = 0.0;
        cal_z = 0.0;
      }
      else if(!start){
        
        cal_z = protec()[0];
        cal_y = speed_y;
      }
      determineState();



      
      if((front_block && left_block && right_block) &&  State_Start() == 0 ){
        rotation2 = true;
        while (count <= 500){
          
          cal_y = 0;
          cal_z = 0.4; 
          
          // ต้องใส่เงื่อนไขในการหมุนตัว 
          
          
          count ++; 
          if (count > 500) {
            count = 0;
            break;
          }
        }
      }
      
      if( rotation2 = true &&  State_Start() ==  2){
        while (count <= 500){
          
          cal_y = 0;
          cal_z = -0.4; 
          rotation2 = false;
          count ++; 
          if (count > 500) {
            count = 0;
            rotation2 = false;
            break;
          }
        }
      }
      
      else if((front_block && left_block && right_block) &&  State_Start() == 1){
        while (count <= 500){
          
          cal_y = 0;
          cal_z = 0.4;
          count ++; 
          if (count > 500) {
            count = 0;
            break;
          }
        }
      }

      else if((front_block && left_block && right_block) &&  State_Start() == 2 ){
        rotation2 = true;

        while (count <= 500){
          
          cal_y = 0;
          cal_z = -0.4;
          count ++; 
          if (count > 500) {
            count = 0;
            break;
          }
        }
      }
      
      if( front_block){
        cal_y = -speed_y_protec;  
      }
      
      if (front_block == false && rotation == false && isTurning == false) {
        if(start){
          cal_y = 0.0;
          cal_z = 0.0;
        }
        else{
          cal_z = cal_speed();
          cal_y = speed_y;
        }
        Cal_Z.setNumber(cal_z);
        Cal_Y.setNumber(cal_y);
        state.setString("Middle");
      }

      else if ((left_back <= left_back_protech || right_back <= right_back_protech) && !front_block )  {
        cal_y = speed_y_protec;
        cal_z = 0;
      }
  
      
          
       else if (front_block == false && rotation == true) {
        block_Count++;
        rotation = false;
      } 
      
    
      Cal_Y.setNumber(cal_y);
      Cal_Z.setNumber(cal_z);

      protection = false;
      break;

  
    }

    Cal_Z.setDouble(cal_z);
    Cal_Y.setDouble(cal_y);
    Turn_count.setDouble(block_Count);

    return new double[] { cal_z, cal_y };
  }


  private void determineState() {
    if ((protec()[2] < -determine_protec && protec()[2] > determine_protec) || protec()[1] < 0
        || (protec()[0] < -determine_protec || protec()[0] > determine_protec)) {
      currentState = RobotState.PROTECTION;
    }

    else {
      currentState = RobotState.MIDDLE;
    }

  }

  /**
   * PROTECTION ROBOT
   * 
   * @return
   */

  public double[] protec() {
    double cal_y = 0.0;
    double cal_x = 0.0;
    double cal_z = 0.0;
    final double front = findFrontMinDistance()[0];
    final double left = findLeftMinDistance()[0];
    final double right = findRightMinDistance()[0];
    final double back = findBackMinDistance_Protect()[0];
    final double left_back = findLeftMinDistance_Protect()[0];
    final double right_back = findRightMinDistance_Protect()[0];
    final double left_front = findLeftMinDistance_Protect_Front()[0];
    final double right_front = findRightMinDistance_Protect_Front()[0];

    final double Left_error = Math.max(0, left_protech - left);
    final double Right_error = Math.max(0, right_protech - right);
    final double Left_back_error = Math.max(0, left_back_protech - left_back);
    final double Right_back_error = Math.max(0, right_back_protech - right_back);
    final double Left_front_error = Math.max(0, left_front_protech - left_front);
    final double Right_front_error = Math.max(0, right_front_protech - right_front);
    final double Front_error = Math.max(-speed_y, front_protech - front);

    Block_Distance_back_Min_Protect.setBoolean(back <= back_protech);
    Block_Distance_right_Min_Protect.setBoolean(right_back <= right_back_protech);
    Block_Distance_left_Min_Protect.setBoolean(left_back <= left_back_protech);
    Block_Distance_right_Min_Protect_front.setBoolean(right_front <= right_front_protech);
    Block_Distance_left_Min_Protect_front.setBoolean(left_front <= left_front_protech);



    cal_z = (MathUtil.clamp((Left_error - Right_error) * error_z_protec, -speed_z, speed_z));

    if(front <= front_protech){
      cal_y = -(MathUtil.clamp(Front_error, -(speed_y_protec), (speed_y_protec)));
    }

    if ((back <= back_protech || left_back <= left_back_protech || right_back <= right_back_protech) && (TypeDrive() == "Differance" || TypeDrive() == "SixWheelDrive") ||  TypeDrive() == "TheStack_MyBot")  {
      cal_y = speed_y_protec;
      cal_z = 0;
    }


    else if (back <= back_protech) {
      cal_y = speed_y_protec;
    }


    else if ((front >= 4000 || left_front <= left_front_protech || right_front <= right_front_protech) && (TypeDrive() == "Differance" || TypeDrive() == "SixWheelDrive" || TypeDrive() == "TheStack"  ) ) {
      cal_y = -speed_y_protec;
    }

   

    else {
      cal_y = -(MathUtil.clamp(Front_error, -(speed_y_protec), (speed_y_protec)));
    }

    Cal_X.setDouble(cal_x);
    Cal_Y.setDouble(cal_y);
    Cal_Z.setDouble(cal_z);

    return new double[] { cal_x, cal_y, cal_z };
  }

  public double middle() {
    final double leftDist = findLeftMinDistance()[0];
    final double rightDist = findRightMinDistance()[0];
    final double frontDist = findFrontMinDistance()[0];
    double error = 0;

    final double error2 = 0;

    final double left_back = findLeftMinDistance_Protect()[0];
    final double right_back = findRightMinDistance_Protect()[0];

    final double State_Start = State_Start();


    if((left_back <= left_back_protech || right_back <= right_back_protech) && (TypeDrive() == "Differance" || TypeDrive() == "SixWheelDrive" || TypeDrive() == "TheStack" )){
      error = 0;
      Middle.setDouble(error);
    }

    else if((State_Start == 0) && (error2 < 100 && error2 > -100)){
      error = error2 + 300; //left
      Start_Choose_way.setBoolean(true);
    }

    else if((State_Start == 0) && (error2 > 1000 || error2 < -1000)){
      error = error2 + 300; //left
      Start_Choose_way.setBoolean(true);
    }
    
    else if((State_Start == 2) && (error2 < 100 && error2 > -100)){
      error = error2 - 300; //right
      Start_Choose_way.setBoolean(true);
    }
    
    else if((State_Start == 2) && (error2 > 1000 || error2 < -1000)){
      error = error2 - 300; //right
      Start_Choose_way.setBoolean(true);
    }
    
    
    else{
      error = error2 ;
      Start_Choose_way.setBoolean(false);
    }

    Middle.setDouble(error);
    state.setString("middle_middle");
    return error;
  }

  public double cal_speed() {
    final double cal_z = -(MathUtil.clamp((middle() * error_z_middle), -speed_z, speed_z));
    Cal_Z.setDouble(cal_z);
    return cal_z;
  }

  public Double State_Start(){
    boolean start = sensor.getSwitchStart();
    boolean emergency = sensor.getSwitchEmergency();
    boolean stop = sensor.getSwitchStop();
    double state_Start = 0 ;

    if (start) {
      count_start++;
      
    }
    else if(emergency || stop){
      count_start = 0;
    }

    if( count_start >= 50 && count_start < 200){
      state_Start = 1;
    }
    
    else if( count_start >= 200){
      state_Start = 2;
    }
    State_Start_count.setDouble(count_start);
    return  state_Start;
  }
  

  /**
   * Block
   * 
   * @return
   */

  public boolean[] is_Block() {
    final double frontDist = findFrontMinDistance()[0];
    final boolean block_front = frontDist < block_distance_Front || frontDist >= 3000.0;
    block_boolean_front.setBoolean(block_front);

    final double LeftDist = findLeftMinDistance()[0];
    final boolean block_left = LeftDist < block_distance_Left || LeftDist >= 3000.0;
    block_boolean_left.setBoolean(block_left);

    final double RightDist = findRightMinDistance()[0];
    final boolean block_right = RightDist < block_distance_Right || RightDist >= 3000.0;
    block_boolean_right.setBoolean(block_right);

    return new boolean[] { block_front, block_left, block_right };
  }



  /*
   * 
   * 
   * LIDAR
   * 
   * 
   * 
   */
  // ** Front */
  public double[] findFrontMinDistance() {
    double minDistance = DEFAULT_MAX_DISTANCE;
    double minAngle = FRONT_MIN_ANGLE_LEFT;

    for (int angle = FRONT_MIN_ANGLE_LEFT; angle <= FRONT_MAX_ANGLE_LEFT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);

      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                 // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;
      }
    }

    for (int angle = FRONT_MIN_ANGLE_RIGHT; angle <= FRONT_MAX_ANGLE_RIGHT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);

      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                 // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;
      }
    }

    Distance_front_Min.setDouble(minDistance);
    Angle_front_Min.setDouble(minAngle);

    return new double[] { minDistance, minAngle };
  }

  // ** RIGHT */
  public double[] findRightMinDistance() {
    double minDistance = DEFAULT_MAX_DISTANCE;
    double minAngle = RIGHT_MIN_ANGLE_LEFT;

    for (int angle = RIGHT_MIN_ANGLE_LEFT; angle <= RIGHT_MAX_ANGLE_LEFT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);

      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                 // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;
      }
    }

    for (int angle = RIGHT_MIN_ANGLE_RIGHT; angle <= RIGHT_MAX_ANGLE_RIGHT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);

      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                 // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;

      }
    }

    Distance_right_Min.setDouble(minDistance);
    Angle_right_Min.setDouble(minAngle);

    return new double[] { minDistance, minAngle };
  }

  // ** LEFT */

  public double[] findLeftMinDistance() {
    double minDistance = DEFAULT_MAX_DISTANCE;
    double minAngle = LEFT_MIN_ANGLE_LEFT;

    for (int angle = LEFT_MIN_ANGLE_LEFT; angle <= LEFT_MAX_ANGLE_LEFT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);

      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                 // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;
      }
    }

    for (int angle = LEFT_MIN_ANGLE_RIGHT; angle <= LEFT_MAX_ANGLE_RIGHT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);

      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                 // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;
      }
    }

    Distance_left_Min.setDouble(minDistance);
    Angle_left_Min.setDouble(minAngle);

    return new double[] { minDistance, minAngle };
  }

  /*
   * 
   * 
   * LIDAR Protect
   * 
   * 
   * 
   */
  // ** Back Protect */
  public double[] findBackMinDistance_Protect() {
    double minDistance = DEFAULT_MAX_DISTANCE;
    double minAngle = BACK_PROTECT_MIN_ANGLE_LEFT;

    for (int angle = BACK_PROTECT_MIN_ANGLE_LEFT; angle <= BACK_PROTECT_MAX_ANGLE_LEFT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);

      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                 // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;
      }
    }

    for (int angle = BACK_PROTECT_MIN_ANGLE_RIGHT; angle <= BACK_PROTECT_MAX_ANGLE_RIGHT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);

      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                 // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;
      }
    }

    Distance_back_Min_Protect.setDouble(minDistance);
    Angle_back_Min_Protect.setDouble(minAngle);

    return new double[] { minDistance, minAngle };
  }

  // ** RIGHT Protect */
  public double[] findRightMinDistance_Protect() {
    double minDistance = DEFAULT_MAX_DISTANCE;
    double minAngle = RIGHT_PROTECT_MIN_ANGLE_LEFT;

    for (int angle = RIGHT_PROTECT_MIN_ANGLE_LEFT; angle <= RIGHT_PROTECT_MAX_ANGLE_LEFT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);

      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                 // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;

      }
    }

    for (int angle = RIGHT_PROTECT_MIN_ANGLE_RIGHT; angle <= RIGHT_PROTECT_MAX_ANGLE_RIGHT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);

      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                 // เพื่อป้องกันค่าผิดพลาด

        minDistance = distance;
        minAngle = angle;

      }
    }

    Distance_right_Min_Protect.setDouble(minDistance);
    Angle_right_Min_Protect.setDouble(minAngle);

    return new double[] { minDistance, minAngle };
  }

  
  // ** LEFT Protect */
  
  public double[] findLeftMinDistance_Protect() {
    double minDistance = DEFAULT_MAX_DISTANCE;
    double minAngle = LEFT_PROTECT_MIN_ANGLE_LEFT;
    
    for (int angle = LEFT_PROTECT_MIN_ANGLE_LEFT; angle <= LEFT_PROTECT_MAX_ANGLE_LEFT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);
      
      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;
        
      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
        // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;
      }
    }
    
    for (int angle = LEFT_PROTECT_MIN_ANGLE_RIGHT; angle <= LEFT_PROTECT_MAX_ANGLE_RIGHT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);
      
      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;
        
      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
        // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;
        
      }
    }
    
    Distance_left_Min_Protect.setDouble(minDistance);
    Angle_left_Min_Protect.setDouble(minAngle);
    
    return new double[] { minDistance, minAngle };
  }
  
  // ** RIGHT Protect */
  public double[] findRightMinDistance_Protect_Front() {
    double minDistance = DEFAULT_MAX_DISTANCE;
    double minAngle = RIGHT_PROTECT_MIN_ANGLE_FRONT;

    for (int angle = RIGHT_PROTECT_MIN_ANGLE_FRONT; angle <= RIGHT_PROTECT_MAX_ANGLE_FRONT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);

      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                 // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;

      }
    }

    Distance_right_Min_Protect_front.setDouble(minDistance);
    Angle_right_Min_Protect_front.setDouble(minAngle);

    return new double[] { minDistance, minAngle };
  }


  // ** LEFT Protect Front */

  public double[] findLeftMinDistance_Protect_Front() {
    double minDistance = DEFAULT_MAX_DISTANCE;
    double minAngle = LEFT_PROTECT_MIN_ANGLE_FRONT;

    for (int angle = LEFT_PROTECT_MIN_ANGLE_FRONT; angle <= LEFT_PROTECT_MAX_ANGLE_FRONT; angle += 10) {
      final double distance = sensor.getDistionLidar(angle);

      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                 // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;
      }
    }


    Distance_left_Min_Protect_front.setDouble(minDistance);
    Angle_left_Min_Protect_front.setDouble(minAngle);

    return new double[] { minDistance, minAngle };
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getMovementValues();
    YAW.setDouble(sensor.getAngle());
    swStart.setBoolean(sensor.getSwitchStart());
    SwStop.setBoolean(sensor.getSwitchEmergency());
    SWStop.setBoolean(sensor.getSwitchStop());
    SwReset.setBoolean(sensor.getSwitchReset());
    reset();
    State_Start();
    State_Start_Number.setDouble(State_Start());

  }
}
