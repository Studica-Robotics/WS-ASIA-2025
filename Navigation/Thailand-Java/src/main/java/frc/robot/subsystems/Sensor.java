package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.Lidar;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.DriveType_ModuleC.Differance;
import frc.robot.DriveType_ModuleC.Mecanum;
import frc.robot.DriveType_ModuleC.SixWheelDrive;
import frc.robot.DriveType_ModuleC.TheStack;
import frc.robot.DriveType_ModuleC.XDrive;
import frc.robot.DriveType_ModuleC.YDrive;

public class Sensor extends SubsystemBase {

    // private static final XDrive DriveType = RobotContainer.xDrive;
    // private static final Mecanum DriveType = RobotContainer.mecanum;
    // private static final SixWheelDrive DriveType = RobotContainer.sixWheelDrive;
    private static final Differance DriveType = RobotContainer.differance;
    // private static final TheStack DriveType = RobotContainer.theStack;
    // private static final YDrive DriveType = RobotContainer.yDrive;
    /**
     * Switch 
     */
    private final DigitalInput SW_Start;
    private final DigitalInput SW_Emmer;
    private final DigitalInput SW_Reset;
    private final DigitalInput SW_Stop;

    
    /**
     * IMU
     */
    private final AHRS navx;


    /**
     * Lidar
     */
    private Lidar lidar;
    private Lidar.ScanData scanData;
    public boolean scanning = true;



    /**
     * Shufflboard
     */
    private ShuffleboardTab tab;
    private NetworkTableEntry Sw_Start;
    private NetworkTableEntry Sw_Emmergenzy;
    private NetworkTableEntry Sw_Reset;
    private NetworkTableEntry Sw_Stop;
    
    private NetworkTableEntry Sw_Start_Time;



    private NetworkTableEntry ValueAngle;
    private NetworkTableEntry navx_angle;

    private NetworkTableEntry Lidar_Distance_Front;
    private NetworkTableEntry Distance_front_Min;
    private NetworkTableEntry Angle_front_Min;
    
    private NetworkTableEntry Lidar_Distance_Left;
    private NetworkTableEntry Lidar_Distance_Right;
    
    private NetworkTableEntry Distance_left_Min;
    private NetworkTableEntry Angle_left_Min;

    private NetworkTableEntry Distance_left_Min_Protect;
    private NetworkTableEntry Angle_left_Min_Protect;


    public Sensor() 
    {

        /**
         *  Switch
         */
        SW_Start = new DigitalInput(Constants.START_BUTTON);
        SW_Emmer = new DigitalInput(Constants.EMERGENCY_BUTTON);
        SW_Reset = new DigitalInput(Constants.RESET_BUTTON);
        SW_Stop = new DigitalInput(Constants.STOP_BUTTON);
        
      
        /**
         * IMU
         */
        navx = new AHRS(SPI.Port.kMXP);
        

        /**
         *  Lidar 
         */

        lidar = new Lidar(Lidar.Port.kUSB1); 

        // Configure filters
        // lidar.clusterConfig(50.0f, 5);

        // lidar.kalmanConfig(1e-5f, 1e-1f, 1.0f);
        // lidar.movingAverageConfig(5);
        // lidar.medianConfig(5);
        // lidar.jitterConfig(50.0f);

        // Enable Filter
        // lidar.enableFilter(Lidar.Filter.kCLUSTER, true);
        lidar.enableFilter(Lidar.Filter.kCLUSTER, false);

     

    

        if (Config.Sensor) {
            tab = Shuffleboard.getTab("Sensor");
            Sw_Emmergenzy  = tab.add("Sw_Emmergenzy", false).withPosition(0, 1).getEntry();
            Sw_Start = tab.add("Sw_Start", false).withPosition(1, 0).getEntry();
            Sw_Reset = tab.add("Sw_Reset", false).withPosition(1, 1).getEntry();
            Sw_Stop  = tab.add("Sw_Stop", false).withPosition(1, 2).getEntry();
            
            Sw_Start_Time = tab.add("Sw_Start_Time", false).withPosition(2, 1).getEntry();
    
            ValueAngle = tab.add("ValueAngle", 0).withPosition(4, 0).getEntry();
            navx_angle = tab.add("navx_angle", 0).withPosition(5, 0).getEntry();

            Distance_front_Min = tab.add("Distance_front_Min", 0).withPosition(5, 1).getEntry();
            Angle_front_Min = tab.add("Angle_front_Min", 0).withPosition(4, 1).getEntry();
            
            
            Lidar_Distance_Front = tab.add("Lidar_Distance", 0).withPosition(5, 2).getEntry();
            Lidar_Distance_Left = tab.add("Lidar_Distance_Left", 0).withPosition(4, 3).getEntry();
            Lidar_Distance_Right = tab.add("Lidar_Distance_Right", 0).withPosition(6, 3).getEntry();


            Distance_left_Min = tab.add("Distance_left_Min", 0).getEntry();
            Angle_left_Min = tab.add("Angle_left_Min", 0).getEntry();

            Distance_left_Min_Protect = tab.add("Distance_left_Min_Protect", 0).getEntry();
            Angle_left_Min_Protect = tab.add("Angle_left_Min_Protect", 0).getEntry();

        }
    }


    /**
     * Starts the lidar if it was stopped
     */
    public void startScan()
    {
        lidar.start();
        scanning = true;
    }

    /**
     * Stops the lidar if needed. This will reduce the overhead of CPU and RAM by very little. 
     */
    public void stopScan()
    {
        lidar.stop();
        scanning = false;
    }

    public double getDistionLidar(int d)
    {
        // scanData.angle[degrees];
        scanData = lidar.getData();
        return scanData.distance[d];
        
    }







    public boolean getSwitchStart() {
        return !SW_Start.get() == true;
    }

    public boolean getSwitchEmergency() {
        return !SW_Emmer.get() == true;
    }

    public boolean getSwitchReset() {
        return !SW_Reset.get() == true;
    }

    public boolean getSwitchStop() {
        return !SW_Stop.get() == true;
    }

    public double getYaw() {
        return navx.getYaw();
    }

    public double getAngle() {
        return navx.getAngle();
    }

    int timeCount = 0;
    boolean timeStatus ;
    public boolean getTimeSwitch(){
        if (getSwitchStart() == true){
            timeCount ++;
            timeStatus = true;
        }
        else if(getSwitchEmergency() == true){
            timeCount = 0;
            timeStatus = false;
        }


        if(timeCount < 75 && timeStatus){
            return true;
        }
        else if(timeCount > 75 && timeStatus){
            return false; 
        }
        else{
            return false;
        }
    }

  

  /*
   * 
   * 
   * LIDAR
   * 
   * 
   * frc
   */

    private final double DEFAULT_MAX_DISTANCE = 9999.0;
    private final int FRONT_MIN_ANGLE_LEFT = DriveType.FRONT_MIN_ANGLE_LEFT;
    private final int FRONT_MAX_ANGLE_LEFT = DriveType.FRONT_MAX_ANGLE_LEFT;
    private final int FRONT_MIN_ANGLE_RIGHT = DriveType.FRONT_MIN_ANGLE_RIGHT;
    private final int FRONT_MAX_ANGLE_RIGHT = DriveType.FRONT_MAX_ANGLE_RIGHT;

    private final int LEFT_MIN_ANGLE_LEFT = DriveType.LEFT_MIN_ANGLE_LEFT;
    private final int LEFT_MAX_ANGLE_LEFT = DriveType.LEFT_MAX_ANGLE_LEFT;
    private final int LEFT_MIN_ANGLE_RIGHT = DriveType.LEFT_MIN_ANGLE_RIGHT;
    private final int LEFT_MAX_ANGLE_RIGHT = DriveType.LEFT_MAX_ANGLE_RIGHT;

    private final int RIGHT_MIN_ANGLE_LEFT = DriveType.RIGHT_MIN_ANGLE_LEFT;
    private final int RIGHT_MAX_ANGLE_LEFT = DriveType.RIGHT_MAX_ANGLE_LEFT;
    private final int RIGHT_MIN_ANGLE_RIGHT = DriveType.RIGHT_MIN_ANGLE_RIGHT;
    private final int RIGHT_MAX_ANGLE_RIGHT = DriveType.RIGHT_MAX_ANGLE_RIGHT;

    private final int LEFT_PROTECT_MIN_ANGLE_LEFT = DriveType.LEFT_PROTECT_MIN_ANGLE_LEFT;
    private final int LEFT_PROTECT_MAX_ANGLE_LEFT = DriveType.LEFT_PROTECT_MAX_ANGLE_LEFT;
    private final int LEFT_PROTECT_MIN_ANGLE_RIGHT = DriveType.LEFT_PROTECT_MIN_ANGLE_RIGHT;
    private final int LEFT_PROTECT_MAX_ANGLE_RIGHT = DriveType.LEFT_PROTECT_MAX_ANGLE_RIGHT;

    private final int RIGHT_BACK_MIN_ANGLE_LEFT = DriveType.RIGHT_MIN_ANGLE_LEFT;
    private final int RIGHT_BACK_MAX_ANGLE_LEFT = DriveType.RIGHT_MAX_ANGLE_LEFT;
    private final int RIGHT_BACK_MIN_ANGLE_RIGHT = DriveType.RIGHT_MIN_ANGLE_RIGHT;
    private final int RIGHT_BACK_MAX_ANGLE_RIGHT = DriveType.RIGHT_MAX_ANGLE_RIGHT;



    // ** Front */
    public double[] findFrontMinDistance() {
        double minDistance = DEFAULT_MAX_DISTANCE;
        double minAngle = FRONT_MIN_ANGLE_LEFT;

        for (int angle = FRONT_MIN_ANGLE_LEFT; angle <= FRONT_MAX_ANGLE_LEFT; angle += 5) {
        final double distance = getDistionLidar(angle);

        if (distance == 9999.0) {
            minDistance = distance;
            minAngle = angle;

        } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                    // เพื่อป้องกันค่าผิดพลาด
            minDistance = distance;
            minAngle = angle;
        }
        }

        for (int angle = FRONT_MIN_ANGLE_RIGHT; angle <= FRONT_MAX_ANGLE_RIGHT; angle += 5) {
        final double distance = getDistionLidar(angle);

        if (distance == 9999.0) {
            minDistance = distance;
            minAngle = angle;

        } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                    // เพื่อป้องกันค่าผิดพลาด
            minDistance = distance;
            minAngle = angle;
        }
        }

        if(Config.Sensor){
          Distance_front_Min.setDouble(minDistance);
          Angle_front_Min.setDouble(minAngle);
        }

        return new double[] { minDistance, minAngle };
    }

    
    
  // ** LEFT */

  public double[] findLeftMinDistance() {
    double minDistance = DEFAULT_MAX_DISTANCE;
    double minAngle = LEFT_MIN_ANGLE_LEFT;

    for (int angle = LEFT_MIN_ANGLE_LEFT; angle <= LEFT_MAX_ANGLE_LEFT; angle += 5) {
      final double distance = getDistionLidar(angle);

      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                 // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;
      }
    }

    for (int angle = LEFT_MIN_ANGLE_RIGHT; angle <= LEFT_MAX_ANGLE_RIGHT; angle += 5) {
      final double distance = getDistionLidar(angle);

      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;

      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
                                                                                 // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;
      }
    }

    if(Config.Sensor){
      Distance_left_Min.setDouble(minDistance);
      Angle_left_Min.setDouble(minAngle);
    }

    return new double[] { minDistance, minAngle };
  }
 




  // ** LEFT Protect */
  
  public double[] findLeftMinDistance_Protect() {
    double minDistance = DEFAULT_MAX_DISTANCE;
    double minAngle = LEFT_PROTECT_MIN_ANGLE_LEFT;
    
    for (int angle = LEFT_PROTECT_MIN_ANGLE_LEFT; angle <= LEFT_PROTECT_MAX_ANGLE_LEFT; angle += 10) {
      final double distance = getDistionLidar(angle);
      
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
      final double distance = getDistionLidar(angle);
      
      if (distance == 9999.0) {
        minDistance = distance;
        minAngle = angle;
        
      } else if (distance == 9999.0 || distance < minDistance && distance > 0) { // ตรวจสอบว่าค่า > 0
        // เพื่อป้องกันค่าผิดพลาด
        minDistance = distance;
        minAngle = angle;
        
      }
    }
    
    if(Config.Sensor){
    Distance_left_Min_Protect.setDouble(minDistance);
    Angle_left_Min_Protect.setDouble(minAngle);
    }
    
    return new double[] { minDistance, minAngle };
  }
  

    
    
    @Override
    public void periodic() {



        findFrontMinDistance();

        findLeftMinDistance();

        findLeftMinDistance_Protect();


         if(Config.Sensor){
             Sw_Start.setBoolean(getSwitchStart());
             Sw_Stop.setBoolean(getSwitchStop());
             Sw_Emmergenzy.setBoolean(getSwitchEmergency());
             Sw_Reset.setBoolean(getSwitchReset());
             ValueAngle.setDouble(getAngle());
             navx_angle.setDouble(getYaw());
   
            Lidar_Distance_Front.setDouble(getDistionLidar(270));
            
            Lidar_Distance_Left.setDouble(getDistionLidar(225));
            Lidar_Distance_Right.setDouble(getDistionLidar(315));

            Sw_Start_Time.setBoolean(getTimeSwitch());




         }
 
      
    }

}
