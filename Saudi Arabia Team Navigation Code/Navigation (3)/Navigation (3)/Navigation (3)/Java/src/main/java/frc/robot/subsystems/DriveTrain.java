package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

//Vendor imports
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import com.studica.frc.Lidar;

//WPI imports
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * DriveTrain class 
 * <p>
 * This class creates the instance of the Titan and enables and sets the speed of the defined motor. 
 */
public class DriveTrain extends SubsystemBase
{
    /**
     * Motors
     */
    private TitanQuad frontrightmotor;
    private TitanQuad frontleftmotor;
    private TitanQuad backleftmotor;
    private TitanQuad backrightmotor;


    /**
     * Encoders
     */
    private TitanQuadEncoder      frontleftEncoder;
    private TitanQuadEncoder      frontrightEncoder;
    private TitanQuadEncoder      backleftEncoder;
    private TitanQuadEncoder      backrightEncoder;

    /**
     * Sensors
     */
    private AHRS NavX;
     private Lidar lidar;
     private boolean scanning;
     public Lidar.ScanData scanData;
     private AnalogInput      sharp;

      /**
     * Odometry
     */
    public double posX;
    public double posY;

    /**
     * Control Panel
     */
    private        DigitalOutput          runningLED;
    private        DigitalOutput          stoppedLED;
    private        DigitalInput           startButton;
    private        DigitalInput           emergencyStopButton;




    /**
     * Constructor
     */
    public DriveTrain()
    {
        //Motors
        frontrightmotor = new TitanQuad(Constants.TITAN_ID, Constants.FrontRightMotor);
        frontleftmotor  = new TitanQuad(Constants.TITAN_ID, Constants.FrontLeftMotor);
        backrightmotor = new TitanQuad(Constants.TITAN_ID, Constants.BackRightMotor);
        backleftmotor  = new TitanQuad(Constants.TITAN_ID, Constants.BackLeftMotor);
        frontrightmotor.setInverted(true);
        backrightmotor.setInverted(true);


        //Encoder
        frontleftEncoder = new TitanQuadEncoder(frontleftmotor, Constants.FrontLeftMotor, Constants.wheelDistPerTick);
        frontrightEncoder = new TitanQuadEncoder(frontrightmotor, Constants.FrontRightMotor, Constants.wheelDistPerTick);
        backleftEncoder = new TitanQuadEncoder(backleftmotor, Constants.BackLeftMotor, Constants.wheelDistPerTick);
        backrightEncoder = new TitanQuadEncoder(backrightmotor, Constants.BackRightMotor, Constants.wheelDistPerTick);
       // frontrightEncoder.setReverseDirection();

        //Sensors
        NavX = new AHRS(SPI.Port.kMXP);
        lidar = new Lidar(Lidar.Port.kUSB2);

         //Control Panel
         runningLED = new DigitalOutput(Constants.runningLED); 
         stoppedLED = new DigitalOutput(Constants.stoppedLED);
         startButton = new DigitalInput(Constants.startButton);
         emergencyStopButton = new DigitalInput(Constants.emergencyStop);
        
    }


    public void setLeftMotorSpeed(double speed) {
        frontleftmotor.set(speed);
        backleftmotor.set(speed);
    }

    public void setRightMotorSpeed(double speed) {
        frontrightmotor.set(speed);
        backrightmotor.set(speed);
    }

    public void setTwoMotorSpeed(double leftSpeed, double rightSpeed) {
        setLeftMotorSpeed(leftSpeed);
        setRightMotorSpeed(rightSpeed);
    }

    // public void setTwo2MotorSpeed(double backleftspeed, double backrightspeed) {
    // backleftmotor.set(backleftspeed);
    // backrightmotor.set(backrightspeed);


    

     public void startScan()
    {
        lidar.start();
        scanning = true;
    }

    public void stopScan()
    {
        lidar.stop();
        scanning = false;
    }

    public double getFrontLeftEncoderDistance() {
        return frontleftEncoder.getEncoderDistance();
    }

    public double getFrontRightEncoderDistance() {
        return frontrightEncoder.getEncoderDistance();
    }

    public double getBackLeftEncoderDistance() {
        return frontleftEncoder.getEncoderDistance();
    }

    public double getBackRightEncoderDistance() {
        return frontrightEncoder.getEncoderDistance();
    }

    public double getAverageEncoderDistance() {
        return (getFrontRightEncoderDistance() + 
                getFrontLeftEncoderDistance() +
                 getBackLeftEncoderDistance() +
                  getBackRightEncoderDistance()) 
                / 4.0;
    }


    public double getfrontleftEncoderDistance() {
        return frontleftEncoder.getEncoderDistance();
    }

    public double getbackleftEncoderDistance() {
        return backleftEncoder.getEncoderDistance();
    }

    public double getfrontrightEncoderDistance() {
        return frontrightEncoder.getEncoderDistance();
    }

    public double getbackrightEncoderDistance() {
        return backrightEncoder.getEncoderDistance();
    }

    public void resetEncoders() {
        frontleftEncoder.reset();
        frontrightEncoder.reset();
        backleftEncoder.reset();
        backrightEncoder.reset();
        
    }

    public double getNavXYaw() {
        return NavX.getYaw();
    }

    public double getNavXAngle() {
        return NavX.getAngle();
    }

    public double getNavXPitch() {
        return NavX.getPitch();
    }

    public double getNavXRoll() {
        return NavX.getRoll();
    }

    public void resetNavX() {
        NavX.reset();
    }

    public double getHeadingRadians() {
        return Math.toRadians(getNavXYaw());
    }

    public double getDistance() {
        return (Math.pow(sharp.getAverageVoltage(), -1.2045)) * 27.726;
    }

    
    //Control Panel
    public void setRunningLED(boolean on) 
    { 
      runningLED.set(on); 
    }

    
    public void setStoppedLED(boolean on) 
    { 
      stoppedLED.set(on); 
    }


    public boolean getEmergencyStop() 
    {
         return emergencyStopButton.get(); 
    }


    public boolean getStartButton() 
    {
        return startButton.get();
    }


    public boolean getRunningLED()
    {
        return runningLED.get();
    }


    public boolean getStoppedLED()
    {
        return stoppedLED.get();
    }
    


    public void periodic()
    {
        scanData = lidar.getData();
        SmartDashboard.putNumber("Lidar 270", scanData.distance[270]);
        SmartDashboard.putNumber("Lidar 0", scanData.distance[0]);
        SmartDashboard.putNumber("Lidar 180", scanData.distance[180]);
        SmartDashboard.putNumber("Lidar 90", scanData.distance[90]);
        SmartDashboard.putNumber("LeftEncoder", getFrontLeftEncoderDistance());
        SmartDashboard.putNumber("RightEncoder", getFrontRightEncoderDistance());
        SmartDashboard.putNumber("BackLeftEncoder", getBackLeftEncoderDistance());
        SmartDashboard.putNumber("BackRightEncoder", getBackRightEncoderDistance());
        SmartDashboard.putNumber("AverageEncoder", getAverageEncoderDistance());
        SmartDashboard.putNumber("NavX Yaw", getNavXYaw());
        SmartDashboard.putNumber("NavX Angle", getNavXAngle());
        SmartDashboard.putNumber("NavX Roll", getNavXRoll());
        SmartDashboard.putNumber("NavX Pitch", getNavXPitch());
        SmartDashboard.putBoolean("Start Button", getStartButton());
        SmartDashboard.putBoolean("Emergency Button", getEmergencyStop());
        SmartDashboard.putNumber("posX", posX);
        SmartDashboard.putNumber("posY", posY);
    }
    

}