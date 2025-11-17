package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

// Vendor imports
import com.studica.frc.TitanQuad;
import com.studica.frc.TitanQuadEncoder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import com.studica.frc.Lidar;

// WPI imports
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * DriveTrain subsystem.
 * <p>
 * This class:
 * - Controls the left and right drive motors (TitanQuad).
 * - Reads encoders for distance tracking.
 * - Reads NavX (gyro) for orientation.
 * - Handles Lidar scanning and exposes scan data.
 */
public class DriveTrain extends SubsystemBase {

    // ============================
    //  DRIVE MOTORS
    // ============================
    private TitanQuad rightmotor;
    private TitanQuad leftmotor;

    // ============================
    //  ENCODERS
    // ============================
    private TitanQuadEncoder leftEncoder;
    private TitanQuadEncoder rightEncoder;

    private DigitalInput StartButton;
    private DigitalInput EStopButton;



    // ============================
    //  SENSORS
    // ============================
    private AHRS NavX;            // Gyro/IMU (NavX)
    private Lidar lidar;          // 2D Lidar (Studica)
    private boolean scanning;     // Flag to track if Lidar is currently scanning
    public Lidar.ScanData scanData; // Holds the latest Lidar scan data

    // ============================
    //  SIMPLE ODOMETRY (POSITION)
    // ============================
    // These can be updated externally using encoder and heading data.
    public double posX;
    public double posY;

    // ============================
    //  CONSTRUCTOR
    // ============================
    public DriveTrain() {
        // -----------------------------
        // Motors
        // -----------------------------
        // Create left and right TitanQuad motor controllers using the same Titan CAN ID
        // and the motor channel IDs defined in Constants.
        rightmotor = new TitanQuad(Constants.TITAN_ID, Constants.RightMotor);
        leftmotor  = new TitanQuad(Constants.TITAN_ID, Constants.LeftMotor);

        // Invert the right motor so that positive speed drives the robot forward
        // (This depends on wiring and gearbox orientation).
        rightmotor.setInverted(true);

        // -----------------------------
        // Encoders
        // -----------------------------
        // Encoders are attached to the TitanQuad controllers.
        // wheelDistPerTick converts encoder ticks to linear distance (meters or mm).
        leftEncoder = new TitanQuadEncoder(leftmotor, Constants.LeftMotor, Constants.wheelDistPerTick);
        rightEncoder = new TitanQuadEncoder(rightmotor, Constants.RightMotor, Constants.wheelDistPerTick);

        // Reverse direction of right encoder if needed so that:
        //   forward movement → both encoders increase in the same direction.
        rightEncoder.setReverseDirection();

        // -----------------------------
        // Sensors
        // -----------------------------
        // NavX gyro on the MXP port (SPI)
        NavX = new AHRS(SPI.Port.kMXP);

        // Lidar on USB2 (depends on how it's connected to the robot)
        lidar = new Lidar(Lidar.Port.kUSB2);
        StartButton = new DigitalInput(Constants.StartButton) ; 
        EStopButton = new DigitalInput(Constants.EStopButton) ; 

    }



    // ============================
    //  MOTOR CONTROL METHODS
    // ============================

    /**
     * Set speed of the left motor only.
     * @param speed Motor output in range [-1.0, 1.0].
     */
    public void setLeftMotorSpeed(double speed) {
        leftmotor.set(speed);
    }

    /**
     * Set speed of the right motor only.
     * @param speed Motor output in range [-1.0, 1.0].
     */
    public void setRightMotorSpeed(double speed) {
        rightmotor.set(speed);
    }

    /**
     * Set speed of both drive motors.
     * @param leftSpeed  Left motor speed [-1.0, 1.0].
     * @param rightSpeed Right motor speed [-1.0, 1.0].
     */
    public void setTwoMotorSpeed(double leftSpeed, double rightSpeed) {
        leftmotor.set(leftSpeed);
        rightmotor.set(rightSpeed);
    }

    // ============================
    //  LIDAR CONTROL
    // ============================

    /**
     * Start the Lidar scanning process.
     * Call this once (e.g. in robotInit or subsystem init).
     */
    public void startScan() {
        lidar.start();
        scanning = true;
    }

    /**
     * Stop the Lidar scanning process.
     * (Optional, usually you keep it running during the match).
     */
    public void stopScan() {
        lidar.stop();
        scanning = false;
    }

    // ============================
    //  ENCODER ACCESS
    // ============================

    /**
     * @return Distance traveled measured by the left encoder
     *         in units defined by Constants.wheelDistPerTick.
     */
    public double getLeftEncoderDistance() {
        return leftEncoder.getEncoderDistance();
    }

    /**
     * @return Distance traveled measured by the right encoder
     *         in units defined by Constants.wheelDistPerTick.
     */
    public double getRightEncoderDistance() {
        return rightEncoder.getEncoderDistance();
    }

    /**
     * @return Average distance between left and right encoders.
     *         Useful for forward odometry.
     */
    public double getAverageEncoderDistance() {
        return (getRightEncoderDistance() + getLeftEncoderDistance()) / 2.0;
    }

    /**
     * Reset both encoders to zero distance.
     */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    public Boolean getStartButton() {
        return (StartButton.get());
    }

    public Boolean getEStopButton() {
        return (EStopButton.get());
    }

    // ============================
    //  NAVX (GYRO) ACCESS
    // ============================

    /**
     * @return NavX yaw angle in degrees (typically -180 to 180).
     *         Used for heading control and turning.
     */
    public double getNavXYaw() {
        return NavX.getYaw();
    }

    /**
     * @return NavX integrated angle in degrees.
     *         This may increase continuously while spinning.
     */
    public double getNavXAngle() {
        return NavX.getAngle();
    }

    /**
     * @return NavX pitch angle in degrees.
     */
    public double getNavXPitch() {
        return NavX.getPitch();
    }

    /**
     * @return NavX roll angle in degrees.
     */
    public double getNavXRoll() {
        return NavX.getRoll();
    }

    /**
     * Reset NavX yaw/angle to zero.
     * Useful before autonomous or when re-zeroing heading.
     */
    public void resetNavX() {
        NavX.reset();
    }

    /**
     * @return Heading in radians (converted from Yaw).
     *         Useful if you want to use trigonometry with Math.sin/cos.
     */
    public double getHeadingRadians() {
        return Math.toRadians(getNavXYaw());
    }

    // ============================
    //  PERIODIC — CALLED EVERY LOOP
    // ============================

    @Override
    public void periodic() {
        // Update Lidar scan data each loop
        // Make sure lidar.start() was called somewhere (e.g., robotInit).
        scanData = lidar.getData();

        // Send some key Lidar distances to SmartDashboard
        // Angles assume 0 = front, 90 = left, 180 = back, 270 = right (depends on your Lidar library).
        SmartDashboard.putNumber("Lidar 270 (right)", scanData.distance[270]);
        SmartDashboard.putNumber("Lidar 0 (front)",   scanData.distance[0]);
        SmartDashboard.putNumber("Lidar 180 (back)",  scanData.distance[180]);
        SmartDashboard.putNumber("Lidar 90 (left)",   scanData.distance[90]);

        // Encoders
        SmartDashboard.putNumber("LeftEncoder", getLeftEncoderDistance());
        SmartDashboard.putNumber("RightEncoder", getRightEncoderDistance());
        SmartDashboard.putNumber("AverageEncoder", getAverageEncoderDistance());

        // NavX orientation
        SmartDashboard.putNumber("NavX Yaw", getNavXYaw());
        SmartDashboard.putNumber("NavX Angle", getNavXAngle());
        SmartDashboard.putNumber("NavX Roll", getNavXRoll());
        SmartDashboard.putNumber("NavX Pitch", getNavXPitch());

        // Odometry values (posX, posY) – these must be updated somewhere else
        SmartDashboard.putNumber("posX", posX);
        SmartDashboard.putNumber("posY", posY);
    }

}
