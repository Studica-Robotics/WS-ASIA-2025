package frc.robot;


public final class Constants
{
    /**
     * Motor Constants
     */
    public static final int TITAN_ID        = 42;
    public static final int FrontRightMotor          = 3;
    public static final int FrontLeftMotor          = 1;
    public static final int BackRightMotor          = 2;
    public static final int BackLeftMotor          = 0;


     /**
     * Encoders
     */

    public static final double wheelDiameter = 125;
    public static final double pulsePerRev = 1464;
    public static final double gearRatio = 1/1;
    public static final double wheelPulse = pulsePerRev * gearRatio;
    public static final double wheelDistPerTick = (Math.PI * wheelDiameter) / pulsePerRev;

    /**
     * Control Panel
     */
        
    public static final int runningLED    = 21;
    public static final int stoppedLED    = 20;
    public static final int startButton   = 10;    
    public static final int emergencyStop =  9;      
}
