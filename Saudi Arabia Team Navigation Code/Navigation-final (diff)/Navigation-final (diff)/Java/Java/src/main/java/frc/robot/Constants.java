package frc.robot;


public final class Constants
{
    /**
     * Motor Constants
     */
    public static final int TITAN_ID        = 42;
    public static final int RightMotor          = 2;
    public static final int LeftMotor          = 0;

     /**
     * Encoders
     */

    public static final double wheelDiameter = 125;
    public static final double pulsePerRev = 1464;
    public static final double gearRatio = 1/1;
    public static final double wheelPulse = pulsePerRev * gearRatio;
    public static final double wheelDistPerTick = (Math.PI * wheelDiameter) / pulsePerRev;


    public static final int StartButton = 8;
    public static final int EStopButton = 9;


}
