package frc.robot.commands.driveCommands;

import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class Lidar extends CommandBase {

    private static final DriveTrain drive = RobotContainer.drive;

    // ======================================================
    //  ROBOT DIMENSIONS RELATIVE TO LIDAR (in mm)
    // ======================================================
    private static final double LIDAR_TO_FRONT_MM = 200.0; // 20 cm
    private static final double LIDAR_TO_RIGHT_MM = 150.0; // 15 cm
    private static final double LIDAR_TO_LEFT_MM  = 150.0; // 15 cm

    // ===================
    //  SPEED SETTINGS
    // ===================
    private static final double FORWARD_SPEED   = 0.25;
    private static final double MIN_TURN_SPEED = 0.10;
    private static final double MAX_TURN_SPEED = 0.25;

    // ======================================================
    //  FRONT SAFETY DISTANCE (distance from robot front)
    // ======================================================
    private static final double FRONT_CLEAR_DIST_MM = 100.0;

    // ============================
    //  TURNING (NAVX SETTINGS)
    // ============================
    private static final double TURN_ANGLE_DEG  = 88.0;
    private static final double ANGLE_TOLERANCE = 2.0;
    private static final double TURN_KP = 0.01;    // used to scale turning speed

    // =====================================
    //  HEADING HOLD (FOR STRAIGHT DRIVING)
    // =====================================
    private static final double HEADING_KP = 0.02;      
    private static final double HEADING_MAX_CORR = 0.15;

    // LIDAR angle indices
    private static final int FRONT_ANGLE = 180;
    private static final int LEFT_ANGLE  = 90;
    private static final int RIGHT_ANGLE = 270;

    // Robot movement states
    private enum Mode {
        FORWARD,
        TURNING_LEFT,
        TURNING_RIGHT
    }

    private Mode mode = Mode.FORWARD;

    // Angle used to track turning progress
    private double turnStartAngle = 0.0;

    // Heading used for straight driving
    private double forwardHeading = 0.0;

    public Lidar() {
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Reset NavX at the beginning
        drive.resetNavX();

        mode = Mode.FORWARD;

        // Store the initial heading as the forward direction
        forwardHeading = getHeading();
    }

    @Override
    public void execute() {
        switch (mode) {
            case FORWARD:
                handleForwardMode();
                break;

            case TURNING_LEFT:
                handleTurningMode(true);
                break;

            case TURNING_RIGHT:
                handleTurningMode(false);
                break;
        }
    }

    // =======================================
    //  FORWARD MODE — MOVE STRAIGHT UNTIL OBSTACLE
    // =======================================
    private void handleForwardMode() {

        double frontRaw = getMinFrontDistance();
        double rightRaw = getRawDistance(RIGHT_ANGLE);
        double leftRaw  = getRawDistance(LEFT_ANGLE);

        double frontRobot;

        if (!isValid(frontRaw)) {
            frontRobot = -1;
        } else {
            frontRobot = Math.max(frontRaw - LIDAR_TO_FRONT_MM, 0.0);
        }

        // If obstacle detected too close in front → choose a turning side
        if (frontRobot != -1 && frontRobot < FRONT_CLEAR_DIST_MM) {

            boolean turnLeft;

            double leftDist  = isValid(leftRaw)  ? Math.max(leftRaw  - LIDAR_TO_LEFT_MM, 0.0) : -1;
            double rightDist = isValid(rightRaw) ? Math.max(rightRaw - LIDAR_TO_RIGHT_MM, 0.0) : -1;

            if (!isValid(leftDist) && !isValid(rightDist)) {
                turnLeft = true; // default
            } else if (!isValid(leftDist)) {
                turnLeft = false;
            } else if (!isValid(rightDist)) {
                turnLeft = true;
            } else {
                turnLeft = leftDist > rightDist; // pick the more open side
            }

            if (turnLeft) startTurnLeft(); 
            else startTurnRight();

            return;
        }

        // Move forward using heading correction
        driveStraightWithHeading();
    }

    // ======================================================
    //  HEADING HOLD — KEEP ROBOT DRIVING STRAIGHT
    // ======================================================
    private void driveStraightWithHeading() {
        double currentAngle = getHeading();

        // signed angle error: how much robot drifted left/right
        double errorDeg = signedAngleDelta(forwardHeading, currentAngle);

        // correction speed adjustment
        double correction = HEADING_KP * errorDeg;
        correction = MathUtil.clamp(correction, -HEADING_MAX_CORR, HEADING_MAX_CORR);

        // Modify wheel speeds to force robot straight
        double leftSpeed  = FORWARD_SPEED - correction;
        double rightSpeed = FORWARD_SPEED + correction;

        drive.setTwoMotorSpeed(leftSpeed, rightSpeed);
    }

    // =======================================
    //  TURNING MODE — NAVX BASED ROTATION
    // =======================================
    private void handleTurningMode(boolean leftTurn) {

        double currentAngle = getHeading();
        double turnedDeg = Math.abs(signedAngleDelta(turnStartAngle, currentAngle));

        double remaining = TURN_ANGLE_DEG - turnedDeg;

        if (remaining <= ANGLE_TOLERANCE) {
            mode = Mode.FORWARD;
            forwardHeading = getHeading(); // new straight direction after turn
            return;
        }

        // dynamic turning speed based on remaining angle
        double turnSpeed = TURN_KP * remaining;
        turnSpeed = MathUtil.clamp(turnSpeed, MIN_TURN_SPEED, MAX_TURN_SPEED);

        if (leftTurn)
            drive.setTwoMotorSpeed(-turnSpeed, turnSpeed);
        else
            drive.setTwoMotorSpeed(turnSpeed, -turnSpeed);
    }

    private void startTurnLeft() {
        turnStartAngle = getHeading();
        mode = Mode.TURNING_LEFT;
    }

    private void startTurnRight() {
        turnStartAngle = getHeading();
        mode = Mode.TURNING_RIGHT;
    }

    // =======================================
    //  LIDAR HELPERS
    // =======================================
    private double getRawDistance(int angle) {
        int idx = angle % 360;
        if (idx < 0) idx += 360;
        return drive.scanData.distance[idx];
    }

    private boolean isValid(double d) {
        return d != -1.0;
    }

    // read minimum distance in front sector (170–190 degrees)
    private double getMinFrontDistance() {
        double min = -1.0;

        for (int angle = 170; angle <= 190; angle++) {
            double d = getRawDistance(angle);
            if (!isValid(d)) continue;
            if (min == -1 || d < min) min = d;
        }

        return min;
    }

    // =======================================
    //  NAVX HELPERS
    // =======================================
    private double getHeading() {
        return drive.getNavXYaw();
    }

    // signed angle difference (-180° to +180°)
    private double signedAngleDelta(double start, double current) {
        double diff = current - start;
        while (diff > 180.0) diff -= 360.0;
        while (diff < -180.0) diff += 360.0;
        return diff;
    }

    @Override
    public void end(boolean interrupted) {
        drive.setTwoMotorSpeed(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}






/* 
=====================================================================================================================

1) ROBOT DRIFTS LEFT/RIGHT WHILE DRIVING STRAIGHT:
    → Increase HEADING_KP (example: 0.02 → 0.03 → 0.04)
    → Or increase HEADING_MAX_CORR (0.15 → 0.2)

2) TURN ANGLE IS TOO SMALL (<90°):
    → Increase TURN_ANGLE_DEG (88 → 89 → 90)
    → Or increase TURN_KP to make turning more aggressive

3) TURN ANGLE OVERSHOOTS (>90°):
    → Decrease TURN_ANGLE_DEG (88 → 87)
    → Decrease MAX_TURN_SPEED or increase MIN_TURN_SPEED slightly

4) ROBOT STOPS TOO CLOSE TO FRONT WALL:
    → Increase FRONT_CLEAR_DIST_MM (100 → 150 → 200)

5) TURNING IS TOO FAST OR SHAKY:
    → Reduce MAX_TURN_SPEED (0.25 → 0.2)
    → Increase ANGLE_TOLERANCE slightly (2° → 3°)

6) TURNING TOO SLOW:
    → Increase MIN_TURN_SPEED (0.10 → 0.12 → 0.15)
    → Increase TURN_KP slightly (0.01 → 0.015)

=====================================================================================================================
*/