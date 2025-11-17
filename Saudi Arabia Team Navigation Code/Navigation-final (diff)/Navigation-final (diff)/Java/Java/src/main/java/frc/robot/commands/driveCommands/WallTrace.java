package frc.robot.commands.driveCommands;

import edu.wpi.first.wpiutil.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
 * Autonomous wall-tracing command dengan tahapan lengkap:
 * <ol>
 *   <li>Masuk arena (maju 30-40 cm) lalu putar kanan 90 derajat</li>
 *   <li>Maju sampai sisi kanan melihat dinding lalu mulai trace kanan</li>
 *   <li>Jika sisi kanan & depan kosong, jalankan pola eksplorasi berbentuk L</li>
 *   <li>Jika dead-end (semua sisi dekat) putar kiri 180 derajat</li>
 * </ol>
 *
 * Orientasi Lidar pada robot: depan 180 derajat, kiri 90 derajat, kanan 270 derajat.
 */
public class WallTrace extends CommandBase {

  private static final DriveTrain drive = RobotContainer.drive;

  // Kecepatan dasar tracing dan gain koreksi
  private static final double TRACE_BASE_SPEED = 0.38;
  private static final double TRACE_TURN_SPEED = 0.32;
  private static final double RIGHT_TARGET_MM = 350.0;
  private static final double LEFT_TARGET_MM = 350.0;
  private static final double FRONT_MIN_MM = 320.0;
  private static final double DEAD_END_THRESHOLD_MM = 280.0;
  private static final double OPEN_FRONT_MIN_MM = 650.0;
  private static final double OPEN_RIGHT_MIN_MM = 650.0;
  private static final double KP = 0.0032;
  private static final double KD = 0.0012;
  private static final double MAX_CORRECTION = 0.25;

  // Tahapan gerakan khusus
  private static final double ENTRY_DISTANCE_MM = 350.0;
  private static final double ENTRY_SPEED = 0.4;
  private static final double SEEK_WALL_SPEED = 0.25;
  private static final double OPEN_STEP_MM = 300.0;
  private static final double OPEN_SPEED = 0.35;
  private static final double TURN_KP = 0.012;
  private static final double TURN_MAX_OUTPUT = 0.4;
  private static final double TURN_TOLERANCE_DEG = 2.0;

  public enum Side {
    LEFT,
    RIGHT
  }

  private enum Phase {
    ENTER_FORWARD,
    ENTER_TURN_RIGHT,
    SEEK_RIGHT_WALL,
    TRACE,
    OPEN_FORWARD_ONE,
    OPEN_TURN_RIGHT_ONE,
    OPEN_FORWARD_TWO,
    OPEN_TURN_RIGHT_TWO,
    REACQUIRE_RIGHT_WALL,
    ESCAPE_TURN
  }

  private Side preferredSide;
  private Phase phase;
  private double previousError = 0.0;
  private double targetHeading = 0.0;

  public WallTrace() {
    this(Side.RIGHT);
  }

  public WallTrace(Side preferredSide) {
    this.preferredSide = preferredSide;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.resetEncoders();
    drive.resetNavX();
    drive.startScan();
    previousError = 0.0;
    startPhase(Phase.ENTER_FORWARD);
  }

  @Override
  public void execute() {
    switch (phase) {
      case ENTER_FORWARD:
        if (driveForwardDistance(ENTRY_DISTANCE_MM, ENTRY_SPEED)) {
          stopDrive();
          startPhase(Phase.ENTER_TURN_RIGHT);
        }
        return;
      case ENTER_TURN_RIGHT:
        if (turnToTarget()) {
          stopDrive();
          startPhase(Phase.SEEK_RIGHT_WALL);
        }
        return;
      case SEEK_RIGHT_WALL:
        if (seekRightWall()) {
          stopDrive();
          startPhase(Phase.TRACE);
        }
        return;
      case OPEN_FORWARD_ONE:
        if (driveForwardDistance(OPEN_STEP_MM, OPEN_SPEED)) {
          stopDrive();
          startPhase(Phase.OPEN_TURN_RIGHT_ONE);
        }
        return;
      case OPEN_TURN_RIGHT_ONE:
        if (turnToTarget()) {
          stopDrive();
          startPhase(Phase.OPEN_FORWARD_TWO);
        }
        return;
      case OPEN_FORWARD_TWO:
        if (driveForwardDistance(OPEN_STEP_MM, OPEN_SPEED)) {
          stopDrive();
          startPhase(Phase.OPEN_TURN_RIGHT_TWO);
        }
        return;
      case OPEN_TURN_RIGHT_TWO:
        if (turnToTarget()) {
          stopDrive();
          startPhase(Phase.REACQUIRE_RIGHT_WALL);
        }
        return;
      case REACQUIRE_RIGHT_WALL:
        if (seekRightWall()) {
          stopDrive();
          startPhase(Phase.TRACE);
        }
        return;
      case ESCAPE_TURN:
        if (turnToTarget()) {
          stopDrive();
          startPhase(Phase.TRACE);
        }
        return;
      case TRACE:
      default:
        traceLoop();
        return;
    }
  }

  @Override
  public void end(boolean interrupted) {
    stopDrive();
    drive.stopScan();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void setPreferredSide(Side newSide) {
    preferredSide = newSide;
  }

  public void toggleSide() {
    preferredSide = (preferredSide == Side.RIGHT) ? Side.LEFT : Side.RIGHT;
  }

  public Side getPreferredSide() {
    return preferredSide;
  }

  private void traceLoop() {
    double front = getDistance(180);
    double left = getDistance(90);
    double right = getDistance(270);

    boolean frontClose = isValid(front) && front < FRONT_MIN_MM;
    boolean rightClose = isValid(right) && right < DEAD_END_THRESHOLD_MM;
    boolean leftClose = isValid(left) && left < DEAD_END_THRESHOLD_MM;

    if (frontClose && rightClose && leftClose) {
      stopDrive();
      startPhase(Phase.ESCAPE_TURN);
      return;
    }

    boolean rightOpen = !isValid(right) || right > OPEN_RIGHT_MIN_MM;
    boolean frontOpen = !isValid(front) || front > OPEN_FRONT_MIN_MM;
    if (rightOpen && frontOpen) {
      stopDrive();
      startPhase(Phase.OPEN_FORWARD_ONE);
      return;
    }

    if (frontClose) {
      drive.setTwoMotorSpeed(-TRACE_TURN_SPEED, TRACE_TURN_SPEED);
      previousError = 0.0;
      return;
    }

    runWallFollowing(left, right);
  }

  private void runWallFollowing(double left, double right) {
    boolean rightValid = isValid(right);
    boolean leftValid = isValid(left);

    Side activeSide = preferredSide;
    if (activeSide == Side.RIGHT && !rightValid && leftValid) {
      activeSide = Side.LEFT;
    } else if (activeSide == Side.LEFT && !leftValid && rightValid) {
      activeSide = Side.RIGHT;
    }

    boolean followRight = activeSide == Side.RIGHT;
    double target = followRight ? RIGHT_TARGET_MM : LEFT_TARGET_MM;
    double measured = followRight ? right : left;

    if (!isValid(measured)) {
      drive.setTwoMotorSpeed(TRACE_BASE_SPEED, TRACE_BASE_SPEED);
      previousError = 0.0;
      return;
    }

    double error = target - measured;
    if (!followRight) {
      error = -error;
    }

    double derivative = error - previousError;
    double correction = KP * error + KD * derivative;
    correction = MathUtil.clamp(correction, -MAX_CORRECTION, MAX_CORRECTION);

    double leftSpeed = MathUtil.clamp(TRACE_BASE_SPEED + correction, -0.8, 0.8);
    double rightSpeed = MathUtil.clamp(TRACE_BASE_SPEED - correction, -0.8, 0.8);

    drive.setTwoMotorSpeed(leftSpeed, rightSpeed);
    previousError = error;
  }

  private boolean seekRightWall() {
    double front = getDistance(180);
    if (isValid(front) && front < FRONT_MIN_MM) {
      stopDrive();
      startPhase(Phase.ESCAPE_TURN);
      return false;
    }

    double right = getDistance(270);
    if (isValid(right) && right <= RIGHT_TARGET_MM) {
      return true;
    }
    drive.setTwoMotorSpeed(SEEK_WALL_SPEED, SEEK_WALL_SPEED);
    return false;
  }

  private void startPhase(Phase newPhase) {
    phase = newPhase;
    switch (newPhase) {
      case ENTER_FORWARD:
        drive.resetEncoders();
        break;
      case ENTER_TURN_RIGHT:
        targetHeading = normalizeAngle(drive.getNavXAngle() - 90.0);
        break;
      case SEEK_RIGHT_WALL:
        drive.resetEncoders();
        break;
      case OPEN_FORWARD_ONE:
      case OPEN_FORWARD_TWO:
      case REACQUIRE_RIGHT_WALL:
        drive.resetEncoders();
        break;
      case OPEN_TURN_RIGHT_ONE:
      case OPEN_TURN_RIGHT_TWO:
        targetHeading = normalizeAngle(drive.getNavXAngle() - 90.0);
        break;
      case ESCAPE_TURN:
        targetHeading = normalizeAngle(drive.getNavXAngle() + 180.0);
        break;
      case TRACE:
      default:
        previousError = 0.0;
        break;
    }
  }

  private boolean driveForwardDistance(double distanceMm, double speed) {
    double travelled = Math.abs(drive.getAverageEncoderDistance());
    drive.setTwoMotorSpeed(speed, speed);
    return travelled >= distanceMm;
  }

  private boolean turnToTarget() {
    double error = normalizeAngle(targetHeading - drive.getNavXAngle());
    double output = MathUtil.clamp(error * TURN_KP, -TURN_MAX_OUTPUT, TURN_MAX_OUTPUT);
    drive.setTwoMotorSpeed(output, -output);
    return Math.abs(error) <= TURN_TOLERANCE_DEG;
  }

  private void stopDrive() {
    drive.setTwoMotorSpeed(0.0, 0.0);
  }

  private static double getDistance(int angleDegrees) {
    if (drive.scanData == null || drive.scanData.distance == null || drive.scanData.distance.length == 0) {
      return -1.0;
    }
    int index = Math.floorMod(angleDegrees, drive.scanData.distance.length);
    return drive.scanData.distance[index];
  }

  private static boolean isValid(double value) {
    return value > 0;
  }

  private static double normalizeAngle(double angleDeg) {
    double angle = angleDeg;
    while (angle > 180.0) {
      angle -= 360.0;
    }
    while (angle < -180.0) {
      angle += 360.0;
    }
    return angle;
  }
}
