package frc.robot.commands.driveCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/**
 * Navigation command that implements a simple wall-following state machine
 * similar to the C++ Navigation example. It uses Lidar + NavX from the
 * DriveTrain subsystem and can be mocked by commenting out the motor calls
 * and replacing them with prints during console testing.
 */
public class Navigation extends CommandBase {

  private enum RobotState {
    CURT_ENTRANCE,
    IDLE,
    FOLLOW_WALL,
    ROTATE_RIGHT,
    ROTATE_LEFT
  }

  private static final DriveTrain drive = RobotContainer.drive;

  // Timings and speeds (seconds / normalized outputs)
  private static final double TIME_TO_ENTER_CURT = 2.0;
  private static final double SPEED_TO_ENTER_CURT = 0.3;

  // Rotation parameters
  private static final double ROTATE_ANGLE_DEG = 45.0;
  private static final double TURN_TOLERANCE_DEG = 2.0;
  private static final double ANGULAR_VELOCITY_LEFT_1 = 0.3;
  private static final double ANGULAR_VELOCITY_LEFT_2 = 0.33;
  private static final double ANGULAR_VELOCITY_RIGHT = 0.3;

  // Lidar thresholds (in millimeters, because scanData is mm)
  private static final double FRONT_OBSTACLE_DIST_MM = 250.0; // ~0.25 m
  private static final double SIDE_OBSTACLE_DIST_MM = 550.0;  // ~0.55 m

  private RobotState state = RobotState.FOLLOW_WALL;
  private boolean isCurrentStateDone = true;
  private boolean doOnceFlag = true;
  private boolean robotInCurt = true;
  private int rotationCounter = 0;

  private double lastTimeSec;
  private double targetAngleDeg;

  private boolean frontObjFlag;
  private boolean leftObjFlag;

  public Navigation() {
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.resetNavX();
    drive.startScan();
    state = RobotState.FOLLOW_WALL;
    isCurrentStateDone = true;
    doOnceFlag = true;
    robotInCurt = true;
    rotationCounter = 0;
  }

  @Override
  public void execute() {
    updateObstacleFlags();
    SmartDashboard.putNumber("Nav Angle", drive.getNavXAngle());

    if (isCurrentStateDone && !robotInCurt) {
      state = RobotState.CURT_ENTRANCE;
    } else {
      if (leftObjFlag && !frontObjFlag && isCurrentStateDone) {
        state = RobotState.FOLLOW_WALL;
      } else if (frontObjFlag && isCurrentStateDone) {
        state = RobotState.ROTATE_RIGHT;
      } else if (!leftObjFlag && !frontObjFlag && isCurrentStateDone) {
        state = RobotState.ROTATE_LEFT;
      }
    }

    switch (state) {
      case CURT_ENTRANCE:
        handleCurtEntrance();
        break;
      case IDLE:
        handleIdle();
        break;
      case FOLLOW_WALL:
        handleFollowWall();
        break;
      case ROTATE_RIGHT:
        handleRotate(false);
        break;
      case ROTATE_LEFT:
        handleRotate(true);
        break;
      default:
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // For console / mock testing, you can comment this out
    drive.setTwoMotorSpeed(0.0, 0.0);
    drive.stopScan();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void handleIdle() {
    isCurrentStateDone = false;
    double now = Timer.getFPGATimestamp();
    if (doOnceFlag) {
      doOnceFlag = false;
      lastTimeSec = now;
      // For console testing, comment this line and print instead
      drive.setTwoMotorSpeed(0.0, 0.0);
    }
    if (now - lastTimeSec >= 0.1) {
      // For console testing, comment this line and print instead
      drive.setTwoMotorSpeed(0.0, 0.0);
      isCurrentStateDone = true;
      doOnceFlag = true;
    }
  }

  private void handleCurtEntrance() {
    isCurrentStateDone = false;
    double now = Timer.getFPGATimestamp();
    if (doOnceFlag) {
      doOnceFlag = false;
      lastTimeSec = now;
    }
    if (now - lastTimeSec < TIME_TO_ENTER_CURT) {
      // For console testing, comment this line and print instead
      drive.setTwoMotorSpeed(SPEED_TO_ENTER_CURT, SPEED_TO_ENTER_CURT);
    } else {
      doOnceFlag = true;
      robotInCurt = true;
      state = RobotState.IDLE;
    }
  }

  private void handleFollowWall() {
    isCurrentStateDone = false;
    double now = Timer.getFPGATimestamp();
    if (doOnceFlag) {
      doOnceFlag = false;
      lastTimeSec = now;
    }

    if (frontObjFlag || (now - lastTimeSec) > 2.0) {
      doOnceFlag = true;
      state = RobotState.IDLE;
      return;
    }

    double left = getDistanceMm(90);
    double right = getDistanceMm(270);

    // Reuse simple PD-based wall following similar to WallTrace
    double target = SIDE_OBSTACLE_DIST_MM;
    double measured = left; // follow left wall by default

    if (measured <= 0) {
      // For console testing, comment this line and print instead
      drive.setTwoMotorSpeed(0.35, 0.35);
      return;
    }

    double error = target - measured;
    double kP = 0.0032;
    double kD = 0.0012;
    double correction = kP * error; // simplified: no derivative state

    double baseSpeed = 0.38;
    double leftSpeed = baseSpeed + correction;
    double rightSpeed = baseSpeed - correction;

    leftSpeed = Math.max(Math.min(leftSpeed, 0.8), -0.8);
    rightSpeed = Math.max(Math.min(rightSpeed, 0.8), -0.8);

    // For console testing, comment this line and print instead
    drive.setTwoMotorSpeed(leftSpeed, rightSpeed);
  }

  private void handleRotate(boolean left) {
    isCurrentStateDone = false;
    double currentAngle = normalizeAngle(drive.getNavXAngle());
    if (doOnceFlag) {
      doOnceFlag = false;
      double delta = left ? ROTATE_ANGLE_DEG : -ROTATE_ANGLE_DEG;
      targetAngleDeg = normalizeAngle(currentAngle + delta);
      if (left) {
        rotationCounter++;
      } else {
        rotationCounter = 0;
      }
    }

    double error = normalizeAngle(targetAngleDeg - currentAngle);
    if (Math.abs(error) < TURN_TOLERANCE_DEG) {
      doOnceFlag = true;
      state = RobotState.IDLE;
      return;
    }

    double angularVel =
        (rotationCounter == 1) ? ANGULAR_VELOCITY_LEFT_1 : ANGULAR_VELOCITY_LEFT_2;
    double leftSpeed;
    double rightSpeed;

    if (left) {
      leftSpeed = -angularVel;
      rightSpeed = angularVel;
    } else {
      leftSpeed = ANGULAR_VELOCITY_RIGHT;
      rightSpeed = -ANGULAR_VELOCITY_RIGHT;
    }

    // For console testing, comment this line and print instead
    drive.setTwoMotorSpeed(leftSpeed, rightSpeed);
  }

  private void updateObstacleFlags() {
    double front = getDistanceMm(180);
    double left = getDistanceMm(90);
    frontObjFlag = front > 0 && front < FRONT_OBSTACLE_DIST_MM;
    leftObjFlag = left > 0 && left < SIDE_OBSTACLE_DIST_MM;

    SmartDashboard.putBoolean("FrontObstacle", frontObjFlag);
    SmartDashboard.putBoolean("LeftObstacle", leftObjFlag);
  }

  private static double getDistanceMm(int angleDeg) {
    if (drive.scanData == null || drive.scanData.distance == null
        || drive.scanData.distance.length == 0) {
      return -1.0;
    }
    int index = Math.floorMod(angleDeg, drive.scanData.distance.length);
    return drive.scanData.distance[index];
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

