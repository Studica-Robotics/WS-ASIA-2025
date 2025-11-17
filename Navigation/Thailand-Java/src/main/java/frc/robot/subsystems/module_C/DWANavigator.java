/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.module_C;

import java.util.*;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotContainer;
import frc.robot.DriveType_ModuleC.Mecanum;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Sensor;

public class DWANavigator {

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
//   private static final TheStack_MyBot driveType = RobotContainer.theStack_MyBot;
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

  private int r;
  private int l;

  private final static ShuffleboardTab tab = Shuffleboard.getTab("DWA");

// ความเร็วที่เลือกได้
private final NetworkTableEntry DWA_VX = tab.add("DWA_VX", 0).withPosition(0,0).getEntry();
private final NetworkTableEntry DWA_VY = tab.add("DWA_VY", 0).withPosition(1,0).getEntry();
private final NetworkTableEntry DWA_OMEGA = tab.add("DWA_OMEGA", 0).withPosition(2,0).getEntry();

// คะแนนที่ดีที่สุด
private final NetworkTableEntry DWA_BEST_SCORE = tab.add("DWA_BestScore", 0).withPosition(3,0).getEntry();

// ระยะห่าง obstacle ที่ใกล้ที่สุด
private final NetworkTableEntry DWA_MIN_DIST = tab.add("DWA_MinDist", 0).withPosition(4,0).getEntry();

// มุมเป้าหมาย
private final NetworkTableEntry DWA_GOAL_ANGLE = tab.add("DWA_GoalAngle", 0).withPosition(5,0).getEntry();

// จำนวน obstacle ที่ตรวจพบ
private final NetworkTableEntry DWA_OBS_COUNT = tab.add("DWA_ObsCount", 0).withPosition(6,0).getEntry();

// สถานะการชน
private final NetworkTableEntry DWA_COLLISION = tab.add("DWA_Collision", false).withPosition(7,0).getEntry();

// สถานะการเลือก drive type
private final NetworkTableEntry DWA_DRIVE_TYPE = tab.add("DriveType", TypeDrive()).withPosition(8,0).getEntry();


    public static class Velocity {
        public double vx, vy, omega;
        public Velocity(double vx, double vy, double omega) {
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
        }
    }

    public static class RobotState {
        public double x, y, theta, vx, vy, omega;
        public RobotState(double x, double y, double theta, double vx, double vy, double omega) {
            this.x = x;
            this.y = y;
            this.theta = theta;
            this.vx = vx;
            this.vy = vy;
            this.omega = omega;
        }
    }

    // Parameters
    private double maxVx = 1.0;      // m/s
    private double maxVy = 1.0;      // m/s
    private double maxOmega = 1.0;   // rad/s
    private double accVx = 0.5;      // m/s^2
    private double accVy = 0.5;      // m/s^2
    private double accOmega = 0.5;   // rad/s^2
    private double dt = 0.1;         // time step (s)
    private double predictTime = 2.0; // prediction horizon (s)
    private double robotRadius = 0.3; // m

    public DWANavigator() {}

    public DWANavigator(double maxVx, double maxVy, double maxOmega, double accVx, double accVy, double accOmega, double dt, double predictTime, double robotRadius) {
        this.maxVx = maxVx;
        this.maxVy = maxVy;
        this.maxOmega = maxOmega;
        this.accVx = accVx;
        this.accVy = accVy;
        this.accOmega = accOmega;
        this.dt = dt;
        this.predictTime = predictTime;
        this.robotRadius = robotRadius;
    }

    /**
     * @param state สถานะปัจจุบันของหุ่นยนต์
     * @param goalX จุดเป้าหมาย x
     * @param goalY จุดเป้าหมาย y
     * @param obstacles รายการสิ่งกีดขวาง (แต่ละอันเป็น double[]{x, y})
     * @return Velocity ที่ดีที่สุด vx, vy, omega
     */
    public Velocity plan(RobotState state, double goalX, double goalY, List<double[]> obstacles) {
      double bestScore = Double.NEGATIVE_INFINITY;
      Velocity bestVel = new Velocity(0, 0, 0);
  
      double dynamicRobotRadius = robotRadius + Math.max(front_protech, Math.max(left_protech, right_protech));
  
      List<double[]> filteredObstacles = new ArrayList<>();
      for (double[] obs : obstacles) {
          double angleToObs = Math.toDegrees(Math.atan2(obs[1] - state.y, obs[0] - state.x));
          if (angleToObs >= FRONT_MIN_ANGLE_LEFT && angleToObs <= FRONT_MAX_ANGLE_RIGHT) {
              filteredObstacles.add(obs);
          }
      }
  
      double minDist = Double.POSITIVE_INFINITY;
      double goalAngle = Math.toDegrees(Math.atan2(goalY - state.y, goalX - state.x));
      boolean collision = false;
  
      double obsCount = filteredObstacles.size();
  
      double headingScore = 0;
      double velocityScore = 0;
  
      for (double vx = Math.max(-maxVx, state.vx - accVx * dt); vx <= Math.min(maxVx, state.vx + accVx * dt); vx += 0.1) {
          for (double vy = Math.max(-maxVy, state.vy - accVy * dt); vy <= Math.min(maxVy, state.vy + accVy * dt); vy += 0.1) {
              for (double omega = Math.max(-maxOmega, state.omega - accOmega * dt); omega <= Math.min(maxOmega, state.omega + accOmega * dt); omega += 0.1) {
                  RobotState simState = new RobotState(state.x, state.y, state.theta, vx, vy, omega);
                  double localMinDist = Double.POSITIVE_INFINITY;
  
                  for (double t = 0; t < predictTime; t += dt) {
                      simState.x += vx * Math.cos(simState.theta) * dt - vy * Math.sin(simState.theta) * dt;
                      simState.y += vx * Math.sin(simState.theta) * dt + vy * Math.cos(simState.theta) * dt;
                      simState.theta += omega * dt;
  
                      for (double[] obs : filteredObstacles) {
                          double dist = Math.hypot(simState.x - obs[0], simState.y - obs[1]);
                          if (dist < localMinDist) localMinDist = dist;
                      }
                  }
  
                  if (localMinDist < dynamicRobotRadius + 0.1) {
                      collision = true;
                      continue;
                  }
  
                  headingScore = 1.0 / (Math.hypot(simState.x - goalX, simState.y - goalY) + 1e-6);
                  velocityScore = Math.hypot(vx, vy);
  
                  double score = 0.6 * headingScore + 0.3 * velocityScore + 0.1 * localMinDist;
  
                  if (score > bestScore) {
                      bestScore = score;
                      bestVel = new Velocity(vx, vy, omega);
                      minDist = localMinDist;
                  }
              }
          }
      }
  
      // อัปเดตค่าไปยัง Shuffleboard
      DWA_VX.setDouble(bestVel.vx);
      DWA_VY.setDouble(bestVel.vy);
      DWA_OMEGA.setDouble(bestVel.omega);
      DWA_BEST_SCORE.setDouble(bestScore);
      DWA_MIN_DIST.setDouble(minDist);
      DWA_GOAL_ANGLE.setDouble(goalAngle);
      DWA_OBS_COUNT.setDouble(obsCount);
      DWA_COLLISION.setBoolean(collision);
      DWA_DRIVE_TYPE.setString(TypeDrive());
  
      return bestVel;
  }

	public double[] getMovementValues() {
		return null;
	}

	public double[] protec() {
		return null;
	}
}