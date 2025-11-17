package frc.robot.commands.driveCommands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/**
 * LiDAR-based obstacle-avoidance navigator (Java port).
 * 功能等同 C++：前方距離膨脹、走廊淨空、近/清狀態穩定器、水平偏移轉向、停止/慢行/巡航。
 * 來源：LidarNavigation.cpp/.h
 */
public class LidarNavigation extends CommandBase {

  // ===== Constants（完整對齊 C++） =====
  private static final double kCruiseSpeed = 0.4;     // 正常前進速度
  private static final double kSlowSpeed   = 0.3;     // 減速前進
  private static final double kStopDistM   = 0.3;     // 距離小於即停止
  private static final double kSlowDistM   = 1.0;     // 距離小於即減速
  private static final double kTurnGain    = 1.0;     // 轉向增益
  private static final double kMaxOmega    = 1.0;     // 最大轉向量

  // 原檔常數為 0.5，保留原行為（int 計數與 double 門檻比較）
  private static final double kNearFramesRequired  = 0.5;
  private static final double kClearFramesRequired = 0.5;

  private static final double kRobotWidthM    = 0.65; // 車寬
  private static final double kSideInflationM = 0.35; // 側邊安全膨脹
  private static final double kFrontMarginM   = 0.25; // 走廊前方保險距

  // 正前方距離膨脹（LiDAR->保險桿 + 額外安全）
  private static final double kLidarToFrontBumperM = 0.30;
  private static final double kSafetyInflationM    = 0.10;
  private static final double kInflateFrontM       = kLidarToFrontBumperM + kSafetyInflationM;

  // ===== State =====
  private final DriveTrain drive;
  public int nearFrames  = 0;  // 與 .h 一致
  public int clearFrames = 0;  // 與 .h 一致

  public LidarNavigation(DriveTrain drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    nearFrames = 0;
    clearFrames = 0;
    drive.ResetYaw();
  }

  @Override
  public void execute() {
    // A) 正前方距離（加上前向膨脹）
    double distRaw = drive.LidarFrontMinMeters();
    double distFront = Double.POSITIVE_INFINITY;
    if (Double.isFinite(distRaw)) {
      distFront = Math.max(0.0, distRaw - kInflateFrontM);
    }

    // B) 走廊淨空（含車寬 + 側邊膨脹）
    double corridorHalf = 0.5 * kRobotWidthM + kSideInflationM;
    double corridorClear = drive.LidarForwardCorridorClearance(corridorHalf, /*maxM*/ 6.0, kFrontMarginM);

    // C) 取更危險者作為有效距離
    double dist = Math.min(distFront, corridorClear);

    // D) 近/清狀態穩定器
    boolean near = Double.isFinite(dist) && (dist <= kSlowDistM);
    if (near) {
      nearFrames += 1;
      clearFrames = 0;
    } else {
      clearFrames += 1;
      nearFrames = 0;
    }
    boolean nearStable  = (nearFrames  >= kNearFramesRequired);
    boolean clearStable = (clearFrames >= kClearFramesRequired);

    // E) 線速度：停 / 慢 / 巡航
    double v = kCruiseSpeed;
    if (nearStable) {
      v = (dist <= kStopDistM) ? 0.0 : kSlowSpeed;
    } else if (!clearStable) {
      v = 0.5 * (kCruiseSpeed + kSlowSpeed); // 抖動期間取較安全中間值
    }

    // F) 角速度：有目標偏移則修正，否則直行
    double omega = 0.0;
    if (drive.LidarHasTarget()) {
      double offset = clamp(drive.LidarHorizontalOffsetNorm(), -1.0, 1.0);
      omega = clamp(offset * kTurnGain, -kMaxOmega, kMaxOmega);
      if (Double.isFinite(dist) && dist < 1.0) {
        omega *= 1.0; // 保留原檔語意
      }
    }

    // G) 輸出到底盤（沿用 C++ 介面：x=omega, y=v）
    drive.TwoWheelMotorControl(omega, v);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.distRaw", distRaw);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.distFront", distFront);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.corridor", corridorClear);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.dist", dist);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.v", v);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.omega", omega);
  }

  @Override
  public void end(boolean interrupted) {
    drive.TwoWheelMotorControl(0.0, 0.0);
  }

  @Override
  public boolean isFinished() {
    return false; // 持續執行直到被中止
  }

  // ===== Utils =====
  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
