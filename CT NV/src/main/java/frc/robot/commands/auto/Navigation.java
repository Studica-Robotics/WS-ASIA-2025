package frc.robot.commands.auto;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

/**
 * LiDAR-based obstacle-avoidance navigator (Java port).
 * 功能等同 C++：前方距離膨脹、走廊淨空、近/清狀態穩定器、水平偏移轉向、停止/慢行/巡航。
 * 來源：LidarNavigation.cpp/.h
 */
public class Navigation extends CommandBase {

  // ===== Constants（完整對齊 C++） =====
  private static final double kCruiseSpeed = 0.3;     // 正常前進速度
  private static final double kSlowSpeed   = 0.3;     // 減速前進
  private static final double kStopDistM   = 0.01;     // 距離小於即停止
  private static final double kSlowDistM   = 1.0;     // 距離小於即減速
  private static final double kTurnGain    = 0.5;     // 轉向增益
  private static final double kMaxOmega    = 0.4;     // 最大轉向量

  // 原檔常數為 0.5，保留原行為（int 計數與 double 門檻比較）
  private static final double kNearFramesRequired  = 2;
  private static final double kClearFramesRequired = 2;

  private static final double kRobotWidthM    = 0.35; // 車寬
  private static final double kSideInflationM = 0.25; // 側邊安全膨脹
  private static final double kFrontMarginM   = 0.20; // 走廊前方保險距

  // 正前方距離膨脹（LiDAR->保險桿 + 額外安全）
  private static final double kLidarToFrontBumperM = 0.20;
  private static final double kSafetyInflationM    = 0.10;
  private static final double kInflateFrontM       = kLidarToFrontBumperM + kSafetyInflationM;
  //彎中補油
  private static final double kCenterNearM   = 0.50; // 只有靠牆近才啟動置中
  private static final double kCenterDeadband= 0.05; // 左右差很小就不動
  private static final double kWallRepel     = 0.25; // 0.2~0.35 區間微調
  private static final double kRepelLimit    = 0.15; // 單次最大修正量上限(降低蛇行)

  final double kTJuncFrontM   = 0.55;  // 前方 < 55cm 視為撞牆
  final double kTJuncCorrM    = 0.40;  // 走廊 < 55cm 視為入口狹窄
  final double kTJuncGapMin   = 0.50;  // 左右牆距差 > 50cm 才判斷成 T 口
  final double kLeftBiasOmega = 0.18;  // 左手偏壓大小（0.12~0.22 試）
  // ===== State =====
  private final DriveTrain drive;
  public int nearFrames  = 0;  // 與 .h 一致
  public int clearFrames = 0;  // 與 .h 一致

  public Navigation(DriveTrain drive) {
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
    double repel = 0.0;
    double left  = drive.LidarLeftMinMeters();
    double right = drive.LidarRightMinMeters();

    boolean haveSides = Double.isFinite(left) && Double.isFinite(right);
    if (haveSides) {
      double nearSide = Math.min(left, right);
      double diff = Math.abs(left - right);

      // 只有當「真的靠牆」且「左右明顯不對稱」才介入，避免大空地亂抖
      if (nearSide < kCenterNearM && diff > kCenterDeadband) {
        // 1/d 權重：越靠近修正越大（但下面會再夾限）
        repel = kWallRepel * ((1.0 / left) - (1.0 / right));

        // 依當前轉彎量稍微降權：大轉彎時少介入，避免和目標轉向打架
        double turnFactor = Math.min(1.0, Math.abs(omega) / kMaxOmega);
        double blend = 1.0 - 0.5 * turnFactor; // 大轉彎時只保留 50% 力量
        repel *= blend;

        // 夾限，避免蛇行
        if (repel >  +kRepelLimit) repel = +kRepelLimit;
        if (repel <  -kRepelLimit) repel = -kRepelLimit;

    omega = clamp(omega + repel, -kMaxOmega, kMaxOmega);
      }
    // G) 輸出到底盤（沿用 C++ 介面：x=omega, y=v）
    boolean frontBlocked = Double.isFinite(distFront) && distFront < kTJuncFrontM;
    boolean corridorNarrow = Double.isFinite(corridorClear) && corridorClear < kTJuncCorrM;
    boolean sidesValid = Double.isFinite(left) && Double.isFinite(right);
    boolean clearGap = sidesValid && Math.abs(left - right) > kTJuncGapMin;

    if (frontBlocked && corridorNarrow && clearGap) {
      // 你的約定：右更空→offset 為正→omega 正 = 右轉
      // 我們要“摸左邊走”，所以給「往左」的負偏壓
      omega = clamp(omega - kLeftBiasOmega, -kMaxOmega, kMaxOmega);
  edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("LN.TJunctionBias", true);
} else {
  edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putBoolean("LN.TJunctionBias", false);
}
    drive.TwoMotorControl(omega, v);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.distRaw", distRaw);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.distFront", distFront);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.corridor", corridorClear);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.dist", dist);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.v", v);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.omega", omega);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.leftSideMin", left);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.rightSideMin", right);
    edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.putNumber("LN.repel", repel);
  }
  }
  @Override
  public void end(boolean interrupted) {
    drive.TwoMotorControl(0.0, 0.0);
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
