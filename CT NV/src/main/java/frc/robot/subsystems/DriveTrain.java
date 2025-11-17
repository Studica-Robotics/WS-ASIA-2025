package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.Lidar;
import com.studica.frc.TitanQuad;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {

  // ===== 調整參數 =====
  private static final boolean DEBUG = true;
  // 多數 Studica Lidar distance 為 mm；若你的距離已是 m，改成 1.0
  private static final double K_TO_METERS = 0.001;
  private static final double FRONT_WINDOW_DEG = 20.0;
  private static final double DEFAULT_MAX_RANGE_M = 6.0;
  // ===== 硬體 =====
  private final TitanQuad frontLeftMotor  = new TitanQuad(Constants.TITAN_ID, Constants.FRONT_LEFT_MOTOR);
  private final TitanQuad backLeftMotor   = new TitanQuad(Constants.TITAN_ID, Constants.BACK_LEFT_MOTOR);
  private final TitanQuad frontRightMotor = new TitanQuad(Constants.TITAN_ID, Constants.FRONT_RIGHT_MOTOR);
  private final TitanQuad backRightMotor  = new TitanQuad(Constants.TITAN_ID, Constants.BACK_RIGHT_MOTOR);

  private final AHRS navX = new AHRS(SPI.Port.kMXP);

  private final ASDFG ASDFG = new ASDFG(ASDFG.Port.kUSB1);
  private ASDFG.ScanData scanData = new ASDFG.ScanData();
  public boolean lidarRunning = true;

  private final DigitalInput startButton  = new DigitalInput(Constants.START_BUTTON);
  private final DigitalInput eStopButton  = new DigitalInput(Constants.E_STOP_SWITCH);
  private final DigitalOutput runningLED  = new DigitalOutput(Constants.RUNNING_LED);
  private final DigitalOutput stoppedLED  = new DigitalOutput(Constants.STOPPED_LED);

  // ===== 狀態 =====
  private double denomonator = 0.0; // 沿用你 C++ 命名
  private double lidarAngleOffsetDeg = 180.0; // 與 C++ 預設相同

  // ===== 建構子 =====
  public DriveTrain() {
    // 讓 Titan 有時間初始化
    Timer.delay(1.0);

    // Motor invert（依 C++）
    frontLeftMotor.setInverted(false);
    backLeftMotor.setInverted(false);
    frontRightMotor.setInverted(true);
    backRightMotor.setInverted(true);

    ResetYaw();
  }

  // ===================== 介面：與 C++ 名稱對齊 =====================

  public void SetLidarAngleOffsetDeg(double deg){ lidarAngleOffsetDeg = deg; }
  public double GetLidarAngleOffsetDeg() { return lidarAngleOffsetDeg; }

  public double GetYaw() { return navX.getYaw(); }      // ±180°
  public double GetAngle() { return navX.getAngle(); }  // 連續角

  public void ResetYaw() { navX.zeroYaw(); }

  public boolean GetStartButton() { return startButton.get(); }
  public boolean GetEStopButton() { return eStopButton.get(); }

  public void SetRunningLED(boolean on) { runningLED.set(on); }
  public void SetStoppedLED(boolean on) { stoppedLED.set(on); }

  /** 注意：與你 C++ 相同——名字/行為是反的（保留原樣避免破壞流程）。 */
  public void LidarStop() {
    if (!lidarRunning) {
      lidar.start();
      lidarRunning = true;
    }
  }
  public void LidarStart() {
    if (lidarRunning) {
      lidar.stop();
      lidarRunning = false;
    }
  }
  // 左側最近距離（m）：掃描 -120° 到 -60° 的扇區
  public double LidarLeftMinMeters() {
    if (!lidarRunning) return Double.POSITIVE_INFINITY;
    return LidarSectorMinMeters(-80.0, -5.0);
  }

  // 右側最近距離（m）：掃描 +60° 到 +120°
  public double LidarRightMinMeters() {
    if (!lidarRunning) return Double.POSITIVE_INFINITY;
    return LidarSectorMinMeters(+5.0, +80.0);
  }
  
  // ===================== 馬達控制（對齊 C++） =====================

  public void StackMotorControl(double x, double y) {
    frontLeftMotor.set (y + x);
    backLeftMotor.set  (y + x);
    frontRightMotor.set(y - x);
    backRightMotor.set (y - x);
  }

  public void TwoWheelMotorControl(double x, double y) {
    frontLeftMotor.set (y + x);
    frontRightMotor.set(y - x);
  }
  
  public void TwoMotorControl(double x, double y) {
    frontLeftMotor.set (-y + x);
    frontRightMotor.set(-y - x);
  }

  public void SixWheelMotorControl(double x, double y) {
    frontLeftMotor.set (y + x);
    backLeftMotor.set  (y + x);
    frontRightMotor.set(y - x);
    backRightMotor.set (y - x);
  }

  public void MecanumMotorControl(double x, double y, double z) {
    denomonator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(z), 1.0);
    frontLeftMotor.set ( y + (x) + z / denomonator );
    backLeftMotor.set  ( y - (x) + z / denomonator );
    frontRightMotor.set( y - (x) - z / denomonator );
    backRightMotor.set ( y + (x) - z / denomonator );
  }

  public void XBotMotorControl(double x, double y, double z) {
    denomonator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(z), 1.0);
    frontLeftMotor.set ( y + (x) + z / denomonator );
    backLeftMotor.set  ( y - (x) + z / denomonator );
    frontRightMotor.set( y - (x) - z / denomonator );
    backRightMotor.set ( y + (x) - z / denomonator );
  }

  public void stopAllMotors(){
    frontLeftMotor.set(0); backLeftMotor.set(0);
    frontRightMotor.set(0); backRightMotor.set(0);
  }

  // ===================== LiDAR 幾何工具（對齊 C++ 行為） =====================

  private static double norm180(double a) {
    double x = (a + 180.0) % 360.0;
    if (x < 0) x += 360.0;
    return x - 180.0;
  }
  private static boolean inRange180(double deg, double L, double R) {
    if (L <= R) return deg >= L && deg <= R;
    return (deg >= L) || (deg <= R); // wrap case
  }
  private static double deg2rad(double d){ return d * Math.PI / 180.0; }
  private static double clamp(double v,double lo,double hi){ return Math.max(lo, Math.min(hi, v)); }

  private static double sectorAvgMeters(Lidar.ScanData sd,
                                        double leftDeg, double rightDeg,
                                        double angleOffsetDeg,
                                        double maxM) {
    final double L = norm180(leftDeg);
    final double R = norm180(rightDeg);
    double sum = 0.0;
    int cnt = 0;
    for (int i=0;i<360;i++){
      double a = norm180(sd.angle[i] + angleOffsetDeg);
      double d = sd.distance[i];
      if(!Double.isFinite(a) || !Double.isFinite(d) || d<=0) continue;
      double dm = d * K_TO_METERS;
      if (dm > maxM) continue;
      if (inRange180(a, L, R)) { sum += dm; cnt++; }
    }
    if (cnt==0) return Double.POSITIVE_INFINITY;
    return sum / cnt;
  }

  public double LidarSectorMinMeters(double degL, double degR) {
    if (!lidarRunning) return Double.POSITIVE_INFINITY;
    final double off = GetLidarAngleOffsetDeg();
    final double L = norm180(degL);
    final double R = norm180(degR);
    double best = Double.POSITIVE_INFINITY;
    for (int i=0;i<360;i++){
      double a = norm180(scanData.angle[i] + off);
      double d = scanData.distance[i];
      if(!Double.isFinite(a) || !Double.isFinite(d) || d<=0) continue;
      if (inRange180(a, L, R)) {
        double dm = d * K_TO_METERS;
        if (dm < best) best = dm;
      }
    }
    return best;
  }

  public double LidarFrontMinMeters() {
    if (!lidarRunning) return Double.POSITIVE_INFINITY;
    final double off = GetLidarAngleOffsetDeg();
    double frontMin = Double.POSITIVE_INFINITY;
    for (int i=0;i<360;i++){
      double a = norm180(scanData.angle[i] + off);
      double d = scanData.distance[i];
      if(!Double.isFinite(a) || !Double.isFinite(d) || d<=0) continue;
      if (a >= -FRONT_WINDOW_DEG && a <= +FRONT_WINDOW_DEG) {
        double dm = d * K_TO_METERS;
        if (dm < frontMin) frontMin = dm;
      }
    }
    return frontMin;
  }

  public boolean LidarHasTarget() {
    if (!lidarRunning) return false;
    final double off = GetLidarAngleOffsetDeg();
    double leftAvg  = sectorAvgMeters(scanData, -90.0, -10.0, off, DEFAULT_MAX_RANGE_M);
    double rightAvg = sectorAvgMeters(scanData, +10.0, +90.0, off, DEFAULT_MAX_RANGE_M);
    return Double.isFinite(leftAvg) && Double.isFinite(rightAvg);
  }

  /** 右更空 → 正（-1..+1） */
  public double LidarHorizontalOffsetNorm() {
    if (!lidarRunning) return 0.0;
    final double off = GetLidarAngleOffsetDeg();
    double leftAvg  = sectorAvgMeters(scanData, -90.0, -10.0, off, DEFAULT_MAX_RANGE_M);
    double rightAvg = sectorAvgMeters(scanData, +10.0, +90.0, off, DEFAULT_MAX_RANGE_M);
    if (!(Double.isFinite(leftAvg) && Double.isFinite(rightAvg))) return 0.0;
    double denom = leftAvg + rightAvg;
    if (denom <= 1e-6) return 0.0;
    return clamp((rightAvg - leftAvg) / denom, -1.0, 1.0);
  }

  /**
   * 在 ±120° 內以 5° 小扇區掃描，若落在車寬 halfWidthM 內，回報距車頭淨空（扣 frontMarginM）
   */
// DriveTrain.java
    public double LidarForwardCorridorClearance(double halfWidthM, double maxM, double frontMarginM) {
    final double kFov = 20.0;           // 只看 ±30°
    final double kStep = 5.0;
    final double kMinProjCos = 0.5;     // 只接受 |a| ≤ 60° (cos ≥ 0.5) 的前向投影
    final double kMinUseful = 0.15;     // 忽略過近的雜訊/自車反射

    double bestClear = maxM;

    for (double a = -kFov; a <= +kFov; a += kStep) {
        double r = LidarSectorMinMeters(a, a + kStep); // 你原有的方法，單位應為「公尺」
        if (!Double.isFinite(r) || r <= kMinUseful) continue;

        double c = Math.cos(Math.toRadians(a));
        if (c < kMinProjCos) continue;  // 斜向投影太小不採計

        double s = Math.sin(Math.toRadians(a));
        double x = r * c;
        double y = r * s;

        if (Math.abs(y) <= halfWidthM) {
            double clear = Math.max(0.0, x - frontMarginM);
            bestClear = Math.min(bestClear, clear);
        }
    }

    // 若完全沒命中，回傳 maxM（通暢），而不是 0
    return Math.max(0.0, Math.min(bestClear, maxM));
}


  // ===================== 週期 =====================
  @Override
  public void periodic() {
    if (lidarRunning) {
      scanData = lidar.getData();
      // A) 發佈 360 點（可搭配自製視覺化）
      double[] dist = new double[360];
      double[] ang  = new double[360];
      for (int i=0;i<360;i++){
        dist[i] = (double) scanData.distance[i];
        ang[i]  = (double) scanData.angle[i];
      }
      NetworkTable table = NetworkTableInstance.getDefault().getTable("Lidar");
      table.getEntry("distance").setDoubleArray(dist);
      table.getEntry("angle").setDoubleArray(ang);

      // B) 常用監看點（raw）
      double frontRawM = LidarFrontMinMeters();
      SmartDashboard.putNumber("TEST.FrontRawM", frontRawM);
      SmartDashboard.putNumber("LiDAR Raw[0] (deg 0/front)",    scanData.distance[0]);
      SmartDashboard.putNumber("LiDAR Raw[90] (deg +90/left)",  scanData.distance[90]);
      SmartDashboard.putNumber("LiDAR Raw[180] (back)",         scanData.distance[180]);
      SmartDashboard.putNumber("LiDAR Raw[270] (deg -90/right)",scanData.distance[270]);
    
      // C) 前方最小距離（±20°，含角度偏移）
      SmartDashboard.putNumber("LiDAR FrontMin (m, ±20°)", LidarFrontMinMeters());
    }

    if (DEBUG) {
      SmartDashboard.putNumber("Yaw",   GetYaw());
      SmartDashboard.putNumber("Angle", GetAngle());
    }
  }
}
