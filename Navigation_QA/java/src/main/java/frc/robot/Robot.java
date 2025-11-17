package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;
import com.studica.frc.TitanQuad;
import com.studica.frc.Lidar;

import java.util.Arrays;

public class Robot extends TimedRobot {

  // ====== DRIVE (TitanQuad channels M0..M3) ======
  private static final int TITAN_CAN_ID   = 42;
  private static final int LEFT_MOTOR_CH  = 0;  
  private static final int RIGHT_MOTOR_CH = 1;  
  private static final int MID_MOTOR_CH   = 3;  
  private static final int TEST_MOTOR_CH  = 2;  
  private static final boolean LEFT_INVERTED   = false;
  private static final boolean RIGHT_INVERTED  = true;
  private static final boolean MID_INVERTED    = false;
  private static final boolean TEST_INVERTED   = false;

  // Base speeds
  private static final double BASE_FWD_SPEED   = 0.40; // free-space cruise
  private static final double MIN_FWD_SPEED    = 0.15; // don’t stall when creeping
  private static final double TURN_SPEED_AVOID = 0.40; // spin speed in hard-avoid
  private static final double BACKUP_SPEED     = 0.30;
  private static final double RAMP_CLAMP       = 0.06; // per 20ms loop

  // ====== navX (optional) ======
  private static final boolean USE_NAVX = true;

  // ====== LiDAR (index-based) ======
  private static final double UNITS_SCALE_DEFAULT = 1.0; // 1.0 if cm; 0.1 if mm
  private static final int FRONT_CENTER_IDX_DEFAULT = 0; // try 0/90/180/270 to match mounting
  private static final int FRONT_HALF_WIDTH_IDX_DEFAULT = 45; // ± indices
  private static final int SIDE_HALF_WIDTH_IDX_DEFAULT  = 60;

  // Reject obvious junk
  private static final double MIN_VALID_CM_DEFAULT = 20.0;  // ignore near floor/bumper
  private static final double MAX_VALID_CM_DEFAULT = 800.0;

  // Smoothing (EMA on front median)
  private static final double FRONT_ALPHA_DEFAULT = 0.3;

  // === 3-zone behavior ===
  private static final double STOP_CM_DEFAULT    = 220.0;   // hard stop & avoid starts here
  private static final double CAUTION_CM_DEFAULT = 130.0;   // start slowing/steering here

  // Steering while moving
  private static final double K_TURN_DEFAULT        = 0.010; // per cm
  private static final double MAX_TURN_WHILE_MOVING = 0.40;

  // Caution speed scaling
  private static final double CAUTION_EXP_DEFAULT = 1.0;   // 1 = linear; >1 slower near STOP

  // Stuck handling in avoid mode
  private static final double SPIN_STUCK_TIMEOUT_S = 2.0;
  private static final double BACKUP_TIME_S        = 0.6;

  // ===== Mid (3rd) & Test (4th) wheel assist =====
  private static final double MID_ASSIST_GAIN   = 1.00;
  private static final double TEST_ASSIST_GAIN  = 1.00;
  private static final double MID_RAMP_CLAMP    = 0.06;
  private static final double TEST_RAMP_CLAMP   = 0.06;
  private static final boolean MID_DISABLE_WHEN_TURNING  = true;
  // We ignore the test "disable when turning" behavior; test wheel is always opposite of forward
  private static final boolean TEST_DISABLE_WHEN_TURNING = false;
  private static final double TURN_DISABLE_THRESH = 0.35; // if |L-R| > this, cut mid assist
  private static final boolean MID_ASSIST_ENABLED_DEFAULT  = true;
  private static final boolean TEST_ASSIST_ENABLED_DEFAULT = true;

  // ===== Hardware =====
  private TitanQuad leftMotor, rightMotor, midMotor, testMotor;
  private AHRS ahrs;
  private Joystick pad;
  private Lidar lidar;
  private boolean scanning = false;

  // ===== State =====
  private double lastL = 0, lastR = 0, lastM = 0, lastT = 0;
  private int lidarWarmupCycles = 0;

  private double frontFiltered = Double.POSITIVE_INFINITY;
  private int turnDir = -1;  // -1=right, +1=left

  private final Timer stateTimer = new Timer();

  private enum Mode { CRUISE, AVOID_TURN, BACKUP }
  private Mode mode = Mode.CRUISE;

  @Override
  public void robotInit() {
    leftMotor  = new TitanQuad(TITAN_CAN_ID, LEFT_MOTOR_CH);
    rightMotor = new TitanQuad(TITAN_CAN_ID, RIGHT_MOTOR_CH);
    midMotor   = new TitanQuad(TITAN_CAN_ID, MID_MOTOR_CH);
    testMotor  = new TitanQuad(TITAN_CAN_ID, TEST_MOTOR_CH); // test motor

    leftMotor.setInverted(LEFT_INVERTED);
    rightMotor.setInverted(RIGHT_INVERTED);
    midMotor.setInverted(MID_INVERTED);
    testMotor.setInverted(TEST_INVERTED);

    if (USE_NAVX) {
      try { ahrs = new AHRS(SPI.Port.kMXP); } catch (Throwable t) { ahrs = null; }
    }
    pad = new Joystick(0);

    try { lidar = new Lidar(Lidar.Port.kUSB1); scanning = true; }
    catch (Throwable t1) {
      try { lidar = new Lidar(Lidar.Port.kUSB2); scanning = true; }
      catch (Throwable t2) { scanning = false; }
    }

    // Tunables
    SmartDashboard.putNumber("UnitsScale_LidarToCm", UNITS_SCALE_DEFAULT);
    SmartDashboard.putNumber("FrontCenterIdx",       FRONT_CENTER_IDX_DEFAULT);
    SmartDashboard.putNumber("FrontHalfWidthIdx",    FRONT_HALF_WIDTH_IDX_DEFAULT);
    SmartDashboard.putNumber("SideHalfWidthIdx",     SIDE_HALF_WIDTH_IDX_DEFAULT);
    SmartDashboard.putNumber("MinValidCm",           MIN_VALID_CM_DEFAULT);
    SmartDashboard.putNumber("MaxValidCm",           MAX_VALID_CM_DEFAULT);
    SmartDashboard.putNumber("FrontAlpha",           FRONT_ALPHA_DEFAULT);

    SmartDashboard.putNumber("StopCm",               STOP_CM_DEFAULT);
    SmartDashboard.putNumber("CautionCm",            CAUTION_CM_DEFAULT);
    SmartDashboard.putNumber("K_Turn",               K_TURN_DEFAULT);
    SmartDashboard.putNumber("CautionExp",           CAUTION_EXP_DEFAULT);

    // Assist tunables
    SmartDashboard.putBoolean("MidAssistEnabled",    MID_ASSIST_ENABLED_DEFAULT);
    SmartDashboard.putNumber("MidAssistGain",        MID_ASSIST_GAIN);
    SmartDashboard.putBoolean("TestAssistEnabled",   TEST_ASSIST_ENABLED_DEFAULT);
    SmartDashboard.putNumber("TestAssistGain",       TEST_ASSIST_GAIN);
    SmartDashboard.putNumber("TurnDisableThresh",    TURN_DISABLE_THRESH);
  }

  @Override
  public void autonomousInit() {
    stopDrive();
    mode = Mode.CRUISE;
    lidarWarmupCycles = 0;
    stateTimer.reset(); stateTimer.start();
    frontFiltered = Double.POSITIVE_INFINITY;
    turnDir = -1;
    if (lidar != null) { try { lidar.start(); scanning = true; } catch (Exception e) { scanning = false; } }
    if (ahrs  != null) { try { ahrs.reset(); } catch (Exception e) {} }
  }

  @Override
  public void autonomousPeriodic() {
    // Live params
    final double unitsScale = SmartDashboard.getNumber("UnitsScale_LidarToCm", UNITS_SCALE_DEFAULT);
    final int    frontIdx   = wrap360((int) SmartDashboard.getNumber("FrontCenterIdx", FRONT_CENTER_IDX_DEFAULT));
    final int    frontHW    = (int) SmartDashboard.getNumber("FrontHalfWidthIdx", FRONT_HALF_WIDTH_IDX_DEFAULT);
    final int    sideHW     = (int) SmartDashboard.getNumber("SideHalfWidthIdx",  SIDE_HALF_WIDTH_IDX_DEFAULT);
    final double minValidCm = SmartDashboard.getNumber("MinValidCm", MIN_VALID_CM_DEFAULT);
    final double maxValidCm = SmartDashboard.getNumber("MaxValidCm", MAX_VALID_CM_DEFAULT);
    final double alphaFront = clamp(SmartDashboard.getNumber("FrontAlpha", FRONT_ALPHA_DEFAULT), 0, 1);

    final double stopCm     = SmartDashboard.getNumber("StopCm",    STOP_CM_DEFAULT);
    final double cautionCm  = SmartDashboard.getNumber("CautionCm", CAUTION_CM_DEFAULT);
    final double kTurn      = SmartDashboard.getNumber("K_Turn",    K_TURN_DEFAULT);
    final double cautionExp = Math.max(0.5, SmartDashboard.getNumber("CautionExp", CAUTION_EXP_DEFAULT));

    // Assist live params
    final boolean midAssistEnabled  = SmartDashboard.getBoolean("MidAssistEnabled",  MID_ASSIST_ENABLED_DEFAULT);
    final double  midGain           = SmartDashboard.getNumber("MidAssistGain",      MID_ASSIST_GAIN);
    final boolean testAssistEnabled = SmartDashboard.getBoolean("TestAssistEnabled", TEST_ASSIST_ENABLED_DEFAULT);
    final double  testGain          = SmartDashboard.getNumber("TestAssistGain",     TEST_ASSIST_GAIN);
    final double  turnCutThresh     = SmartDashboard.getNumber("TurnDisableThresh",  TURN_DISABLE_THRESH);

    // Read scan
    Lidar.ScanData scan = null;
    if (scanning && lidar != null) { try { scan = lidar.getData(); } catch (Throwable t) { scan = null; } }

    // Warm-up
    if (lidarWarmupCycles < 10) {
      lidarWarmupCycles++;
      setDriveSmoothFour(
        BASE_FWD_SPEED * 0.6, BASE_FWD_SPEED * 0.6,
        midAssistEnabled, midGain,
        testAssistEnabled, testGain,
        turnCutThresh
      );
      SmartDashboard.putBoolean("LiDARWarmup", true);
      publishZeros();
      return;
    } else {
      SmartDashboard.putBoolean("LiDARWarmup", false);
    }

    // Sectors (index-based)
    SectorStats front = sectorByIndex(scan, unitsScale, frontIdx,               frontHW, minValidCm, maxValidCm);
    SectorStats left  = sectorByIndex(scan, unitsScale, wrap360(frontIdx + 90), sideHW,  minValidCm, maxValidCm);
    SectorStats right = sectorByIndex(scan, unitsScale, wrap360(frontIdx - 90), sideHW,  minValidCm, maxValidCm);

    // Front values
    double frontMin    = (front.validCount > 0) ? front.minCm    : Double.POSITIVE_INFINITY;
    double frontMedian = (front.validCount > 0) ? front.medianCm : Double.POSITIVE_INFINITY;

    // Smooth median
    if (Double.isInfinite(frontFiltered)) frontFiltered = frontMedian;
    else frontFiltered = alphaFront * frontMedian + (1 - alphaFront) * frontFiltered;

    // Decide zone
    boolean inEmergency = frontMin < stopCm;          // hard stop & avoid
    boolean inCaution   = frontFiltered < cautionCm;  // start slowing/steering

    // FSM
    switch (mode) {
      case CRUISE: {
        if (inEmergency) {
          mode = Mode.AVOID_TURN;
          stateTimer.reset();
          stopDrive();
        } else {
          double fwd = BASE_FWD_SPEED;
          double turn = 0.0;
          if (inCaution) {
            double x = clamp((frontFiltered - stopCm) / Math.max(1.0, (cautionCm - stopCm)), 0.0, 1.0);
            x = Math.pow(x, cautionExp);
            fwd = MIN_FWD_SPEED + (BASE_FWD_SPEED - MIN_FWD_SPEED) * x;

            boolean leftOK  = left.validCount  > 0;
            boolean rightOK = right.validCount > 0;
            double diff = 0.0; // + -> turn right (more open on right)
            if (leftOK && rightOK)       diff = (right.medianCm - left.medianCm);
            else if (rightOK)            diff = +50.0;
            else if (leftOK)             diff = -50.0;
            turn = clamp(kTurn * diff, -MAX_TURN_WHILE_MOVING, +MAX_TURN_WHILE_MOVING);
          }
          double lCmd = clamp(fwd + turn, -1, 1);
          double rCmd = clamp(fwd - turn, -1, 1);

          setDriveSmoothFour(
            lCmd, rCmd,
            midAssistEnabled,  midGain,
            testAssistEnabled, testGain,
            turnCutThresh
          );
        }
        break;
      }

      case AVOID_TURN: {
        if (stateTimer.get() > SPIN_STUCK_TIMEOUT_S) {
          mode = Mode.BACKUP;
          stateTimer.reset();
          stopDrive();
          break;
        }
        if (left.validCount > 0 && right.validCount > 0) {
          turnDir = (left.medianCm > right.medianCm) ? +1 : -1;
        } else if (left.validCount > 0) {
          turnDir = +1;
        } else if (right.validCount > 0) {
          turnDir = -1;
        }
        if (!inEmergency && frontFiltered > (cautionCm + 10.0)) {
          mode = Mode.CRUISE;
          stopDrive();
        } else {
          // Spin in place — disable mid assist to avoid scrub (test motor remains opposite of forward=~0 -> ~0)
          setDriveSmooth(lCmdForTurn(turnDir), rCmdForTurn(turnDir));
        }
        break;
      }

      case BACKUP: {
        if (stateTimer.get() < BACKUP_TIME_S) {
          setDriveSmoothFour(
            -BACKUP_SPEED, -BACKUP_SPEED,
            midAssistEnabled,  midGain,
            testAssistEnabled, testGain,
            turnCutThresh
          );
        } else {
          mode = Mode.AVOID_TURN; // try spinning again
          stateTimer.reset();
          stopDrive();
        }
        break;
      }
    }

    // Telemetry
    SmartDashboard.putString("Mode", mode.toString());
    SmartDashboard.putNumber("FrontMin(cm)",      frontMin);
    SmartDashboard.putNumber("FrontMedian(cm)",   frontMedian);
    SmartDashboard.putNumber("FrontFiltered(cm)", frontFiltered);
    SmartDashboard.putNumber("FrontValid",        front.validCount);
    SmartDashboard.putNumber("LeftMedian(cm)",    left.medianCm);
    SmartDashboard.putNumber("LeftValid",         left.validCount);
    SmartDashboard.putNumber("RightMedian(cm)",   right.medianCm);
    SmartDashboard.putNumber("RightValid",        right.validCount);
    SmartDashboard.putNumber("Cmd_Left",          lastL);
    SmartDashboard.putNumber("Cmd_Right",         lastR);
    SmartDashboard.putNumber("Cmd_Mid",           lastM);
    SmartDashboard.putNumber("Cmd_Test",          lastT);
    if (ahrs != null) SmartDashboard.putNumber("Yaw", ahrs.getYaw());
  }

  private double lCmdForTurn(int dir) { return clamp(dir * TURN_SPEED_AVOID, -1, 1); }
  private double rCmdForTurn(int dir) { return clamp(-dir * TURN_SPEED_AVOID, -1, 1); }

  private void publishZeros() {
    SmartDashboard.putNumber("FrontMin(cm)", -1);
    SmartDashboard.putNumber("FrontMedian(cm)", -1);
    SmartDashboard.putNumber("FrontFiltered(cm)", -1);
    SmartDashboard.putNumber("FrontValid", 0);
    SmartDashboard.putNumber("LeftMedian(cm)", -1);
    SmartDashboard.putNumber("LeftValid", 0);
    SmartDashboard.putNumber("RightMedian(cm)", -1);
    SmartDashboard.putNumber("RightValid", 0);
    SmartDashboard.putNumber("Cmd_Left", 0);
    SmartDashboard.putNumber("Cmd_Right", 0);
    SmartDashboard.putNumber("Cmd_Mid", 0);
    SmartDashboard.putNumber("Cmd_Test", 0);
  }

  // ===== Teleop (for testing) =====
  @Override
  public void teleopInit() {
    if (lidar != null) try { lidar.start(); scanning = true; } catch (Exception e) { scanning = false; }
    stopDrive();
  }

  @Override
  public void teleopPeriodic() {
    double fwd = -pad.getRawAxis(1);
    double rot =  pad.getRawAxis(2);

    double lCmd = clamp((fwd + rot) * 0.6, -1, 1);
    double rCmd = clamp((fwd - rot) * 0.6, -1, 1);

    boolean midAssistEnabled  = SmartDashboard.getBoolean("MidAssistEnabled",  MID_ASSIST_ENABLED_DEFAULT);
    double  midGain           = SmartDashboard.getNumber("MidAssistGain",      MID_ASSIST_GAIN);
    boolean testAssistEnabled = SmartDashboard.getBoolean("TestAssistEnabled", TEST_ASSIST_ENABLED_DEFAULT);
    double  testGain          = SmartDashboard.getNumber("TestAssistGain",     TEST_ASSIST_GAIN);
    double  turnCutThresh     = SmartDashboard.getNumber("TurnDisableThresh",  TURN_DISABLE_THRESH);

    setDriveSmoothFour(
      lCmd, rCmd,
      midAssistEnabled,  midGain,
      testAssistEnabled, testGain,
      turnCutThresh
    );
  }

  @Override
  public void disabledInit() { stopDrive(); }

  // ===== LiDAR helpers =====
  private static class SectorStats {
    double minCm = Double.POSITIVE_INFINITY;
    double medianCm = Double.POSITIVE_INFINITY;
    int    validCount = 0;
  }

  /** Index-centered cone: collects ALL valid samples; returns min & median. */
  private SectorStats sectorByIndex(Lidar.ScanData scan, double unitsScale,
                                    int centerIdx, int halfWidthIdx,
                                    double minValidCm, double maxValidCm) {
    SectorStats s = new SectorStats();
    if (scan == null || scan.distance == null) return s;
    int N = scan.distance.length;
    double[] buf = new double[(halfWidthIdx * 2) + 1];
    int n = 0;
    for (int off = -halfWidthIdx; off <= halfWidthIdx; off++) {
      int idx = wrap360(centerIdx + off);
      if (idx < 0 || idx >= N) continue;
      float d = scan.distance[idx];
      if (d <= 0) continue;
      double cm = d * unitsScale;
      if (cm >= minValidCm && cm <= maxValidCm) buf[n++] = cm;
    }
    if (n > 0) {
      Arrays.sort(buf, 0, n);
      s.minCm    = buf[0];
      s.medianCm = (n & 1) == 1 ? buf[n/2] : 0.5 * (buf[n/2 - 1] + buf[n/2]);
      s.validCount = n;
    }
    return s;
  }

  private static int wrap360(int deg) { int d = deg % 360; if (d < 0) d += 360; return d; }

  // ===== Drive helpers =====

  // Low-level raw setter for all 4 motors
  private void setDrive(double l, double r, double m, double t) {
    leftMotor.set(l);
    rightMotor.set(r);
    midMotor.set(m);
    testMotor.set(t);
  }
  private void setDrive(double l, double r) { setDrive(l, r, 0, 0); }

  private void stopDrive() {
    lastL = 0; lastR = 0; lastM = 0; lastT = 0;
    setDrive(0, 0, 0, 0);
  }

  // Two-wheel smoothed drive (used for in-place spins)
  private void setDriveSmooth(double lCmd, double rCmd) {
    lCmd = rampToward(lastL, lCmd, RAMP_CLAMP);
    rCmd = rampToward(lastR, rCmd, RAMP_CLAMP);
    lastL = lCmd; lastR = rCmd;
    // during spins, mid/test outputs are 0; test motor ~0 since forward ≈ 0
    setDrive(lCmd, rCmd, 0, 0);
  }

  // Four-motor smoothed drive with mid assist
  // TEST motor is ALWAYS the opposite direction of the main forward component
  private void setDriveSmoothFour(
      double lCmd, double rCmd,
      boolean allowMid,  double midGain,
      boolean allowTest, double testGain,
      double turnDisableThresh) {

    // Smooth L/R
    lCmd = rampToward(lastL, lCmd, RAMP_CLAMP);
    rCmd = rampToward(lastR, rCmd, RAMP_CLAMP);
    lastL = lCmd; lastR = rCmd;

    // Forward component approximated by average of tank sides
    double fwd = 0.5 * (lCmd + rCmd);
    double turnMagnitude = Math.abs(lCmd - rCmd);

    // Mid motor command (assist forward unless turning sharply)
    double mCmd = 0.0;
    if (allowMid) {
      boolean disableForTurn = MID_DISABLE_WHEN_TURNING && (turnMagnitude > turnDisableThresh);
      if (!disableForTurn) mCmd = clamp(fwd * midGain, -1.0, 1.0);
    }
    mCmd = rampToward(lastM, mCmd, MID_RAMP_CLAMP);
    lastM = mCmd;

    // Test motor command: ALWAYS opposite of forward component
    double tCmd = 0.0;
    if (allowTest) {
      tCmd = clamp(-fwd * testGain, -1.0, 1.0);
    }
    tCmd = rampToward(lastT, tCmd, TEST_RAMP_CLAMP);
    lastT = tCmd;

    setDrive(lCmd, rCmd, mCmd, tCmd);
  }

  private static double rampToward(double cur, double tgt, double step) {
    double d = tgt - cur;
    if (Math.abs(d) > step) d = Math.copySign(step, d);
    return cur + d;
  }

  private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
}
