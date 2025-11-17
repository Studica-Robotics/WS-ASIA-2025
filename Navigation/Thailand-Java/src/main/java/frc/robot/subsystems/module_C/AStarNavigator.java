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

public class AStarNavigator {

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
    return Drive;
  }

  // private static final Differance driveType = RobotContainer.differance;
  // private static final XDrive driveType = RobotContainer.xDrive;
  private static final Mecanum driveType = RobotContainer.mecanum;
  // private static final TheStack driveType = RobotContainer.theStack;
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

  private final static ShuffleboardTab tab = Shuffleboard.getTab("AStar");

// ตำแหน่งเริ่มต้นและเป้าหมาย
private final NetworkTableEntry Start_X = tab.add("Start_X", 0).withPosition(0,0).getEntry();
private final NetworkTableEntry Start_Y = tab.add("Start_Y", 0).withPosition(1,0).getEntry();
private final NetworkTableEntry Goal_X = tab.add("Goal_X", 0).withPosition(2,0).getEntry();
private final NetworkTableEntry Goal_Y = tab.add("Goal_Y", 0).withPosition(3,0).getEntry();

// ความเร็วที่ใช้คำนวณ
private final NetworkTableEntry Speed_X = tab.add("Speed_X", 0).withPosition(0,1).getEntry();
private final NetworkTableEntry Speed_Y = tab.add("Speed_Y", 0).withPosition(1,1).getEntry();
private final NetworkTableEntry Speed_Z = tab.add("Speed_Z", 0).withPosition(2,1).getEntry();

// ผลลัพธ์การคำนวณ movement
private final NetworkTableEntry Cal_X = tab.add("Cal_X", 0).withPosition(0,2).getEntry();
private final NetworkTableEntry Cal_Y = tab.add("Cal_Y", 0).withPosition(1,2).getEntry();
private final NetworkTableEntry Cal_Z = tab.add("Cal_Z", 0).withPosition(2,2).getEntry();

// จำนวนจุดใน path
private final NetworkTableEntry Path_Size = tab.add("Path_Size", 0).withPosition(3,2).getEntry();

// ระยะและมุมของจุดถัดไป
private final NetworkTableEntry Next_Distance = tab.add("Next_Distance", 0).withPosition(0,3).getEntry();
private final NetworkTableEntry Next_Angle = tab.add("Next_Angle", 0).withPosition(1,3).getEntry();

// สถานะการหาเส้นทาง
private final NetworkTableEntry Path_Found = tab.add("Path_Found", false).withPosition(2,3).getEntry();

// ข้อมูล obstacle ที่ถูกเพิ่ม
private final NetworkTableEntry Obstacle_Count = tab.add("Obstacle_Count", 0).withPosition(3,3).getEntry();



    public static class Node implements Comparable<Node> {
        public int x, y;
        public double gCost, hCost;
        public Node parent;

        public Node(int x, int y, double gCost, double hCost, Node parent) {
            this.x = x;
            this.y = y;
            this.gCost = gCost;
            this.hCost = hCost;
            this.parent = parent;
        }

        public double getFCost() {
            return gCost + hCost;
        }

        @Override
        public int compareTo(Node other) {
            return Double.compare(this.getFCost(), other.getFCost());
        }

        @Override
        public boolean equals(Object obj) {
            if (!(obj instanceof Node)) return false;
            Node other = (Node) obj;
            return this.x == other.x && this.y == other.y;
        }

        @Override
        public int hashCode() {
            return Objects.hash(x, y);
        }
    }

    // Heuristic: Euclidean distance
    private double heuristic(int x1, int y1, int x2, int y2) {
        return Math.hypot(x1 - x2, y1 - y2);
    }

    public double[] calculateAStarMovement(int[][] grid, int startX, int startY, int goalX, int goalY, double speed_x, double speed_y, double speed_z) {
      // ส่งค่าพารามิเตอร์ไปยัง Shuffleboard
      Start_X.setDouble(startX);
      Start_Y.setDouble(startY);
      Goal_X.setDouble(goalX);
      Goal_Y.setDouble(goalY);
  
      Speed_X.setDouble(speed_x);
      Speed_Y.setDouble(speed_y);
      Speed_Z.setDouble(speed_z);
  
      int rows = grid.length;
      int cols = grid[0].length;
  
      int obstacleCount = 0;
  
      for (int y = 0; y < rows; y++) {
          for (int x = 0; x < cols; x++) {
              double angle = Math.toDegrees(Math.atan2(y - startY, x - startX));
              double dist = Math.hypot(x - startX, y - startY);
              if (dist < block_distance_Front && angle >= FRONT_MIN_ANGLE_LEFT && angle <= FRONT_MAX_ANGLE_RIGHT) {
                  grid[y][x] = 1;
                  obstacleCount++;
              }
              if (dist < block_distance_Left && angle >= LEFT_MIN_ANGLE_LEFT && angle <= LEFT_MAX_ANGLE_RIGHT) {
                  grid[y][x] = 1;
                  obstacleCount++;
              }
              if (dist < block_distance_Right && angle >= RIGHT_MIN_ANGLE_LEFT && angle <= RIGHT_MAX_ANGLE_RIGHT) {
                  grid[y][x] = 1;
                  obstacleCount++;
              }
          }
      }
      Obstacle_Count.setDouble(obstacleCount);
  
      List<int[]> path = findPath(grid, startX, startY, goalX, goalY);
  
      Path_Size.setDouble(path.size());
      Path_Found.setBoolean(path.size() > 1);
  
      double cal_x = 0.0;
      double cal_y = 0.0;
      double cal_z = 0.0;
      double nextDist = 0.0;
      double nextAngle = 0.0;
  
      if (path.size() > 1) {
          int[] next = path.get(1);
          int dx = next[0] - startX;
          int dy = next[1] - startY;
  
          cal_x = dx * speed_x;
          cal_y = dy * speed_y;
  
          nextDist = Math.hypot(dx, dy);
          nextAngle = Math.toDegrees(Math.atan2(dy, dx));
  
          if (nextDist < front_protech) {
              cal_z = speed_z * 0.5 * Math.signum(dx);
          } else if (dx != 0 && dy != 0) {
              cal_z = speed_z * Math.signum(dx);
          }
      }
  
      Cal_X.setDouble(cal_x);
      Cal_Y.setDouble(cal_y);
      Cal_Z.setDouble(cal_z);
  
      Next_Distance.setDouble(nextDist);
      Next_Angle.setDouble(nextAngle);
  
      return new double[] { cal_x, cal_y, cal_z };
  }

    public List<int[]> findPath(int[][] grid, int startX, int startY, int goalX, int goalY) {
        int rows = grid.length;
        int cols = grid[0].length;

        PriorityQueue<Node> openSet = new PriorityQueue<>();
        Set<Node> closedSet = new HashSet<>();

        Node startNode = new Node(startX, startY, 0, heuristic(startX, startY, goalX, goalY), null);
        openSet.add(startNode);

        while (!openSet.isEmpty()) {
            Node current = openSet.poll();

            if (current.x == goalX && current.y == goalY) {
                // Path found
                List<int[]> path = new ArrayList<>();
                while (current != null) {
                    path.add(0, new int[]{current.x, current.y});
                    current = current.parent;
                }
                return path;
            }

            closedSet.add(current);

            // 4 directions (up, down, left, right)
            int[][] directions = { {0,1}, {1,0}, {0,-1}, {-1,0} };
            for (int[] dir : directions) {
                int nx = current.x + dir[0];
                int ny = current.y + dir[1];

                if (nx < 0 || ny < 0 || nx >= cols || ny >= rows) continue;
                if (grid[ny][nx] == 1) continue; // Obstacle

                Node neighbor = new Node(nx, ny, current.gCost + 1, heuristic(nx, ny, goalX, goalY), current);

                if (closedSet.contains(neighbor)) continue;

                // If not in openSet or found better path
                boolean better = true;
                for (Node n : openSet) {
                    if (n.equals(neighbor) && n.gCost <= neighbor.gCost) {
                        better = false;
                        break;
                    }
                }
                if (better) openSet.add(neighbor);
            }
        }
        // No path found
        return new ArrayList<>();

        
    }

	public double[] getMovementValues() {
		return null;
	}

	public double[] protec() {
		return null;
	}
}