package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DepthCamera;

public class AutoCommand extends SequentialCommandGroup {
  public AutoCommand(DriveTrain drive, DepthCamera camera) {
    addCommands(
        // 若要用深度相機導航，改成：new Navigation(drive, camera).withTimeout(600.0)
        new Navigation(drive).withTimeout(600.0) // 600 秒 = 10 分鐘
    );
  }
}
