package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.subsystems.superstructure.intake.Intake;
import java.util.function.Supplier;

public class AutoCommands {

  public static Command waitForRegion(Supplier<Region> region) {
    return Commands.waitUntil(
        () -> region.get().contains(RobotState.getInstance().getEstimatedPose().getTranslation()));
  }

  public static Command intakeWhileInRegion(Intake intake, Region region) {
    return Commands.sequence(
        waitForRegion(() -> region),
        intake.runCommand(),
        Commands.waitUntil(
            () -> !region.contains(RobotState.getInstance().getEstimatedPose().getTranslation())),
        intake.stopCommand());
  }

  public interface Region {
    boolean contains(Translation2d point);
  }

  public static class RectangularRegion implements Region {
    private final Translation2d topLeft;
    private final Translation2d bottomRight;

    public RectangularRegion(Translation2d topLeft, Translation2d bottomRight) {
      this.topLeft = topLeft;
      this.bottomRight = bottomRight;
    }

    public RectangularRegion(Translation2d center, double width, double height) {
      topLeft = new Translation2d(center.getX() - width / 2, center.getY() + height / 2);
      bottomRight = new Translation2d(center.getX() + width / 2, center.getY() - height / 2);
    }

    public boolean contains(Translation2d point) {
      return point.getX() >= topLeft.getX()
          && point.getX() <= bottomRight.getX()
          && point.getY() <= topLeft.getY()
          && point.getY() >= bottomRight.getY();
    }
  }

  public record CircularRegion(Translation2d center, double radius) implements Region {
    public boolean contains(Translation2d point) {
      return center.getDistance(point) <= radius;
    }
  }
}
