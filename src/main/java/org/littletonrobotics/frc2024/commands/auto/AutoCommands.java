package org.littletonrobotics.frc2024.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.littletonrobotics.frc2024.RobotState;

public class AutoCommands {

    public static boolean inRegion(Supplier<Region> region) {
        return region.get().contains(RobotState.getInstance().getEstimatedPose().getTranslation());
    }

    public static Command waitForRegion(Supplier<Region> region) {
        return Commands.waitUntil(() -> inRegion(region));
    }

    public interface Region {
        boolean contains(Translation2d point);
    }

    public static class RectangularRegion implements Region {
        public final Translation2d topLeft;
        public final Translation2d bottomRight;

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
