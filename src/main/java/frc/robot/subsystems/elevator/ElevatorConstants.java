package frc.robot.subsystems.elevator;

import frc.robot.constants.GlobalConstants;
import frc.robot.lib.Gains;

public class ElevatorConstants {

    public static final Gains kGains =
            switch (GlobalConstants.getRobotType()) {
                    // TODO: tune
                default -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0, 0);
            };

    public static final double kGearRatio =
            switch (GlobalConstants.getRobotType()) {
                    // TODO: tune
                default -> 5.0;
            };

    public static final MotionMagicConfigs kMotionMagicConfigs =
            switch (GlobalConstants.getRobotType()) {
                    // TODO: tune
                default -> new MotionMagicConfigs(0, 0, 0);
            };
    public static final ElevatorHeights kElevatorHeights = new ElevatorHeights(0, 1.6, 3.4, 6.73);

    public record MotionMagicConfigs(double vel, double accel, double jerk) {}

    public record ElevatorHeights(double stow, double l2, double l3, double l4) {}

    public record ElevatorConfigs(int ID) {}
}
