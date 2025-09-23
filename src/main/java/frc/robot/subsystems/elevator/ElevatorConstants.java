package frc.robot.subsystems.elevator;

import frc.robot.constants.GlobalConstants;
import frc.robot.lib.Gains;

public class ElevatorConstants {

    // TODO: tune
    public static final double kStowHeight = 0;
    public static final double kL2Height = 1.6;
    public static final double kL3Height = 3.4;
    public static final double kL4Height = 6.73;

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

    public record MotionMagicConfigs(double vel, double accel, double jerk) {}
}
