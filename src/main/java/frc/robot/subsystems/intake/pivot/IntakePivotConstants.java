package frc.robot.subsystems.intake.pivot;

import frc.robot.constants.GlobalConstants;
import frc.robot.lib.Gains;

public class IntakePivotConstants {
    public static final double kAngleTolerance = 0;

    public static final double kGearRatio = 0;

    public static final Gains kGains =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> new Gains(175, 0, 0, 0.15, 2.5, 0, 0.08);
                default -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0, 0);
            };

    public static final double kSupplyCurrentLimit = 60;

    public static final MotionConfigs kMotionConfigs = new MotionConfigs(0, 0, 0);

    public record MotionConfigs(double kAcceleration, double kCruiseVel, double kJerk) {}
}
