package frc.robot.subsystems.intake.pivot;

import frc.robot.constants.GlobalConstants;
import frc.robot.lib.Gains;

public class IntakePivotConstants {
    public static final double kAngleTolerance = 1;

    public static final double kGearRatio = 14;

    public static final double kZeroPos = 0;

    public static final Gains kGains =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> new Gains(175, 0, 0, 0.15, 2.5, 0, 0.08);
                default -> new Gains(175, 0, 0, 0.15, 2.5, 0, 0.08);
            };

    public static final double kSupplyCurrentLimit = 60;

    public static final MotionConfigs kMotionConfigs = new MotionConfigs(8000, 7200, 0);

    public record MotionConfigs(double kAcceleration, double kCruiseVel, double kJerk) {}
}
