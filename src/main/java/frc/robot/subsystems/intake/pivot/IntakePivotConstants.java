package frc.robot.subsystems.intake.pivot;

import frc.robot.constants.GlobalConstants;
import frc.robot.lib.Gains;

public class IntakePivotConstants {
    public static final double kAngleTolerance = 0;

    public static final double kGearRatio = 0;

    public static final Gains kGains =
            switch (GlobalConstants.getRobotType()) {
                case SIMBOT -> new Gains(10, 0.0, 0.0, 8.4, 0.0, 0.0, 22.9);
                default -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0, 0);
            };

    public static final MotionConfigs kMotionConfigs = new MotionConfigs(0, 0, 0);

    public record MotionConfigs(double kAcceleration, double kCruiseVel, double kJerk) {}
}
