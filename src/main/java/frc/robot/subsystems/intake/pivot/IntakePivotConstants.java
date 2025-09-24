package frc.robot.subsystems.intake.pivot;

import frc.robot.lib.Gains;

public class IntakePivotConstants {
    public static final double kAngleTolerance = 0;

    public static final double kGearRatio = 0;

    public static final Gains kGains = new Gains(0, 0, 0, 0, 0, 0, 0);

    public static final MotionConfigs kMotionConfigs = new MotionConfigs(0, 0, 0);

    public record MotionConfigs(double kAcceleration, double kCruiseVel, double kJerk) {}
}
