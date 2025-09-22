package frc.robot.subsystems.endeffector.pivot;

public class PivotConstants {

    public static final double kAngleTolerance = 0;

    public static final double kGearRatio = 0;

    public static final Gains kGains = new Gains(0, 0, 0, 0, 0, 0, 0);

    public static final MotionConfigs kMotionConfigs = new MotionConfigs(0, 0, 0);

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

    public record MotionConfigs(double kAcceleration, double kCruiseVel, double kJerk) {}
}
