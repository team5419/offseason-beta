package frc.robot.subsystems.outtake.wrist;

import frc.robot.lib.Gains;

public class WristConstants {

    public static final double kAngleTolerance = 0;

    public static final double kGearRatio = 14;

    public static final double kSupplyCurrentLimit = 60;

    public static final Gains kGains = new Gains(175, 0, 0, 0.15, 2.5, 0, 0.08);

    public static final MotionConfigs kMotionConfigs = new MotionConfigs(8000, 7200, 0);

    public static final WristAngles kWristAngles = new WristAngles(0, 0, 0, 0); // TODO Tune

    public record MotionConfigs(double kAcceleration, double kCruiseVel, double kJerk) {}

    public record WristAngles(double stow, double l1, double lowGoal, double l4) {}
}
