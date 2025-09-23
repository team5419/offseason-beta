package frc.robot.subsystems.wrist;

import frc.robot.constants.Ports;

public class WristConstants {

    public static final double kAngleTolerance = 0;

    public static final double kGearRatio = 0;

    public static final Gains kGains = new Gains(0, 0, 0, 0, 0, 0, 0);

    public static final MotionConfigs kMotionConfigs = new MotionConfigs(0, 0, 0);

    public static final WristConfig kWristConfig = new WristConfig(Ports.kWristID);

    public static final WristAngles kWristAngles = new WristAngles(0, 0, 0, 0); // TODO Tune

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

    public record MotionConfigs(double kAcceleration, double kCruiseVel, double kJerk) {}

    public record WristConfig(int id) {}

    public record WristAngles(double stow, double l1, double lowGoal, double l4) {}
}
