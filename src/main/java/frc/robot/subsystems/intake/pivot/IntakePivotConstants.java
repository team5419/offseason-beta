package frc.robot.subsystems.intake.pivot;

import frc.robot.constants.GlobalConstants;

public class IntakePivotConstants {

    public static final double kGearRatio =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 80;
                case SIMBOT -> 310.0 / 3;
                default -> 14;
            };

    public static final double kBottomDegree =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 0;
                default -> 0;
            };

    public static final double kTopDegree =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 150;
                default -> 150;
            };

    public static final double kTolorance =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 0.05;
                default -> 0.05;
            };

    public static final double kZeroPos =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 0;
                default -> 0;
            };

    public static final Gains kGains =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> new Gains(175, 0, 0, 0.15, 2.5, 0, 0.08);
                case SIMBOT -> new Gains(8, 0.0, 1.0, 8.4, 0.0, 0.0, 22.9);
                default -> new Gains(175, 0, 0, 0.15, 2.5, 0, 0.08);
            };

    public static final double kSupplyCurrentLimit =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 60;
                default -> 60;
            };

    public static final MotionConfigs kMotionConfigs =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> new MotionConfigs(8000, 7200, 0);
                default -> new MotionConfigs(8000, 7200, 0);
            };

    public record MotionConfigs(double kAcceleration, double kCruiseVel, double kJerk) {}

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}
}
