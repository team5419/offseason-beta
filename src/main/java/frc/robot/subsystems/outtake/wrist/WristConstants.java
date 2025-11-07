package frc.robot.subsystems.outtake.wrist;

import frc.robot.constants.GlobalConstants;
import frc.robot.lib.Gains;

public class WristConstants {

    public static final double kAngleTolerance =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 0;
                default -> 0;
            };

    public static final double kTopDegree =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 180;
                default -> 180;
            };

    public static final double kGearRatio =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 24;
                case SIMBOT -> 310 / 3;
                default -> 90;
            };

    public static final double kBottomDegree =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> -60;
                default -> -60;
            };

    public static final double kSupplyCurrentLimit = 60;

    public static final Gains kGains =
            switch (GlobalConstants.getRobotType()) {
                case SIMBOT -> new Gains(5, 0.0, 1.0, 8.4, 0.0, 0.0, 22.9);
                case BETA -> new Gains(0, 0, 0, 0, 0, 0, 0);
                default -> new Gains(175, 0, 0, 0.15, 2.5, 0, 0.08);
            };

    public static final MotionConfigs kMotionConfigs =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> new MotionConfigs(0, 0, 0);
                default -> new MotionConfigs(8000, 7200, 0);
            };

    public static final WristAngles kWristAngles =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> new WristAngles(0, 0, 0, 0);
                default -> new WristAngles(0, 0, 0, 0);
            };

    public record MotionConfigs(double kAcceleration, double kCruiseVel, double kJerk) {}

    public record WristAngles(double stow, double l1, double lowGoal, double l4) {}
}
