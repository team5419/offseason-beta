package frc.robot.subsystems.outtake.wrist;

import frc.robot.constants.GlobalConstants;
import frc.robot.lib.Gains;

public class WristConstants {

    public static final double kAngleTolerance = 0;

    public static final double kGearRatio = 0;

    public static final double kSupplyCurrentLimit = 60;

    public static final Gains kGains =
            switch (GlobalConstants.getRobotType()) {
                case SIMBOT -> new Gains(10, 0.0, 0.0, 8.4, 0.0, 0.0, 22.9);
                default -> new Gains(0.05, 0.0, 0.0, 0.01, 0.00103, 0.0, 0);
            };

    public static final MotionConfigs kMotionConfigs = new MotionConfigs(0, 0, 0);

    public static final WristAngles kWristAngles = new WristAngles(0, 0, 0, 0); // TODO Tune

    public record MotionConfigs(double kAcceleration, double kCruiseVel, double kJerk) {}

    public record WristAngles(double stow, double l1, double lowGoal, double l4) {}
}
