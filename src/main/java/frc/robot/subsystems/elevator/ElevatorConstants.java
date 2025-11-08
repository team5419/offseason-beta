package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import frc.robot.constants.GlobalConstants;
import frc.robot.lib.Gains;

public class ElevatorConstants {

    public static final double kInterferenceHeight = 0.05; // TODO: tune

    public static final Gains kGains =
            switch (GlobalConstants.getRobotType()) {
                case SIMBOT -> new Gains(0.0, 0.0, 0.0, 0.0, 0.95, 0.0, 0.56666666);
                case BETA -> new Gains(2.0, 0.0, 0.0, 0.0, 0.85, 0.0, 0.3);
                default -> new Gains(50.0, 0.0, 0.1, 0.2, 0.6, 0.0, 0.7);
            };

    public static final double kGearRatio =
            switch (GlobalConstants.getRobotType()) {
                case BETA -> 5.0;
                default -> 5.0;
            };

    public static final double sprocketPitchRadius = Units.inchesToMeters(2.607);

    public static final MotionMagicConfigs kMotionMagicConfigs =
            switch (GlobalConstants.getRobotType()) {
                    // TODO: tune
                default -> new MotionMagicConfigs(55, 30, 0);
            };

    public static final double kSprocketPitchRadius = Units.inchesToMeters(2.708);

    public static final ElevatorHeights kElevatorHeights = new ElevatorHeights(0, 0.2, 2, 3.4, 6.73);

    public static final double kSupplyCurrentLimit = 60.0;

    public record MotionMagicConfigs(double vel, double accel, double jerk) {}

    public record ElevatorHeights(double stow, double l1, double l2, double l3, double l4) {}

    public record ElevatorConfigs(int ID) {}
}
