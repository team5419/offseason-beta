package frc.robot.subsystems.intake.roller;

import frc.robot.constants.Ports;
import frc.robot.lib.Gains;

public class IntakeRollerConstants {
    public static final RollerConfig kRollerConfig =
            new RollerConfig(Ports.kIntakeRollerLeaderID, Ports.kIntakeRollerFollowerID);

    public static final double kRollerGearRatio = 0.0; // ! Change

    public static final Gains kGains = new Gains(0, 0, 0, 0, 0, 0, 0);

    public static final double kRollerFrequency = 50.0;
    public static final double kVelocityTolerance = 1.0;
    public static final double kSupplyCurrentLimit = 40.0;

    public record RollerConfig(int leaderID, int followerID) {}
}
