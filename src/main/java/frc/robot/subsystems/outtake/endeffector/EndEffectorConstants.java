package frc.robot.subsystems.outtake.endeffector;

import frc.robot.constants.Ports;
import frc.robot.lib.Gains;

public class EndEffectorConstants {

    public static final RollerConfig kRollerConfig =
            new RollerConfig(Ports.kEndEffectorLeaderID, Ports.kEndEffectorFollowerID);

    public static final double kRollerGearRatio = 0.0; // TODO: Change

    public static final Gains kGains = new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // TODO: Change

    public static final double kRollerFrequency = 50.0;
    public static final double kVelocityTolerance = 1.0;
    public static final double kSupplyCurrentLimit = 40.0;

    public record RollerConfig(int leaderID, int followerID) {}
}
