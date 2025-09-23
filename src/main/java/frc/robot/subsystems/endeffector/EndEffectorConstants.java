package frc.robot.subsystems.endeffector;

import frc.robot.constants.Ports;

public class EndEffectorConstants {

    public static final RollerConfig kRollerConfig =
            new RollerConfig(Ports.kEndEffectorLeaderID, Ports.kEndEffectorFollowerID);

    public static final double kRollerGearRatio = 0.0; // ! Change

    public static final Gains kGains = new Gains(0, 0, 0, 0, 0, 0); // ! Change

    public static final double kRollerFrequency = 50.0;
    public static final double kVelocityTolerance = 1.0;
    public static final double kSupplyCurrentLimit = 40.0;

    public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}

    public record RollerConfig(int leaderID, int followerID) {}
}
