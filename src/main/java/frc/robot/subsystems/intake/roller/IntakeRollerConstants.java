package frc.robot.subsystems.intake.roller;

import frc.robot.lib.Gains;

public class IntakeRollerConstants {

    public static final double kRollerGearRatio = 1.0; // ! Change

    public static final Gains kGains = new Gains(0.5, 0.0, 0.0, 0.26, 0.125, 0.0, 0.0);

    public static final double kVelocityTolerance = 1.0;
    public static final double kSupplyCurrentLimit = 40.0;
}
