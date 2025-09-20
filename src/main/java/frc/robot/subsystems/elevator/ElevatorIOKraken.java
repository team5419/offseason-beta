package frc.robot.subsystems.elevator;

import frc.robot.subsystems.elevator.ElevatorIO.EleavtorIOInputs;

public class ElevatorIOKraken {

    public void updateInputs(EleavtorIOInputs inputs) {}

    public void runPosition(double eleHeight, double feedforward) {}

    public void resetPosition(double pos) {}

    public void runVolts(double volts) {}

    public void runVelocity(double velocity) {}

    public void stop() {}

    public void setBrakeMode(boolean enabled) {}

    public void setPID(double kP, double kI, double kD) {}

    public void setFF(double kS, double kG, double kV, double kA) {}
}
