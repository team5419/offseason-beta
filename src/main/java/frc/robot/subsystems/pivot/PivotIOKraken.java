package frc.robot.subsystems.pivot;

import frc.robot.subsystems.pivot.PivotIO.PivotIOInputs;

public class PivotIOKraken {
    public PivotIOKraken() {}

    public void updateInputs(PivotIOInputs inputs) {}

    public void runPosition(double goal) {}

    public void runVolts(double volts) {}

    public void resetPosition(double angle) {}

    public void setBrakeMode(boolean enabled) {}

    public void setPID(double P, double I, double D) {}

    public void setFF(double kA, double kG, double kS, double kV) {}

    public void stop() {}
}
