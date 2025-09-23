package frc.robot.subsystems.outtake.wrist;

public class WristIOTalonFX implements WristIO {
    public WristIOTalonFX() {}

    @Override
    public void updateInputs(WristIOInputs inputs) {}

    @Override
    public void runPosition(double goal) {}

    @Override
    public void runVolts(double volts) {}

    @Override
    public void resetPosition(double angle) {}

    @Override
    public void setBrakeMode(boolean enabled) {}

    @Override
    public void setPID(double P, double I, double D) {}

    @Override
    public void setFF(double kA, double kG, double kS, double kV) {}

    @Override
    public void stop() {}
}
