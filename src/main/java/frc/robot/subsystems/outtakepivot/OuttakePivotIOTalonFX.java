package frc.robot.subsystems.outtakepivot;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class OuttakePivotIOTalonFX implements OuttakePivotIO {

    private TalonFX motor;

    public OuttakePivotIOTalonFX() {
        motor = new TalonFX(Ports.kOuttakePivotID, GlobalConstants.kCANivoreName);
    }

    @Override
    public void updateInputs(OuttakePivotIOInputs inputs) {}

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
