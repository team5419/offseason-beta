package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class IntakePivotIOTalonFX implements IntakePivotIO {

    private TalonFX motor;

    public IntakePivotIOTalonFX() {
        motor = new TalonFX(Ports.kIntakePivotID, GlobalConstants.kCANivoreName);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {}

    @Override
    public void resetPosition(double pos) {}

    @Override
    public void runVolts(double volts) {}

    @Override
    public void stop() {}

    @Override
    public void setBrakeMode(boolean enabled) {}

    @Override
    public void setPID(double kP, double kI, double kD) {}

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {}
}
