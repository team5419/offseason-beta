package frc.robot.subsystems.outtakepivot;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class OuttakePivotIOTalonFX implements OuttakePivotIO {

    private TalonFX motor;

    public OuttakePivotIOTalonFX() {
        motor = new TalonFX(Ports.kOuttakePivotID, GlobalConstants.kCANivoreName);
    }

    public void updateInputs(OuttakePivotIOInputs inputs) {}

    public void runPosition(double goal) {}

    public void runVolts(double volts) {}

    public void resetPosition(double angle) {}

    public void setBrakeMode(boolean enabled) {}

    public void setPID(double P, double I, double D) {}

    public void setFF(double kA, double kG, double kS, double kV) {}

    public void stop() {}
}
