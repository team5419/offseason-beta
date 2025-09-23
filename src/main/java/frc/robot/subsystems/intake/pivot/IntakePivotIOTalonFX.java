package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class IntakePivotIOTalonFX implements IntakePivotIO {

    private TalonFX motor;

    public IntakePivotIOTalonFX() {
        motor = new TalonFX(Ports.kIntakePivotID, GlobalConstants.kCANivoreName);
    }
}
