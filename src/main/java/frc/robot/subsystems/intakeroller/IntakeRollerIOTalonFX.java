package frc.robot.subsystems.intakeroller;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {

    private TalonFX leaderMotor, followerMotor;

    public IntakeRollerIOTalonFX() {
        leaderMotor = new TalonFX(Ports.kIntakeRollerLeaderID, GlobalConstants.kCANivoreName);
        followerMotor = new TalonFX(Ports.kIntakeRollerFollowerID, GlobalConstants.kCANivoreName);
    }

    public void updateInputs(IntakeRollerIOInputs inputs) {}

    public void runVolts(double motorVolts) {}

    public void stop() {}

    public void runVelocity() {}

    public void setPID(double kP, double kI, double kD) {}
}
