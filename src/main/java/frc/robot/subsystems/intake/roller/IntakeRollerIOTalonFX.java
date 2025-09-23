package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {

    private TalonFX leaderMotor, followerMotor;

    public IntakeRollerIOTalonFX() {
        leaderMotor = new TalonFX(Ports.kIntakeRollerLeaderID, GlobalConstants.kCANivoreName);
        followerMotor = new TalonFX(Ports.kIntakeRollerFollowerID, GlobalConstants.kCANivoreName);
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {}

    @Override
    public void runVolts(double motorVolts) {}

    @Override
    public void stop() {}

    @Override
    public void runVelocity(double motorRPS, double ff) {}

    @Override
    public void setPID(double kP, double kI, double kD) {}
}
