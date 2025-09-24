package frc.robot.subsystems.outtake.endeffector;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class EndEffectorIOTalonFX implements EndEffectorIO {

    private TalonFX leaderMotor, followerMotor;

    public EndEffectorIOTalonFX() {
        leaderMotor = new TalonFX(Ports.kEndEffectorLeaderID, GlobalConstants.kCANivoreName);
        followerMotor = new TalonFX(Ports.kEndEffectorFollowerID, GlobalConstants.kCANivoreName);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {}

    @Override
    public void runVolts(double motorVolts) {}

    @Override
    public void stop() {}

    @Override
    public void runVelocity(double motorRPS, double ff) {}

    @Override
    public void setPID(double kP, double kI, double kD) {}
}
