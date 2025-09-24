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

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/VoltageOut.html
    @Override
    public void runVolts(double motorVolts) {}

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/NeutralOut.html
    @Override
    public void stop() {}

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/MotionMagicVelocityVoltage.html
    @Override
    public void runVelocity(double motorRPS, double ff) {}

    @Override
    public void setPID(double kP, double kI, double kD) {}
}
