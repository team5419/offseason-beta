package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class ElevatorIOTalonFX implements ElevatorIO {

    private TalonFX leaderMotor, followerMotor;

    public ElevatorIOTalonFX() {
        leaderMotor = new TalonFX(Ports.kEleLeaderID, GlobalConstants.kCANivoreName);
        followerMotor = new TalonFX(Ports.kEleFollowerID, GlobalConstants.kCANivoreName);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {}

     // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/MotionMagicDutyCycle.html
    @Override
    public void runPosition(double eleHeight, double feedforward) {}

    @Override
    public void resetPosition(double pos) {}

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/VoltageOut.html
    @Override
    public void runVolts(double volts) {}

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode
    @Override
    public void stop() {}

    @Override
    public void setBrakeMode(boolean enabled) {}

    @Override
    public void setPID(double kP, double kI, double kD) {}

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {}
}
