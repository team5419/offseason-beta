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

    @Override
    public void runPosition(double eleHeight, double feedforward) {}

    @Override
    public void resetPosition(double pos) {}

    @Override
    public void runVolts(double volts) {}

    public void runVelocity(double velocity) {}

    @Override
    public void stop() {}

    @Override
    public void setBrakeMode(boolean enabled) {}

    @Override
    public void setPID(double kP, double kI, double kD) {}

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {}
}
