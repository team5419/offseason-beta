package frc.robot.subsystems.outtake.wrist;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class WristIOTalonFX implements WristIO {

    private TalonFX motor;

    public WristIOTalonFX() {
        motor = new TalonFX(Ports.kWristID, GlobalConstants.kCANivoreName);
    }

    // In WristIO, theres a wristIOInputs class that has all the inputs we want to log. Update these values using the
    // method below
    public void updateInputs(WristIOInputs inputs) {}

    // call .setControlMode on the motor controller with the appropriate control mode and value.
    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html
    @Override
    public void runPosition(double goal) {}

    // call .setControlMode on the motor controller with the appropriate control mode and value.
    // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/talonfx-control-intro.html
    @Override
    public void runVolts(double volts) {}

    @Override
    public void resetPosition(double angle) {}

    // call .setControlMode on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode
    @Override
    public void setBrakeMode(boolean enabled) {}

    @Override
    public void setPID(double P, double I, double D) {}

    @Override
    public void setFF(double kA, double kG, double kS, double kV) {}

    @Override
    public void stop() {}
}
