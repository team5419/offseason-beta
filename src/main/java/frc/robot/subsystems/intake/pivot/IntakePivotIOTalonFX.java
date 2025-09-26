package frc.robot.subsystems.intake.pivot;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class IntakePivotIOTalonFX implements IntakePivotIO {

    public IntakePivotIOTalonFX 

       config.Slot0.kP = kGains.kP();
       config.Slot0.kI = kGains.kI();
       config.Slot0.kD = kGains.kD();
       config.Slot0.kS = kGains.kS();
       config.Slot0.kA = kGains.kA();
       config.Slot0.kG = kGains.kG();
       config.Slot0.kV = kGains.kV();

    private TalonFX motor;

    public IntakePivotIOTalonFX() {
        motor = new TalonFX(Ports.kIntakePivotID, GlobalConstants.kCANivoreName);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.resfreshALL(
                            motorPosition,
                            motorVelocity,
                            motorAppliedVoltage,
                            motorSupplyCurrent
                            motorTorqueCurrent,
                            motorTempCelsius)
                        .isOK();
        inputs.positionRAD = talon.getPosition().getValueAsDoubled();
        
                            
        )    }
        
    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/MotionMagicDutyCycle.html
    @Override
    public void runPosition(double goal) {

    }

    @Override
    public void resetPosition(double pos) {

    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/VoltageOut.html
    @Override
    public void runVolts(double volts) {
        talon.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void stop() {
        talon.setControl(NeutralOut);
    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode
    @Override
    public void setBrakeMode(boolean enabled) {}

    @Override
    public void setPID(double kP, double kI, double kD) {}

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {}
}
