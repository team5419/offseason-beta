package frc.robot.subsystems.outtake.wrist;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class WristIOTalonFX implements WristIO {

    private TalonFX motor;
    private TalonFXConfiguration config;

    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorAppliedVoltage;
    private final StatusSignal<Current> motorSupplyCurrent;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorTempCelsius;
    public WristIOTalonFX() {
       motor = new TalonFX(Ports.kWristID, GlobalConstants.kCANivoreName);

        motorPosition = motor.getPosition();
        motorAppliedVoltage = motor.getMotorVoltage();
        motorVelocity = motor.getVelocity();
        motorTempCelsius = motor.getDeviceTemp();
        motorTorqueCurrent = motor.getTorqueCurrent();
        motorSupplyCurrent = motor.getSupplyCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.kLooperHZ, // 50 hz
                motorPosition,
                motorVelocity,
                motorAppliedVoltage,
                motorSupplyCurrent,
                motorTorqueCurrent,
                motorTempCelsius);

        config.Slot0.kP = WristConstants.kGains.kP();
        config.Slot0.kI = WristConstants.kGains.kI();
        config.Slot0.kD = WristConstants.kGains.kD();

    }

    // In WristIO, theres a wristIOInputs class that has all the inputs we want to
    // log. Update these values using the
    // method below
    public void updateInputs(WristIOInputs inputs) {
    }
    MotionMagicVoltage reqMotionMagic = new MotionMagicVoltage(0);
    // call .setControl on the motor controller with the appropriate control mode
    // and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/MotionMagicDutyCycle.html
    @Override
    public void runPosition(double goal) {
        motor.setControl(reqMotionMagic.withPosition(goal));
    }

    // call .setControl on the motor controller with the appropriate control mode
    // and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/VoltageOut.html
    @Override
    public void runVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void resetPosition(double angle) {
        motor.setPosition(0);
    }

    // call .setControl on the motor controller with the appropriate control mode
    // and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode
    @Override
    public void setBrakeMode(boolean enabled) {

    }

    @Override
    public void setPID(double P, double I, double D) {

    }

    @Override
    public void setFF(double kA, double kG, double kS, double kV) {
    }

    @Override
    public void stop() {

    }
}
