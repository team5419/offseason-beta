package frc.robot.subsystems.intake.pivot;

import static frc.robot.subsystems.intake.pivot.IntakePivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class IntakePivotIOTalonFX implements IntakePivotIO {
    private TalonFX motor;
    private TalonFXConfiguration config;
    private final VoltageOut reqVoltage = new VoltageOut(0.0).withEnableFOC(true);
    private final NeutralOut reqNeutral = new NeutralOut();
    private final VelocityVoltage reqVelocity = new VelocityVoltage(0);
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorSupplyCurrent;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorTempCelsius;
    private final StatusSignal<Voltage> motorAppliedVoltage;
    private final StatusSignal<Current> motorSupplyCurrentAmps;
    private final StatusSignal<Double> motorReference;
    private final StatusSignal<Double> motorReferenceVelocity;

    public IntakePivotIOTalonFX() {
        motor = new TalonFX(Ports.kIntakePivotID, GlobalConstants.kCANivoreName);
        motorPosition = motor.getPosition();
        motorAppliedVoltage = motor.getMotorVoltage();
        motorVelocity = motor.getVelocity();
        motorTempCelsius = motor.getDeviceTemp();
        motorSupplyCurrentAmps = motor.getSupplyCurrent();
        motorReference = motor.getClosedLoopReference();
        motorReferenceVelocity = motor.getClosedLoopReferenceSlope();
        motorSupplyCurrent = motor.getSupplyCurrent();
        motorTorqueCurrent = motor.getTorqueCurrent();
        motorVoltage = motor.getMotorVoltage();

    config = new TalonFXConfiguration();

        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.kLooperHZ, // 50 hz
                motorPosition,
                motorVelocity,
                motorAppliedVoltage,
                motorSupplyCurrent,
                motorTorqueCurrent,
                motorTempCelsius);

        // config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit,;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = 1;

        config.Slot0.kP = kGains.kP();
        config.Slot0.kI = kGains.kI();
        config.Slot0.kD = kGains.kD();
        config.Slot0.kS = kGains.kS();
        config.Slot0.kA = kGains.kA();
        config.Slot0.kG = kGains.kG();
        config.Slot0.kV = kGains.kV();
        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                        motorPosition,
                        motorVelocity,
                        motorAppliedVoltage,
                        motorSupplyCurrent,
                        motorTorqueCurrent,
                        motorTempCelsius)
                .isOK();
        inputs.position = motor.getPosition().getValueAsDouble();
    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/MotionMagicDutyCycle.html
    @Override
    public void runPosition(double goal) {
        motor.setControl(new MotionMagicVoltage(null).withEnableFOC(true));
    }
   
    
    @Override
    public void resetPosition(double pos) {
        motor.setPosition(pos);
    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/VoltageOut.html
    @Override
    public void runVolts(double volts) {
        motor.setControl(reqVoltage.withOutput(volts));
    }

    @Override
    public void stop() {
        motor.setControl(reqNeutral);
    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode
    @Override
    public void setBrakeMode(boolean enabled) {
        var MotorOutputConfigs = new TalonFXConfiguration().MotorOutput;
        MotorOutputConfigs.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    }
    @Override
    public void setPID(double kP, double kI, double kD) {
        var slot0 = new TalonFXConfiguration().Slot0;
        slot0.kP = kP;
        slot0.kI = kI;
        slot0.kD = kD;
        motor.getConfigurator().apply(new TalonFXConfiguration().withSlot0(slot0));

    }

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {     var slot0 = new TalonFXConfiguration().Slot0;
        slot0.kS = kS;
        slot0.kG = kG;
        slot0.kV = kV;
        slot0.kA = kA;
        motor.getConfigurator().apply(new TalonFXConfiguration().withSlot0(slot0));}
}
