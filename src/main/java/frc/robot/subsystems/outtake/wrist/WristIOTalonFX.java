package frc.robot.subsystems.outtake.wrist;

import static frc.robot.subsystems.outtake.wrist.WristConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class WristIOTalonFX implements WristIO {

    private TalonFX motor;
    private TalonFXConfiguration config;
    private final NeutralOut neutralOut = new NeutralOut(); // neutral control (equivalent to stopping the motor)
    private MotionMagicVoltage reqMotionMagic = new MotionMagicVoltage(0);


    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorAppliedVoltage;
    private final StatusSignal<Current> motorSupplyCurrent;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorTempCelsius;
    private final StatusSignal<Double> referenceVelocity;
    private final StatusSignal<Double> referencePose;

    public WristIOTalonFX() {
        motor = new TalonFX(Ports.kWristID, GlobalConstants.kCANivoreName);
        motorPosition = motor.getPosition();
        motorAppliedVoltage = motor.getMotorVoltage();
        motorVelocity = motor.getVelocity();
        motorTempCelsius = motor.getDeviceTemp();
        motorTorqueCurrent = motor.getTorqueCurrent();
        motorSupplyCurrent = motor.getSupplyCurrent();

        referenceVelocity = motor.getClosedLoopReferenceSlope();
        referencePose = motor.getClosedLoopReference();

        config.MotionMagic.MotionMagicCruiseVelocity = Units.degreesToRotations(kMotionConfigs.kCruiseVel());
        config.MotionMagic.MotionMagicAcceleration = Units.degreesToRotations(kMotionConfigs.kAcceleration());
        config.MotionMagic.MotionMagicJerk = Units.degreesToRotations(kMotionConfigs.kJerk());

        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.kLooperHZ, // 50 hz
                motorPosition,
                motorVelocity,
                motorAppliedVoltage,
                motorSupplyCurrent,
                motorTorqueCurrent,
                motorTempCelsius,
                referenceVelocity,
                referencePose);

        // Basic motor config
        config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = 1;

        config.Slot0.kP = WristConstants.kGains.kP();
        config.Slot0.kI = WristConstants.kGains.kI();
        config.Slot0.kD = WristConstants.kGains.kD();

        // Apply the configuration
        motor.getConfigurator().apply(config);
    }

    // In WristIO, theres a wristIOInputs class that has all the inputs we want to
    // log. Update these values using the
    // method below
    public void updateInputs(WristIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                        motorPosition,
                        motorVelocity,
                        motorAppliedVoltage,
                        motorSupplyCurrent,
                        motorTorqueCurrent,
                        motorTempCelsius)
                .isOK();
        inputs.position = motorPosition.getValueAsDouble();
        inputs.velocity = motor.getVelocity().getValueAsDouble();
        inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
        inputs.supplyCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
        inputs.tempCelcius = motor.getDeviceTemp().getValueAsDouble();
        inputs.motorConnected = motor.isConnected();
    }

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
        motor.setPosition(angle);
    }

    // call .setControl on the motor controller with the appropriate control mode
    // and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode
    @Override
    public void setBrakeMode(boolean enabled) {
        motor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setPID(double P, double I, double D) {
        config.Slot0.kP = P;
        config.Slot0.kI = I;
        config.Slot0.kD = D;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void setFF(double kA, double kG, double kS, double kV) {
        
        config.Slot0.kG = kG;
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void stop() {
        motor.setControl(neutralOut);
    }
}
