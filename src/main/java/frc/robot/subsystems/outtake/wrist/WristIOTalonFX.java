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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class WristIOTalonFX implements WristIO {

    private TalonFX motor;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private MotionMagicVoltage reqMotionMagic = new MotionMagicVoltage(0);
    private final NeutralOut neutralOut = new NeutralOut();
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorAppliedVoltage;
    private final StatusSignal<Current> motorSupplyCurrent;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorTempCelsius;
    private final StatusSignal<Double> referenceVelocity;
    private final StatusSignal<Double> referencePose;

    public WristIOTalonFX() {
        motor = new TalonFX(Ports.kWristID);

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

        config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = kGearRatio;

        config.Slot0.kP = WristConstants.kGains.kP();
        config.Slot0.kI = WristConstants.kGains.kI();
        config.Slot0.kD = WristConstants.kGains.kD();

        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.kLooperHZ,
                motorPosition,
                motorVelocity,
                motorAppliedVoltage,
                motorSupplyCurrent,
                motorTorqueCurrent,
                motorTempCelsius,
                referencePose,
                referenceVelocity);

        motor.getConfigurator().apply(config);

        resetPosition(kTopDegree);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                        motorPosition,
                        motorVelocity,
                        motorAppliedVoltage,
                        motorSupplyCurrent,
                        motorTorqueCurrent,
                        motorTempCelsius,
                        referencePose,
                        referenceVelocity)
                .isOK();
        inputs.position = Units.rotationsToDegrees(motorPosition.getValueAsDouble());
        inputs.velocity = Units.rotationsToDegrees(motorVelocity.getValueAsDouble());
        inputs.appliedVolts = motorAppliedVoltage.getValueAsDouble();
        inputs.supplyCurrentAmps = motorSupplyCurrent.getValueAsDouble();
        inputs.tempCelcius = motorTempCelsius.getValueAsDouble();
    }

    @Override
    public void runPosition(double degrees) {
        motor.setControl(reqMotionMagic.withPosition(Units.degreesToRotations(degrees)));
    }

    @Override
    public void runVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void resetPosition(double angle) {
        motor.setPosition(Units.degreesToRotations(angle));
    }

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
