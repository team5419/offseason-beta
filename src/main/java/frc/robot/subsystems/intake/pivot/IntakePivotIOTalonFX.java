package frc.robot.subsystems.intake.pivot;

import static frc.robot.subsystems.intake.pivot.IntakePivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class IntakePivotIOTalonFX implements IntakePivotIO {

    private TalonFX pivotMotor;
    private StatusSignal<Angle> motorPosition;
    private StatusSignal<Voltage> motorAppliedVoltage;
    private StatusSignal<AngularVelocity> motorVelocity;
    private StatusSignal<Temperature> motorTempCelsius;
    private StatusSignal<Current> motorSupplyCurrentAmps;
    private StatusSignal<Double> motorReference;
    private StatusSignal<Double> motorReferenceVelocity;
    private StatusSignal<Current> motorSupplyCurrent;
    private StatusSignal<Current> motorTorqueCurrent;
    private StatusSignal<Voltage> motorVoltage;
    private TalonFXConfiguration config = new TalonFXConfiguration();

    private MotionMagicVoltage reqMotionMagic = new MotionMagicVoltage(0);

    public IntakePivotIOTalonFX() {
        pivotMotor = new TalonFX(Ports.kIntakePivotID);

        motorPosition = pivotMotor.getPosition();
        motorAppliedVoltage = pivotMotor.getMotorVoltage();
        motorVelocity = pivotMotor.getVelocity();
        motorTempCelsius = pivotMotor.getDeviceTemp();
        motorSupplyCurrentAmps = pivotMotor.getSupplyCurrent();
        motorReference = pivotMotor.getClosedLoopReference();
        motorReferenceVelocity = pivotMotor.getClosedLoopReferenceSlope();
        motorSupplyCurrent = pivotMotor.getSupplyCurrent();
        motorTorqueCurrent = pivotMotor.getTorqueCurrent();
        motorVoltage = pivotMotor.getMotorVoltage();

        config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = kGearRatio;

        config.Slot0.kP = kGains.kP();
        config.Slot0.kI = kGains.kI();
        config.Slot0.kD = kGains.kD();
        config.Slot0.kS = kGains.kS();
        config.Slot0.kA = kGains.kA();
        config.Slot0.kG = kGains.kG();
        config.Slot0.kV = kGains.kV();

        config.MotionMagic.MotionMagicCruiseVelocity =
                edu.wpi.first.math.util.Units.degreesToRotations(kMotionConfigs.kCruiseVel());
        config.MotionMagic.MotionMagicAcceleration =
                edu.wpi.first.math.util.Units.degreesToRotations(kMotionConfigs.kAcceleration());
        config.MotionMagic.MotionMagicJerk = edu.wpi.first.math.util.Units.degreesToRotations(kMotionConfigs.kJerk());

        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.kLooperHZ,
                motorPosition,
                motorVelocity,
                motorAppliedVoltage,
                motorSupplyCurrent,
                motorTorqueCurrent,
                motorReference,
                motorReferenceVelocity,
                motorVoltage,
                motorTempCelsius);

        pivotMotor.getConfigurator().apply(config);

        resetPosition(67.5);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.refreshAll(
                        motorPosition,
                        motorVelocity,
                        motorAppliedVoltage,
                        motorTempCelsius,
                        motorSupplyCurrentAmps,
                        motorReference,
                        motorReferenceVelocity)
                .isOK();
        inputs.position = Units.rotationsToDegrees(motorPosition.getValueAsDouble());
        inputs.velocity = motorVelocity.getValueAsDouble();
        inputs.appliedVolts = motorAppliedVoltage.getValueAsDouble();
        inputs.tempCelcius = motorTempCelsius.getValueAsDouble(); // °C
        inputs.supplyCurrentAmps = motorSupplyCurrent.getValueAsDouble();
        inputs.referencePose = Units.rotationsToDegrees(motorReference.getValueAsDouble());
        inputs.referenceVelocity = Units.rotationsToDegrees(motorReferenceVelocity.getValueAsDouble());
    }

    @Override
    public void runPosition(double goal) {
        System.out.println("Intake Pivot Goal: " + goal);
        System.out.println("Intake Pivot p: " + config.Slot0.kP);

        // pivotMotor.setControl(reqMotionMagic
        //         .withPosition(edu.wpi.first.math.util.Units.degreesToRotations(goal))
        //         .withSlot(0));
        pivotMotor.setControl(reqMotionMagic.withPosition(goal / 360));
    }

    @Override
    public void resetPosition(double pos) {
        pivotMotor.setPosition(edu.wpi.first.math.util.Units.degreesToRotations(pos));
    }

    @Override
    public void runVolts(double volts) {
        pivotMotor.setControl(new DutyCycleOut(volts));
    }

    @Override
    public void stop() {
        pivotMotor.setControl(new NeutralOut());
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        pivotMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        pivotMotor.getConfigurator().apply(config);
    }

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {
        config.Slot0.kS = kS;
        config.Slot0.kG = kG;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;
        pivotMotor.getConfigurator().apply(config);
    }
}
