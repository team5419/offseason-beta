package frc.robot.subsystems.intake.pivot;

import static frc.robot.subsystems.intake.pivot.IntakePivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
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
    private TalonFXConfiguration config;

    private MotionMagicVoltage reqMotionMagic = new MotionMagicVoltage(0);

    public IntakePivotIOTalonFX() {
        pivotMotor = new TalonFX(Ports.kIntakePivotID, GlobalConstants.kCANivoreName);
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

        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.kLooperHZ,
                motorPosition,
                motorVelocity,
                motorAppliedVoltage,
                motorSupplyCurrent,
                motorTorqueCurrent,
                motorTempCelsius);

        config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = kGearRatio;

        config.Slot0.kP = kGains.kP();
        config.Slot0.kI = kGains.kI();
        config.Slot0.kD = kGains.kD();
        config.Slot0.kS = kGains.kS();
        config.Slot0.kA = kGains.kA();
        config.Slot0.kG = kGains.kG();
        config.Slot0.kV = kGains.kV();
        pivotMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {}

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/MotionMagicDutyCycle.html
    @Override
    public void runPosition(double goal) {
        pivotMotor.setControl(reqMotionMagic.withPosition(goal));
    }

    @Override
    public void resetPosition(double pos) {}

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/VoltageOut.html
    @Override
    public void runVolts(double volts) {}

    @Override
    public void stop() {}

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
