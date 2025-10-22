package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.kGains;
import static frc.robot.subsystems.elevator.ElevatorConstants.kGearRatio;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;
import java.util.List;

public class ElevatorIOTalonFX implements ElevatorIO {

    private TalonFX leaderMotor, followerMotor;
    private TalonFXConfiguration config = new TalonFXConfiguration();
    private Follower follow;
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withUpdateFreqHz(GlobalConstants.kLooperDT);
    private NeutralOut neutralOut = new NeutralOut();
    private VoltageOut reqVoltageOut = new VoltageOut(0);

    private final List<StatusSignal<Angle>> motorPosition;
    private final List<StatusSignal<AngularVelocity>> motorVelocity;
    private final List<StatusSignal<Double>> motorReferencePosition;
    private final List<StatusSignal<Double>> motorReferenceVelocity;
    private final List<StatusSignal<Double>> motorReferenceError;
    private final List<StatusSignal<Voltage>> motorAppliedVoltage;
    private final List<StatusSignal<Current>> motorSupplyCurrent;
    private final List<StatusSignal<Current>> motorTorqueCurrent;
    private final List<StatusSignal<Temperature>> motorTempCelsius;

    public ElevatorIOTalonFX() {
        leaderMotor = new TalonFX(Ports.kEleLeaderID, GlobalConstants.kCANivoreName);
        followerMotor = new TalonFX(Ports.kEleFollowerID, GlobalConstants.kCANivoreName);

        follow = new Follower(leaderMotor.getDeviceID(), true);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.Slot0.kP = kGains.kP();
        config.Slot0.kI = kGains.kI();
        config.Slot0.kD = kGains.kD();
        config.Slot0.kS = kGains.kS();
        config.Slot0.kV = kGains.kV();
        config.Slot0.kG = kGains.kG();
        config.Slot0.kA = kGains.kA();
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        config.Feedback.SensorToMechanismRatio = kGearRatio;

        leaderMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);

        followerMotor.setControl(follow);
        leaderMotor.optimizeBusUtilization();

        motorPosition = List.of(leaderMotor.getPosition(), followerMotor.getPosition());
        motorVelocity = List.of(leaderMotor.getVelocity(), followerMotor.getVelocity());
        motorReferencePosition = List.of(leaderMotor.getClosedLoopReference(), followerMotor.getClosedLoopReference());
        motorReferenceVelocity =
                List.of(leaderMotor.getClosedLoopReferenceSlope(), followerMotor.getClosedLoopReferenceSlope());
        motorReferenceError = List.of(leaderMotor.getClosedLoopError(), followerMotor.getClosedLoopError());
        motorAppliedVoltage = List.of(leaderMotor.getMotorVoltage(), followerMotor.getMotorVoltage());
        motorSupplyCurrent = List.of(leaderMotor.getSupplyCurrent(), followerMotor.getSupplyCurrent());
        motorTorqueCurrent = List.of(leaderMotor.getTorqueCurrent(), followerMotor.getTorqueCurrent());
        motorTempCelsius = List.of(leaderMotor.getDeviceTemp(), followerMotor.getDeviceTemp());

        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.kLooperHZ,
                motorPosition.get(0),
                motorPosition.get(1),
                motorVelocity.get(0),
                motorVelocity.get(1),
                motorReferencePosition.get(0),
                motorReferencePosition.get(1),
                motorReferenceVelocity.get(0),
                motorReferenceVelocity.get(1),
                motorReferenceError.get(0),
                motorReferenceError.get(1),
                motorAppliedVoltage.get(0),
                motorAppliedVoltage.get(1),
                motorSupplyCurrent.get(0),
                motorSupplyCurrent.get(1),
                motorTorqueCurrent.get(0),
                motorTorqueCurrent.get(1),
                motorTempCelsius.get(0),
                motorTempCelsius.get(1),
                leaderMotor.getDutyCycle());
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leaderMotorConnected = BaseStatusSignal.refreshAll(
                        motorPosition.get(0),
                        motorVelocity.get(0),
                        motorReferencePosition.get(0),
                        motorReferenceVelocity.get(0),
                        motorReferenceError.get(0),
                        motorAppliedVoltage.get(0),
                        motorSupplyCurrent.get(0),
                        motorTorqueCurrent.get(0),
                        motorTempCelsius.get(0))
                .isOK();

        inputs.followerMotorConnected = BaseStatusSignal.refreshAll(
                        motorPosition.get(1),
                        motorVelocity.get(1),
                        motorReferencePosition.get(1),
                        motorReferenceVelocity.get(1),
                        motorReferenceError.get(1),
                        motorAppliedVoltage.get(1),
                        motorSupplyCurrent.get(1),
                        motorTorqueCurrent.get(1),
                        motorTempCelsius.get(1))
                .isOK();

        inputs.position = motorPosition.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.velocityRotationsPerSecond = motorVelocity.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.referencePosition = motorReferencePosition.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.referenceVelocity = motorReferenceVelocity.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.referenceError = motorReferenceError.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/MotionMagicDutyCycle.html
    @Override
    public void runPosition(double eleHeight) {
        leaderMotor.setControl(motionMagicVoltage.withPosition(eleHeight));
    }

    @Override
    public void resetPosition(double pos) {
        leaderMotor.setPosition(0);
    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/VoltageOut.html
    @Override
    public void runVolts(double volts) {
        leaderMotor.setControl(reqVoltageOut.withOutput(volts));
    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode
    @Override
    public void stop() {
        leaderMotor.setControl(neutralOut);
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        leaderMotor.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kI = kI;
        config.Slot0.kD = kD;
        leaderMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);
    }

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {
        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kG = kG;
        config.Slot0.kA = kA;
        leaderMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);
    }
}
