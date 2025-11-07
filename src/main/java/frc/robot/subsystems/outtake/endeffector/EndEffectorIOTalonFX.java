package frc.robot.subsystems.outtake.endeffector;

import static frc.robot.subsystems.outtake.endeffector.EndEffectorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;
import java.util.List;

public class EndEffectorIOTalonFX implements EndEffectorIO {

    private TalonFX leaderMotor, followerMotor;
    private Follower follow = new Follower(Ports.kEndEffectorFollowerID, true);
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    private VoltageOut voltageOut = new VoltageOut(0);

    private NeutralOut neutralOut = new NeutralOut();

    private VelocityVoltage reqVel = new VelocityVoltage(0);

    private final List<StatusSignal<Angle>> motorPosition;
    private final List<StatusSignal<AngularVelocity>> motorVelocity;
    private final List<StatusSignal<Double>> motorReferencePosition;
    private final List<StatusSignal<Double>> motorReferenceVelocity;
    private final List<StatusSignal<Voltage>> motorAppliedVoltage;
    private final List<StatusSignal<Current>> motorSupplyCurrent;
    private final List<StatusSignal<Temperature>> motorTempCelsius;

    public EndEffectorIOTalonFX() {
        leaderMotor = new TalonFX(Ports.kEndEffectorLeaderID, GlobalConstants.kCANivoreName);
        followerMotor = new TalonFX(Ports.kEndEffectorFollowerID, GlobalConstants.kCANivoreName);

        motorPosition = List.of(leaderMotor.getPosition(), followerMotor.getPosition());
        motorVelocity = List.of(leaderMotor.getVelocity(), followerMotor.getVelocity());
        motorReferencePosition = List.of(leaderMotor.getClosedLoopReference(), followerMotor.getClosedLoopReference());
        motorReferenceVelocity =
                List.of(leaderMotor.getClosedLoopReferenceSlope(), followerMotor.getClosedLoopReferenceSlope());
        motorSupplyCurrent = List.of(leaderMotor.getSupplyCurrent(), followerMotor.getSupplyCurrent());
        motorTempCelsius = List.of(leaderMotor.getDeviceTemp(), followerMotor.getDeviceTemp());
        motorAppliedVoltage = List.of(leaderMotor.getMotorVoltage(), followerMotor.getMotorVoltage());

        talonConfig.Slot0.kP = kGains.kP();
        talonConfig.Slot0.kI = kGains.kI();
        talonConfig.Slot0.kD = kGains.kD();

        talonConfig.Slot0.kS = kGains.kS();
        talonConfig.Slot0.kV = kGains.kV();
        talonConfig.Slot0.kA = kGains.kA();
        talonConfig.Slot0.kG = kGains.kG();

        talonConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.getConfigurator().apply(talonConfig);
        followerMotor.setControl(follow);

        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.kLooperHZ,
                motorPosition.get(0),
                motorPosition.get(1),
                motorVelocity.get(0),
                motorVelocity.get(1),
                motorAppliedVoltage.get(0),
                motorAppliedVoltage.get(1),
                motorSupplyCurrent.get(0),
                motorSupplyCurrent.get(1),
                motorTempCelsius.get(0),
                motorTempCelsius.get(1),
                motorReferencePosition.get(0),
                motorReferencePosition.get(1),
                motorReferenceVelocity.get(0),
                motorReferenceVelocity.get(1),
                leaderMotor.getDutyCycle());
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
        inputs.leaderMotorConnected = BaseStatusSignal.refreshAll(
                        motorPosition.get(0),
                        motorVelocity.get(0),
                        motorAppliedVoltage.get(0),
                        motorSupplyCurrent.get(0),
                        motorTempCelsius.get(0))
                .isOK();

        inputs.followerMotorConnected = BaseStatusSignal.refreshAll(
                        motorPosition.get(1),
                        motorVelocity.get(1),
                        motorAppliedVoltage.get(1),
                        motorSupplyCurrent.get(1),
                        motorTempCelsius.get(1))
                .isOK();

        inputs.position = motorPosition.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.velocity = motorVelocity.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.appliedVolts = motorAppliedVoltage.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.supplyCurrentAmps = motorSupplyCurrent.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.tempCelcius = motorTempCelsius.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.referencePose = motorReferencePosition.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();

        inputs.referenceVelocity = motorReferenceVelocity.stream()
                .mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
    }

    @Override
    public void runVolts(double motorVolts) {
        leaderMotor.setControl(voltageOut.withOutput(motorVolts));
    }

    @Override
    public void stop() {
        leaderMotor.setControl(neutralOut);
    }

    @Override
    public void runVelocity(double motorRPS) {
        leaderMotor.setControl(reqVel.withVelocity(motorRPS));
    }

    @Override
    public void setFF(double kS, double kV, double kA, double kG) {
        talonConfig.Slot0.kA = kA;
        talonConfig.Slot0.kG = kG;
        talonConfig.Slot0.kS = kS;
        talonConfig.Slot0.kV = kV;
        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.getConfigurator().apply(talonConfig);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        talonConfig.Slot0.kP = kP;
        talonConfig.Slot0.kI = kI;
        talonConfig.Slot0.kD = kD;
        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.getConfigurator().apply(talonConfig);
    }
}
