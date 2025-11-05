package frc.robot.subsystems.outtake.endeffector;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
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

    private final List<StatusSignal<Angle>> motorPosition;
    private final List<StatusSignal<AngularVelocity>> motorVelocity;
    private final List<StatusSignal<Voltage>> motorAppliedVoltage;
    private final List<StatusSignal<Current>> motorSupplyCurrent;
    private final List<StatusSignal<Current>> motorTorqueCurrent;
    private final List<StatusSignal<Temperature>> motorTempCelsius;

    private final boolean oppositeDirection = true;

    private TalonFX leaderMotor;
    private TalonFX followerMotor;
    private TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private Follower follow = new Follower(Ports.kEndEffectorLeaderID, oppositeDirection);

    private NeutralOut neutralOut = new NeutralOut();

    private VelocityVoltage reqVelocity = new VelocityVoltage(0.0);

    public EndEffectorIOTalonFX() {
        leaderMotor = new TalonFX(Ports.kEndEffectorLeaderID, GlobalConstants.kCANivoreName);
        followerMotor = new TalonFX(Ports.kEndEffectorFollowerID, GlobalConstants.kCANivoreName);

        talonConfig.Slot0.kP = EndEffectorConstants.kGains.kP();
        talonConfig.Slot0.kI = EndEffectorConstants.kGains.kI();
        talonConfig.Slot0.kD = EndEffectorConstants.kGains.kD();
        talonConfig.Slot0.kA = EndEffectorConstants.kGains.kA();
        talonConfig.Slot0.kG = EndEffectorConstants.kGains.kG();
        talonConfig.Slot0.kS = EndEffectorConstants.kGains.kS();
        talonConfig.Slot0.kV = EndEffectorConstants.kGains.kV();

        motorPosition = List.of(leaderMotor.getPosition(), followerMotor.getPosition());
        motorVelocity = List.of(leaderMotor.getVelocity(), followerMotor.getVelocity());
        motorAppliedVoltage = List.of(leaderMotor.getMotorVoltage(), followerMotor.getMotorVoltage());
        motorSupplyCurrent = List.of(leaderMotor.getSupplyCurrent(), followerMotor.getSupplyCurrent());
        motorTorqueCurrent = List.of(leaderMotor.getTorqueCurrent(), followerMotor.getTorqueCurrent());
        motorTempCelsius = List.of(leaderMotor.getDeviceTemp(), followerMotor.getDeviceTemp());

        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.kLooperHZ, // 50 hz
                motorPosition.get(0),
                motorPosition.get(1),
                motorVelocity.get(0),
                motorVelocity.get(1),
                motorAppliedVoltage.get(0),
                motorAppliedVoltage.get(1),
                motorSupplyCurrent.get(0),
                motorSupplyCurrent.get(1),
                motorTorqueCurrent.get(0),
                motorTorqueCurrent.get(1),
                motorTempCelsius.get(0),
                motorTempCelsius.get(1),
                leaderMotor.getDutyCycle());

        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.getConfigurator().apply(talonConfig);
        followerMotor.setControl(follow);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {

        inputs.motorConnected = BaseStatusSignal.refreshAll(
                        motorPosition.get(0),
                        motorPosition.get(1),
                        motorVelocity.get(0),
                        motorVelocity.get(1),
                        motorAppliedVoltage.get(0),
                        motorAppliedVoltage.get(1),
                        motorSupplyCurrent.get(0),
                        motorSupplyCurrent.get(1),
                        motorTempCelsius.get(0),
                        motorTempCelsius.get(1))
                .isOK();

        inputs.appliedVolts = motorAppliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
        inputs.supplyCurrentAmps = motorSupplyCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
        inputs.tempCelcius = motorTempCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
        inputs.velocityRPS = motorVelocity.stream().mapToDouble(StatusSignal::getValueAsDouble)
                .toArray();
    }

    @Override
    public void runVolts(double motorVolts) {
        leaderMotor.setVoltage(motorVolts);
    }

    @Override
    public void stop() {
        leaderMotor.setControl(neutralOut);
    }

    @Override
    public void runVelocity(double motorRPS) {
        leaderMotor.setControl(reqVelocity.withVelocity(motorRPS));
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        talonConfig.Slot0.kP = kP;
        talonConfig.Slot0.kI = kI;
        talonConfig.Slot0.kD = kD;
        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.getConfigurator().apply(talonConfig);
    }

    @Override
    public void setFF(double kA, double kG, double kS, double kV) {
        talonConfig.Slot0.kA = kA;
        talonConfig.Slot0.kG = kG;
        talonConfig.Slot0.kS = kS;
        talonConfig.Slot0.kV = kV;
        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.getConfigurator().apply(talonConfig);
    }
}
