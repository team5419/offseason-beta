package frc.robot.subsystems.outtake.endeffector;

import static frc.robot.subsystems.outtake.endeffector.EndEffectorConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
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

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();

    private VoltageOut voltageOut = new VoltageOut(0);

    private NeutralOut neutralOut = new NeutralOut();

    private VelocityVoltage reqVelTorque = new VelocityVoltage(0);

    private final List<StatusSignal<Angle>> motorPosition;
    private final List<StatusSignal<AngularVelocity>> motorVelocity;
    private final List<StatusSignal<Double>> motorReferencePosition;
    private final List<StatusSignal<Voltage>> motorAppliedVoltage;
    private final List<StatusSignal<Current>> motorSupplyCurrent;
    private final List<StatusSignal<Current>> motorTorqueCurrent;
    private final List<StatusSignal<Temperature>> motorTempCelsius;

    public EndEffectorIOTalonFX() {
        leaderMotor = new TalonFX(Ports.kEndEffectorLeaderID, GlobalConstants.kCANivoreName);
        followerMotor = new TalonFX(Ports.kEndEffectorFollowerID, GlobalConstants.kCANivoreName);

        motorPosition = List.of(leaderMotor.getPosition(), followerMotor.getPosition());
        motorVelocity = List.of(leaderMotor.getVelocity(), followerMotor.getVelocity());
        motorReferencePosition = List.of(leaderMotor.getClosedLoopReference(), followerMotor.getClosedLoopReference());
        motorSupplyCurrent = List.of(leaderMotor.getSupplyCurrent(), followerMotor.getSupplyCurrent());
        motorTorqueCurrent = List.of(leaderMotor.getTorqueCurrent(), followerMotor.getTorqueCurrent());
        motorTempCelsius = List.of(leaderMotor.getDeviceTemp(), followerMotor.getDeviceTemp());
        motorAppliedVoltage = List.of(leaderMotor.getMotorVoltage(), followerMotor.getMotorVoltage());

        talonConfig.Slot0.kA = 0.0;
        talonConfig.Slot0.kG = 0.0;

        talonConfig.Slot0.kS = 0.0;
        talonConfig.Slot0.kV = 0.0;

        talonConfig.Slot0.kP = 0.11; // This gets us closer to the target velocity
        talonConfig.Slot0.kI = 0; // 99% of the time, this will be 0
        talonConfig.Slot0.kD = 0.02; // Smooths out the velocity grap

        talonConfig.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.getConfigurator().apply(talonConfig);
    }

    @Override
    public void updateInputs(EndEffectorIOInputs inputs) {
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
                leaderMotor.getDutyCycle());
    }
    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/VoltageOut.html
    @Override
    public void runVolts(double motorVolts) {
        leaderMotor.setControl(voltageOut.withOutput(motorVolts));
    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/NeutralOut.html
    @Override
    public void stop() {
        leaderMotor.setControl(neutralOut);
    }

    // call .setControl on the motor controller with the approrol modpriate conte and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/MotionMagicVelocityVoltage.html
    @Override
    public void runVelocity(double motorRPS) {
        leaderMotor.setControl(reqVelTorque.withVelocity(motorRPS));
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

    @Override
    public void setPID(double kP, double kI, double kD) {
        talonConfig.Slot0.kP = kP;
        talonConfig.Slot0.kI = kI;
        talonConfig.Slot0.kD = kD;
        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.getConfigurator().apply(talonConfig);
    }
}
