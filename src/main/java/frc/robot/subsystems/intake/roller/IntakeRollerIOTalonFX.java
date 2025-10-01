package frc.robot.subsystems.intake.roller;

import java.util.List;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.StatusSignal;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;
import java.util.stream.Stream;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {

    private final List<StatusSignal<Angle>> motorPosition;
    private final List<StatusSignal<AngularVelocity>> motorVelocity;
    private final List<StatusSignal<Voltage>> motorAppliedVoltage;
    private final List<StatusSignal<Current>> motorSupplyCurrent;
    private final List<StatusSignal<Current>> motorTorqueCurrent;
    private final List<StatusSignal<Temperature>> motorTempCelsius;

    private final boolean oppositeDirection = true;

    private TalonFX leaderMotor, followerMotor;
    private TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private Follower follow = new Follower(Ports.kIntakeRollerLeaderID, oppositeDirection); 

    motorPosition = List.of(leaderMotor.getPosition(),followerMotor.getPosition());
    motorVelocity = List.of(leaderMotor.getVelocity(),followerMotor.getVelocity());
    motorAppliedVoltage = List.of(leaderMotor.getVoltage(),followerMotor.getVoltage());
    motorSupplyCurrent = List.of(leaderMotor.getSupplyCurrent(),followerMotor.getSupplyCurrent());
    //motorTorqueCurrent = List.of(leaderMotor.getTorqueCurrent(),followerMotor.getTorqueCurrent());
    motorTempCelsius = List.of(leaderMotor.getDeviceTemp(),followerMotor.getDeviceTemp());

    private VelocityVoltage reqVelocity = new VelocityVoltage(0.0);

    public IntakeRollerIOTalonFX() {
        leaderMotor = new TalonFX(Ports.kIntakeRollerLeaderID, GlobalConstants.kCANivoreName);
        followerMotor = new TalonFX(Ports.kIntakeRollerFollowerID, GlobalConstants.kCANivoreName);

        talonConfig.Slot0.kP = IntakeRollerConstants.kGains.kP();
        talonConfig.Slot0.kI = IntakeRollerConstants.kGains.kI();
        talonConfig.Slot0.kD = IntakeRollerConstants.kGains.kD();

        leaderMotor.getConfigurator().apply(talonConfig);
        followerMotor.setControl(follow);
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        inputs.motorConnected = true;

        inputs.motorPositionRads = motorPosition.stream().mapToDouble(StatusSignal::getValueAsDouble)
        .toArray();
        inputs.motorAppliedVolts = motorAppliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble)
        .toArray();
        inputs.motorSupplyCurrentAmps = motorSupplyCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble)
        .toArray();
        inputs.motorTempCelsius = motorTempCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble)
        .toArray();
        inputs.motorVelocityRPS = motorVelocity.stream().mapToDouble(StatusSignal::getValueAsDouble)
        .toArray();
    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/VoltageOut.html
    @Override
    public void runVolts(double motorVolts) {
        leaderMotor.setVoltage(motorVolts);
    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/NeutralOut.html
    @Override
    public void stop() {
        leaderMotor.set(0);
    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/MotionMagicVelocityVoltage.html
    @Override
    public void runVelocity(double motorRPS, double ff) {
        leaderMotor.setControl(reqVelocity.withVelocity(motorRPS));
    }

    @Override
    public void setPID(double kP, double kI, double kD) {}
}
