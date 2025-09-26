package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.kGains;
import static frc.robot.subsystems.elevator.ElevatorConstants.kGearRatio;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class ElevatorIOTalonFX implements ElevatorIO {

    private TalonFX leaderMotor, followerMotor;
    private TalonFXConfiguration config = new TalonFXConfiguration();

    public ElevatorIOTalonFX() {
        leaderMotor = new TalonFX(Ports.kEleLeaderID, GlobalConstants.kCANivoreName);
        followerMotor = new TalonFX(Ports.kEleFollowerID, GlobalConstants.kCANivoreName);

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
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {}

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/MotionMagicDutyCycle.html
    @Override
    public void runPosition(double eleHeight, double feedforward) {
        leaderMotor.setControl(new MotionMagicDutyCycle(eleHeight));
    }

    @Override
    public void resetPosition(double pos) {
        leaderMotor.setPosition(0);
    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/VoltageOut.html
    @Override
    public void runVolts(double volts) {
        leaderMotor.setControl(new VoltageOut(volts));
    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode
    @Override
    public void stop() {
        leaderMotor.setControl(new NeutralOut());
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
