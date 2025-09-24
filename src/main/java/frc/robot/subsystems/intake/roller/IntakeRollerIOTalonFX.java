package frc.robot.subsystems.intake.roller;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {

    private TalonFX leaderMotor, followerMotor;
    private TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    private Follower follow = new Follower(Ports.kIntakeRollerLeaderID, true);

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
    public void updateInputs(IntakeRollerIOInputs inputs) {}

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
