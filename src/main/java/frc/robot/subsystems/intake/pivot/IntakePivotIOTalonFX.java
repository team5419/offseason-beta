package frc.robot.subsystems.intake.pivot;

import static edu.wpi.first.units.Units.Newton;
import static frc.robot.subsystems.intake.pivot.IntakePivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.google.gson.FieldNamingPolicy;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;

public class IntakePivotIOTalonFX implements IntakePivotIO {
    private TalonFX rollerMottor;
    private TalonFXConfiguration config;
    private final VoltageOut reqVoltage = new VoltageOut(0.0).withEnableFOC(true);
    private final NeutralOut reqNeutral = new NeutralOut();
    private final VelocityVoltage reqVelocity = new VelocityVoltage(0);
    private final StatusSignal<Angle> motorPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorSupplyCurrent;
    private final StatusSignal<Current> motorTorqueCurrent;
    private final StatusSignal<Temperature> motorTempCelsius;

public IntakePivotIOTalonFX(int id Strig canBus, boolean in)
       
       private TalonFX motor;
       
       public IntakePivotIOTalonFX() {
           motor = new TalonFX(Ports.kIntakePivotID, GlobalConstants.kCANivoreName);
           
                  config.Slot0.kP = kGains.kP();
                  config.Slot0.kI = kGains.kI();
                  config.Slot0.kD = kGains.kD();
                  config.Slot0.kS = kGains.kS();
                  config.Slot0.kA = kGains.kA();
                  config.Slot0.kG = kGains.kG();
                  config.Slot0.kV = kGains.kV();
        }

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        inputs.motorConnected = BaseStatusSignal.resfreshALL(
                            motorPosition,
                            motorVelocity,
                            motorAppliedVoltage,
                            motorSupplyCurrent
                            motorTorqueCurrent,
                            motorTempCelsius)
                        .isOK();
        inputs.positionRAD = motor.getPosition().getValueAsDoubled();
        
                            
          }
        
    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/MotionMagicDutyCycle.html
    @Override
    public void runPosition(double goal) {

    }

    @Override
    public void resetPosition(double pos) {

    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/VoltageOut.html
    @Override
    public void runVolts(double volts) {
        motor.setControl(reqVoltage.withOutput(volts));
    }

    @Override
    public void stop() {
        motor.setControl(reqNeutral);
    }

    // call .setControl on the motor controller with the appropriate control mode and value.
    // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode
    @Override
    public void setBrakeMode(boolean enabled) {}

    @Override
    public void setPID(double kP, double kI, double kD) {}

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {}
}
