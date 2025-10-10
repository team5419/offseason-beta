package frc.robot.subsystems.intake.pivot;

import static frc.robot.subsystems.intake.pivot.IntakePivotConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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
    
    private TalonFX motor;
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
       
                private MotionMagicVoltage magicMotion = new MotionMagicVoltage(0);                         
                public IntakePivotIOTalonFX() {
                        motor = new TalonFX(Ports.kIntakePivotID, GlobalConstants.kCANivoreName);
                        motorPosition = motor.getPosition();
                        motorAppliedVoltage = motor.getMotorVoltage();
                        motorVelocity = motor.getVelocity();
                        motorTempCelsius = motor.getDeviceTemp();
                        motorSupplyCurrentAmps = motor.getSupplyCurrent();
                        motorReference = motor.getClosedLoopReference();
                        motorReferenceVelocity = motor.getClosedLoopReferenceSlope();
                        motorSupplyCurrent = motor.getSupplyCurrent();
                        motorTorqueCurrent = motor.getTorqueCurrent();
                        motorVoltage = motor.getMotorVoltage();
            
                    config = new TalonFXConfiguration();
        
                BaseStatusSignal.setUpdateFrequencyForAll(
                        GlobalConstants.kLooperHZ, // 50 hz
                        motorPosition,
                        motorVelocity,
                        motorAppliedVoltage,
                        motorSupplyCurrent,
                        motorTorqueCurrent,
                        motorTempCelsius);
            
                config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
                config.CurrentLimits.SupplyCurrentLimitEnable = true;
                config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
                config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; //May need to be changed
                config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                config.Feedback.SensorToMechanismRatio = 1;
        
                config.Slot0.kP = kGains.kP();
                config.Slot0.kI = kGains.kI();
                config.Slot0.kD = kGains.kD();
                config.Slot0.kS = kGains.kS();
                config.Slot0.kA = kGains.kA();
                config.Slot0.kG = kGains.kG();
                config.Slot0.kV = kGains.kV();
                motor.getConfigurator().apply(config);
            }
            

            @Override
            public void updateInputs(IntakePivotIOInputs inputs) {}
        
             // call .setControl on the motor controller with the appropriate control mode and value.
            // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/MotionMagicDutyCycle.html
            @Override
            public void runPosition(double goal) {
                motor.setControl(magicMotion.withEnableFOC(true));

            }
        
            @Override
            public void resetPosition(double pos) {}
        
             // call .setControl on the motor controller with the appropriate control mode and value.
            // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/controls/VoltageOut.html
            @Override
            public void runVolts(double volts) {}
        
            @Override
            public void stop() {}
        
            // call .setControl on the motor controller with the appropriate control mode and value.
            // https://api.ctr-electronics.com/phoenix6/release/java/com/ctre/phoenix6/configs/MotorOutputConfigs.html#NeutralMode
            @Override
            public void setBrakeMode(boolean enabled) {
                var MotorOutputConfigs = new TalonFXConfiguration().MotorOutput;
                MotorOutputConfigs.NeutralMode = enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;
            }
        
            @Override
            public void setPID(double kP, double kI, double kD) {
                config.Slot0.kP = kP;
                config.Slot0.kI = kI;
                config.Slot0.kD = kD;
                motor.getConfigurator().apply(config);
            }
        
            @Override
            public void setFF(double kS, double kG, double kV, double kA) {
               config.Slot0.kS = kS;
               config.Slot0.kG = kG;
               config.Slot0.kV = kV;
               config.Slot0.kA = kA;
               motor.getConfigurator().apply(config);
    }
}
