package frc.robot.subsystems.intake.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {


    @AutoLog
    class IntakePivotIOInputs {
        public boolean motorConnected = true;
        public double position = 0.0; // in degrees
        public double appliedVolts = 0.0;
        public double velocity = 0.0; // in degrees per second
        public double tempCelcius = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double referencePose = 0.0;
        public double referenceVelocity = 0.0;
    }

    default void updateInputs(IntakePivotIOInputs inputs) {}

    default void runPosition(double goal) {}

    default void runVolts(double volts) {}

    default void resetPosition(double angle) {}

    default void setBrakeMode(boolean enabled) {}

    default void setPID(double P, double I, double D) {}

    default void setFF(double kA, double kG, double kS, double kV) {}

    default void stop() {}
}
