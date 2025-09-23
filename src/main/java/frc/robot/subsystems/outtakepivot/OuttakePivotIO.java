package frc.robot.subsystems.outtakepivot;

import org.littletonrobotics.junction.AutoLog;

public interface OuttakePivotIO {
    @AutoLog
    class OuttakePivotIOInputs {
        public boolean motorConnected = true;
        public double position = 0.0; // degrees
        public double appliedVolts = 0.0;
        public double velocity = 0.0; // degrees per second
        public double tempCelcius = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double referencePose = 0.0;
        public double referenceVelocity = 0.0;
    }

    default void updateInputs(OuttakePivotIOInputs inputs) {}

    /** runs the pivot to a specific position goal */
    default void runPosition(double goal) {}

    default void runVolts(double volts) {}

    default void resetPosition(double angle) {}

    default void setBrakeMode(boolean enabled) {}

    default void setPID(double P, double I, double D) {}

    default void setFF(double kA, double kG, double kS, double kV) {}

    default void stop() {}
}
