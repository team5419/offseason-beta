package frc.robot.subsystems.outtake.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    class EndEffectorIOInputs {
        public boolean motorConnected = true;
        public double[] appliedVolts = new double[] {0.0, 0.0};
        public double[] velocityRPS = new double[] {0.0, 0.0};
        public double[] tempCelcius = new double[] {0.0, 0.0};
        public double[] supplyCurrentAmps = new double[] {0.0, 0.0};
        public double[] referenceVelocityRPS = new double[] {0.0, 0.0};
    }

    default void updateInputs(EndEffectorIOInputs inputs) {}

    default void runVelocity(double rps) {}

    default void runVolts(double volts) {}

    default void resetPosition(double angle) {}

    default void setBrakeMode(boolean enabled) {}

    default void setPID(double P, double I, double D) {}

    default void setFF(double kA, double kG, double kS, double kV) {}

    default void stop() {}
}
