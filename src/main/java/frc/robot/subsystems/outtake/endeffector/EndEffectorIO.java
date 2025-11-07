package frc.robot.subsystems.outtake.endeffector;

import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
    @AutoLog
    class EndEffectorIOInputs {
        public boolean leaderMotorConnected = true;
        public boolean followerMotorConnected = true;
        public double[] position = {0.0, 0.0};
        public double[] appliedVolts = {0.0, 0.0};
        public double[] velocity = {0.0, 0.0};
        public double[] tempCelcius = {0.0, 0.0};
        public double[] supplyCurrentAmps = {0.0, 0.0};
        public double[] referencePose = {0.0, 0.0};
        public double[] referenceVelocity = {0.0, 0.0};
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
