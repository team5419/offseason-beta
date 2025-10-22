package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {

    @AutoLog
    class IntakeRollerIOInputs {
        public boolean motorConnected = true;

        public double[] motorPositionRotations = new double[] {0.0, 0.0};
        public double[] motorVelocityRPS = new double[] {0.0, 0.0};
        public double[] motorAppliedVolts = new double[] {0.0, 0.0};
        public double[] motorSupplyCurrentAmps = new double[] {0.0, 0.0};
        public double[] motorTorqueCurrentAmps = new double[] {0.0, 0.0};
        public double[] motorTempCelsius = new double[] {0.0, 0.0};
    }

    default void updateInputs(IntakeRollerIOInputs inputs) {}

    default void runVolts(double motorVolts) {}

    default void stop() {}

    default void runVelocity(double motorRPS) {}

    default void setPID(double kP, double kI, double kD) {}

    default void setFF(double kA, double kG, double kS, double kV) {}
}
