package frc.robot.subsystems.intake.roller;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {

    @AutoLog
    class RollerIOInputs {
        public boolean motorConnected = true;

        public double motorPositionRads = 0.0;
        public double motorVelocityRPS = 0.0;
        public double motorAppliedVolts = 0.0;
        public double motorSupplyCurrentAmps = 0.0;
        public double motorTorqueCurrentAmps = 0.0;
        public double motorTempCelsius = 0.0;
    }

    default void updateInputs(RollerIOInputs inputs) {}

    default void runVolts(double motorVolts) {}

    default void stop() {}

    default void runVelocity(double motorRPS, double ff) {}

    default void setPID(double kP, double kI, double kD) {}
}
