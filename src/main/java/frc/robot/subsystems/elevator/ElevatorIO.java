package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    class ElevatorIOInputs {
        // u add these!
    }

    default void updateInputs(ElevatorIOInputs inputs) {}

    default void runPosition(double eleHeight, double feedforward) {}

    default void resetPosition(double pos) {}

    default void runVolts(double volts) {}

    default void runVelocity(double velocity) {}

    default void stop() {}

    default void setBrakeMode(boolean enabled) {}

    default void setPID(double kP, double kI, double kD) {}

    default void setFF(double kS, double kG, double kV, double kA) {}
}
