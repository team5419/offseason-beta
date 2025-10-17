package frc.robot.subsystems.beambreak;

import org.littletonrobotics.junction.AutoLog;

public interface BeambreakIO {

    @AutoLog
    public class BeambreakInputs {
        public boolean intakeTriggered;
        public boolean handoffTriggered;
        public boolean endTriggered;
    }

    public default void readInputs(BeambreakInputs inputs) {}
}
