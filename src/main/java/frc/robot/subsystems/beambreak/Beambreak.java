package frc.robot.subsystems.beambreak;

import frc.robot.lib.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Beambreak extends VirtualSubsystem {

    private BeambreakInputsAutoLogged inputs = new BeambreakInputsAutoLogged();

    private BeambreakIO io;

    public Beambreak(BeambreakIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.readInputs(inputs);
        Logger.processInputs("Beambreaks", inputs);
    }

    public boolean coralInIntake() {
        return inputs.intakeTriggered;
    }

    public boolean coralEnteredHandoff() {
        return inputs.handoffTriggered;
    }

    public boolean gamepieceInEndEffector() {
        return inputs.endTriggered;
    }

    public void initiateSimulatedSequence() {
        ((BeambreakIOSim) io).initiateSequence();
    }
}
