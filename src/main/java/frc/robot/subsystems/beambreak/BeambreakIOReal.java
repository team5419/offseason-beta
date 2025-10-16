package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.Ports;

public class BeambreakIOReal implements BeambreakIO {

    private DigitalInput intake;
    private DigitalInput handoff;
    private DigitalInput end;

    public BeambreakIOReal() {

        intake = new DigitalInput(Ports.kIntakeBeambreakPort);

        // both of these are on the end effector
        handoff = new DigitalInput(Ports.kHandoffBeambreakPort);
        end = new DigitalInput(Ports.kEndBeambreakPort);
    }

    @Override
    public void readInputs(BeambreakInputs inputs) {
        inputs.intakeTriggered = intake.get();
        inputs.handoffTriggered = handoff.get();
        inputs.endTriggered = end.get();
    }
}
