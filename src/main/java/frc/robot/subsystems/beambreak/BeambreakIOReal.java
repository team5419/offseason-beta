package frc.robot.subsystems.beambreak;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdiConfiguration;
import com.ctre.phoenix6.hardware.CANdi;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;
import frc.robot.lib.util.PhoenixUtil;

public class BeambreakIOReal implements BeambreakIO {

    private final CANdi intakeCandi;
    private final CANdi handoffCandi;
    private final CANdi endCandi;

    private final StatusSignal<Boolean> intakeBeambreakTriggered;
    private final StatusSignal<Boolean> handoffBeambreakTriggered;
    private final StatusSignal<Boolean> endBeambreakTriggered;

    private final CANdiConfiguration config = PhoenixUtil.createCandiConfiguration();

    public BeambreakIOReal() {

        intakeCandi = new CANdi(Ports.kIntakeBeambreakPort, GlobalConstants.kCANivoreName);

        // both of these are on the end effector
        handoffCandi = new CANdi(Ports.kHandoffBeambreakPort, GlobalConstants.kCANivoreName);
        endCandi = new CANdi(Ports.kEndBeambreakPort, GlobalConstants.kCANivoreName);

        intakeCandi.getConfigurator().apply(config);
        handoffCandi.getConfigurator().apply(config);
        endCandi.getConfigurator().apply(config);

        intakeBeambreakTriggered = intakeCandi.getS1Closed();
        endBeambreakTriggered = handoffCandi.getS1Closed();
        handoffBeambreakTriggered = endCandi.getS1Closed();

        BaseStatusSignal.setUpdateFrequencyForAll(
                GlobalConstants.kLooperHZ, intakeBeambreakTriggered, handoffBeambreakTriggered, endBeambreakTriggered);
    }

    @Override
    public void readInputs(BeambreakInputs inputs) {
        BaseStatusSignal.refreshAll(intakeBeambreakTriggered, handoffBeambreakTriggered, endBeambreakTriggered);

        inputs.intakeTriggered = intakeBeambreakTriggered.getValue();
        inputs.handoffTriggered = handoffBeambreakTriggered.getValue();
        inputs.endTriggered = endBeambreakTriggered.getValue();
    }
}
