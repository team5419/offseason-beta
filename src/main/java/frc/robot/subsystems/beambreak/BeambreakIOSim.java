package frc.robot.subsystems.beambreak;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

public class BeambreakIOSim implements BeambreakIO {

    private volatile boolean intakeTriggered = false;
    private volatile boolean handoffTriggered = false;
    private volatile boolean endTriggered = false;

    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor(r -> {
        Thread t = new Thread(r);
        t.setName("BeambreakIOSim-Scheduler");
        t.setDaemon(true);
        return t;
    });

    private ScheduledFuture<?> runningSequence = null;

    public BeambreakIOSim() {}

    @Override
    public void readInputs(BeambreakInputs inputs) {
        inputs.intakeTriggered = intakeTriggered;
        inputs.handoffTriggered = handoffTriggered;
        inputs.endTriggered = endTriggered;
    }

    public synchronized void initiateSequence() {
        if (runningSequence != null && !runningSequence.isDone()) {
            runningSequence.cancel(true);
        }

        runningSequence = scheduler.schedule(() -> setIntake(true), 500, TimeUnit.MILLISECONDS);

        scheduler.schedule(() -> setIntake(false), 2000, TimeUnit.MILLISECONDS);

        scheduler.schedule(() -> setHandoff(true), 2000, TimeUnit.MILLISECONDS);

        scheduler.schedule(() -> setEnd(true), 4000, TimeUnit.MILLISECONDS);

        scheduler.schedule(() -> setEnd(false), 6000, TimeUnit.MILLISECONDS);

        scheduler.schedule(() -> setHandoff(false), 6000, TimeUnit.MILLISECONDS);

    }

    public void setIntake(boolean val) {
        intakeTriggered = val;
    }

    public void setHandoff(boolean val) {
        handoffTriggered = val;
    }

    public void setEnd(boolean val) {
        endTriggered = val;
    }

    public void shutdown() {
        scheduler.shutdownNow();
    }
}
