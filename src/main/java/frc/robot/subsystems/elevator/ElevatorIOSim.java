package frc.robot.subsystems.elevator;

import static frc.robot.subsystems.elevator.ElevatorConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.AutoLogOutput;

public class ElevatorIOSim implements ElevatorIO {

    private final PIDController controller;

    @AutoLogOutput(key = "Elevator/Sim/AppliedVolts")
    private double appliedVolts = 0.0;

    private final ElevatorSim eSim;

    public ElevatorIOSim() {
        controller = new PIDController(kGains.kP(), kGains.kI(), kGains.kD());
        eSim = new ElevatorSim(DCMotor.getKrakenX60(2), 10, 10, .1, 0, 7, false, 0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {}

    @Override
    public void runVolts(double volts) {
        // TODO: implement
    }

    @Override
    public void runPosition(double setpointMeters, double feedforward) {
        // TODO: implement
    }

    @Override
    public void stop() {
        // TODO: implement
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        // TODO: implement
    }
}
