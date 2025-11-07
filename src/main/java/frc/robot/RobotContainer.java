// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.DriveCommands;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.Ports;
import frc.robot.lib.RumbleThread;
import frc.robot.subsystems.apriltagvision.AprilTagVision;
import frc.robot.subsystems.beambreak.Beambreak;
import frc.robot.subsystems.beambreak.BeambreakIOReal;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.pivot.IntakePivot;
import frc.robot.subsystems.intake.pivot.IntakePivotIO;
import frc.robot.subsystems.intake.pivot.IntakePivotIOSim;
import frc.robot.subsystems.intake.pivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.intake.roller.IntakeRoller;
import frc.robot.subsystems.intake.roller.IntakeRollerIO;
import frc.robot.subsystems.intake.roller.IntakeRollerIOSim;
import frc.robot.subsystems.intake.roller.IntakeRollerIOTalonFX;
import frc.robot.subsystems.outtake.endeffector.EndEffector;
import frc.robot.subsystems.outtake.endeffector.EndEffectorIO;
import frc.robot.subsystems.outtake.endeffector.EndEffectorIOSim;
import frc.robot.subsystems.outtake.endeffector.EndEffectorIOTalonFX;
import frc.robot.subsystems.outtake.wrist.Wrist;
import frc.robot.subsystems.outtake.wrist.WristIO;
import frc.robot.subsystems.outtake.wrist.WristIOSim;
import frc.robot.subsystems.outtake.wrist.WristIOTalonFX;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.module.ModuleIOTalonFX;
import java.io.File;
import lombok.Getter;

public class RobotContainer {

    @Getter
    private CommandXboxController driver = new CommandXboxController(Ports.kDriverPort);

    @Getter
    private CommandXboxController operator = new CommandXboxController(Ports.kOperatorPort);

    @Getter
    private AprilTagVision aprilTagVision;

    @Getter
    private Elevator elevator;

    @Getter
    private IntakePivot intakePivot;

    @Getter
    private IntakeRoller intakeRoller;

    @Getter
    private Wrist wrist;

    @Getter
    private EndEffector endEffector;

    @Getter
    private Beambreak beamBreak;

    @Getter
    private Swerve swerve;

    @Getter
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Get driver station to stop
        DriverStation.silenceJoystickConnectionWarning(true);

        buildRobot();
        configNamedCommands();

        autoChooser = buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureDefaultCommands();

        if (GlobalConstants.kDevMode) {
            configureDevBindings();
        } else {
            configureDriverBindings();
            configureOperatorBindings();
        }

        RumbleThread.getInstance().bindControllers(driver, operator);
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(DriveCommands.joystickDrive(
                swerve,
                () -> driver.getLeftY(),
                () -> driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> driver.leftBumper().getAsBoolean()));
    }

    private void configureDriverBindings() {
        driver.start(); // ! Unbound
        driver.back(); // ! Unbound

        driver.a(); // ! Unbound
        driver.b(); // ! Unbound
        driver.x(); // ! Unbound
        driver.y(); // ! Unbound

        driver.povUp(); // ! Unbound
        driver.povDown(); // ! Unbound
        driver.povLeft(); // ! Unbound
        driver.povRight(); // ! Unbound

        driver.leftBumper(); // ! Unbound
        driver.rightBumper(); // ! Unbound

        driver.leftTrigger(0.1); // ! Unbound
        driver.rightTrigger(0.1); // ! Unbound
    }

    private void configureOperatorBindings() {
        operator.start(); // ! Unbound
        operator.back(); // ! Unbound

        operator.a(); // ! Unbound
        operator.b(); // ! Unbound
        operator.x(); // ! Unbound
        operator.y(); // ! Unbound

        operator.povUp(); // ! Unbound
        operator.povDown(); // ! Unbound
        operator.povLeft(); // ! Unbound
        operator.povRight(); // ! Unbound

        operator.leftBumper(); // ! Unbound
        operator.rightBumper(); // ! Unbound

        operator.leftTrigger(0.1); // ! Unbound
        operator.rightTrigger(0.1); // ! Unbound
    }

    /**
     * Use this function to test new features without
     * changing the current button bindings
     */
    private void configureDevBindings() {}

    private void buildRobot() {

        Elevator tempElevator = null;
        IntakePivot tempIntakePivot = null;
        IntakeRoller tempIntakeRoller = null;
        Wrist tempWrist = null;
        EndEffector tempEndEffector = null;
        Swerve tempSwerve = null;
        Beambreak tempBeamBreak = null;

        if (GlobalConstants.getMode() == GlobalConstants.Mode.REPLAY) return;
        switch (GlobalConstants.getRobotType()) {
            case BETA -> {
                tempElevator = new Elevator(new ElevatorIOTalonFX());
                tempIntakePivot = new IntakePivot(new IntakePivotIOTalonFX());
                tempIntakeRoller = new IntakeRoller(new IntakeRollerIOTalonFX());
                tempWrist = new Wrist(new WristIOTalonFX());
                tempEndEffector = new EndEffector(new EndEffectorIOTalonFX());
                tempBeamBreak = new Beambreak(new BeambreakIOReal());
                tempSwerve = new Swerve(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getFrontLeft()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getFrontRight()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getBackLeft()),
                        new ModuleIOTalonFX(SwerveConstants.TunerConstants.getBackRight()));
            }

            case SIMBOT -> {
                tempElevator = new Elevator(new ElevatorIOSim());
                tempIntakePivot = new IntakePivot(new IntakePivotIOSim());
                tempIntakeRoller = new IntakeRoller(new IntakeRollerIOSim());
                tempWrist = new Wrist(new WristIOSim());
                tempEndEffector = new EndEffector(new EndEffectorIOSim());
                tempSwerve = new Swerve(
                        new GyroIO() {},
                        new ModuleIOSim(SwerveConstants.TunerConstants.getFrontLeft()),
                        new ModuleIOSim(SwerveConstants.TunerConstants.getFrontRight()),
                        new ModuleIOSim(SwerveConstants.TunerConstants.getBackLeft()),
                        new ModuleIOSim(SwerveConstants.TunerConstants.getBackRight()));
            }
        }

        if (tempSwerve == null)
            tempSwerve = new Swerve(
                    new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
        if (tempElevator == null) tempElevator = new Elevator(new ElevatorIO() {});
        if (tempIntakePivot == null) tempIntakePivot = new IntakePivot(new IntakePivotIO() {});
        if (tempIntakeRoller == null) tempIntakeRoller = new IntakeRoller(new IntakeRollerIO() {});
        if (tempWrist == null) tempWrist = new Wrist(new WristIO() {});
        if (tempEndEffector == null) tempEndEffector = new EndEffector(new EndEffectorIO() {});
        if (tempBeamBreak == null) tempBeamBreak = new Beambreak(new BeambreakIOReal());

        swerve = tempSwerve;
        elevator = tempElevator;
        intakePivot = tempIntakePivot;
        intakeRoller = tempIntakeRoller;
        wrist = tempWrist;
        endEffector = tempEndEffector;
        beamBreak = tempBeamBreak;
    }

    /** Adds named commands to pathplanner */
    private void configNamedCommands() {
        NamedCommands.registerCommand(
                "Record Time", new InstantCommand(() -> RobotState.getInstance().setAutoFinished(true)));
        NamedCommands.registerCommand("Intake Coral", new InstantCommand());
        NamedCommands.registerCommand("Half Intake", new InstantCommand());
        NamedCommands.registerCommand("Score L1", new InstantCommand());
        NamedCommands.registerCommand("Score Coral", new InstantCommand());
        NamedCommands.registerCommand("Auto Align", new InstantCommand());
        NamedCommands.registerCommand("Handoff", new InstantCommand());      
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public SendableChooser<Command> buildAutoChooser() {
        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("Do nothing", new InstantCommand());
        File autosDir = new File(Filesystem.getDeployDirectory(), "pathplanner/autos");
        for (File f : autosDir.listFiles()) {
            if (!f.isDirectory()) {
                String fileName[] = f.getName().split("\\.");
                String autoName = fileName[0];
                chooser.addOption(
                        autoName, AutoBuilder.buildAuto(autoName).beforeStarting(() -> System.out.println(autoName)));
            }
        }
        return chooser;
    }
}
