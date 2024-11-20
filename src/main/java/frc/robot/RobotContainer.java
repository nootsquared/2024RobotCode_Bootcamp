// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LED_STATE;
import frc.robot.Constants.NOTE_POSITIONS;
import frc.robot.Constants.NoteState;
import frc.robot.Constants.SHOOT_STATE;
import frc.robot.commands.AimbotAuto;
import frc.robot.commands.AimbotStatic;
import frc.robot.commands.AimbotTele;
import frc.robot.commands.AlignToNoteAuto;
import frc.robot.commands.AngleShooter;
import frc.robot.commands.AngleShooterShoot;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PivotIntakeAuto;
import frc.robot.commands.PivotIntakeTele;
import frc.robot.commands.PositionNoteInFeeder;
import frc.robot.commands.ScoreAmp;
import frc.robot.commands.ScoreTrap;
import frc.robot.commands.SetAmpBarTarget;
import frc.robot.commands.SetElevatorTarget;
import frc.robot.commands.SetPivotTarget;
import frc.robot.commands.SetShooterTargetRPM;
import frc.robot.commands.ShootNoteAmp;
import frc.robot.commands.ShootNoteCenter;
import frc.robot.commands.ShootNoteSource;
import frc.robot.commands.StopIntakeFeed;
import frc.robot.commands.TurnToAmpCorner;
import frc.robot.commands.TurnToSpeaker;
import frc.robot.statemachines.ClimbStateMachine;
import frc.robot.statemachines.ClimbStateMachine.CLIMB_STATES;
import frc.robot.statemachines.TrapStateMachine;
import frc.robot.statemachines.TrapStateMachine.TRAP_STATES;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.VisionIO;
import frc.robot.subsystems.drive.VisionIOLimelight;
import frc.robot.subsystems.elevator.AmpBarIO;
import frc.robot.subsystems.elevator.AmpBarIOSIm;
import frc.robot.subsystems.elevator.AmpBarIOSparkMAX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOSparkFlex;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED_IO;
import frc.robot.subsystems.led.LED_IOCANdle;
import frc.robot.subsystems.led.LED_IOSim;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.shooter.DistanceSensorIO;
import frc.robot.subsystems.shooter.DistanceSensorIOAnalog;
import frc.robot.subsystems.shooter.FeederIOSim;
import frc.robot.subsystems.shooter.FeederIOTalonFX;
import frc.robot.subsystems.shooter.FlywheelIOSim;
import frc.robot.subsystems.shooter.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.LeafBlowerIO;
import frc.robot.subsystems.shooter.LeafBlowerIOTalonSRX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldConstants;
import java.util.Map;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private Intake intake;
  private Shooter shooter;
  private Elevator elevator;
  private LED led;
  private Pivot pivot;

  private final CommandXboxController driveController = new CommandXboxController(0);
  private final CommandXboxController manipController = new CommandXboxController(1);
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Command> autos;

  private final TrapStateMachine trapStateMachine;
  private final ClimbStateMachine climbStateMachine;

  private Trigger manipLeftBumper;
  private Trigger manipRightBumper;
  private Trigger manipRightTrigger;
  private Trigger manipLeftTrigger;
  private Trigger manipAButton;
  private Trigger manipBButton;

  private Trigger driveStartButton;
  private Trigger driveLeftBumper;
  private Trigger driveLeftTrigger;
  private Trigger driveRightBumper;
  private Trigger driveRightTrigger;
  private Trigger driveAButton;
  private Trigger driveXButton;
  private Trigger driveBButton;

  private final LoggedDashboardNumber flywheelSpeed = new LoggedDashboardNumber("fly soeed", 5400);

  private CLIMB_STATES climbSelect() {
    return climbStateMachine.getTargetState();
  }

  private TRAP_STATES trapSelect() {
    return trapStateMachine.getTargetState();
  }

  private SHOOT_STATE getShootState() {
    return pivot.getShootState();
  }

  private final Command climbCommands;

  // private final Command intakeLEDCommands;

  private final Command shootCommands;

  private final Command trapCommands;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.getMode()) {
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new VisionIOLimelight(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        intake = new Intake(new IntakeRollerIOSparkFlex(RobotMap.IntakeIDs.ROLLERS));
        shooter =
            new Shooter(
                new FlywheelIOTalonFX(
                    RobotMap.ShooterIDs.FLYWHEEL_LEFT, RobotMap.ShooterIDs.FLYWHEEL_RIGHT),
                new FeederIOTalonFX(RobotMap.ShooterIDs.FEEDER),
                new DistanceSensorIOAnalog(),
                new LeafBlowerIOTalonSRX(18));
        elevator =
            new Elevator(
                new ElevatorIOTalonFX(RobotMap.ElevatorIDs.LEFT, RobotMap.ElevatorIDs.RIGHT),
                new AmpBarIOSparkMAX(RobotMap.ElevatorIDs.BAR));
        pivot =
            new Pivot(
                new PivotIOTalonFX(
                    RobotMap.PivotIDs.LEFT, RobotMap.PivotIDs.RIGHT, RobotMap.PivotIDs.GYRO));
        led = new LED(new LED_IOCANdle(20, Constants.CANBUS));
        break;
      case REPLAY:
        drive =
            new Drive(
                new GyroIO() {},
                new VisionIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = new Intake(new IntakeRollerIOSim());
        shooter =
            new Shooter(
                new FlywheelIOSim(),
                new FeederIOSim(),
                new DistanceSensorIO() {},
                new LeafBlowerIO() {});
        elevator = new Elevator(new ElevatorIOSim(), null);
        pivot = new Pivot(new PivotIOSim());
        led = new LED(new LED_IOSim());
        break;
      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new VisionIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intake = new Intake(new IntakeRollerIOSim());
        shooter =
            new Shooter(
                new FlywheelIOSim(),
                new FeederIOSim(),
                new DistanceSensorIO() {},
                new LeafBlowerIO() {});
        elevator = new Elevator(new ElevatorIOSim(), new AmpBarIOSIm());
        pivot = new Pivot(new PivotIOSim());
        led = new LED(new LED_IOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        intake = new Intake(new IntakeRollerIOSparkFlex(RobotMap.IntakeIDs.ROLLERS));
        drive =
            new Drive(
                new GyroIO() {},
                new VisionIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter =
            new Shooter(
                new FlywheelIOTalonFX(
                    RobotMap.ShooterIDs.FLYWHEEL_LEFT, RobotMap.ShooterIDs.FLYWHEEL_RIGHT),
                new FeederIOTalonFX(RobotMap.ShooterIDs.FEEDER),
                new DistanceSensorIO() {},
                new LeafBlowerIO() {});
        elevator = new Elevator(new ElevatorIO() {}, new AmpBarIO() {});
        pivot = new Pivot(new PivotIO() {});
        led = new LED(new LED_IO() {});
        break;
    }

    climbStateMachine = new ClimbStateMachine(elevator, shooter, pivot);
    trapStateMachine = new TrapStateMachine(elevator, shooter, pivot);

    manipLeftBumper = manipController.leftBumper();
    manipRightBumper = manipController.rightBumper();
    manipRightTrigger = manipController.rightTrigger();
    manipLeftTrigger = manipController.leftTrigger();
    manipAButton = manipController.a();
    manipBButton = manipController.b();

    driveStartButton = driveController.start();
    driveLeftBumper = driveController.leftBumper();
    driveLeftTrigger = driveController.leftTrigger();
    driveRightBumper = driveController.rightBumper();
    driveRightTrigger = driveController.rightTrigger();
    driveAButton = driveController.a();
    driveXButton = driveController.x();
    driveBButton = driveController.b();

    // intakeLEDCommands =
    // new SelectCommand<>(
    // Map.ofEntries(
    // Map.entry(false, new InstantCommand(() -> led.setState(LED_STATE.YELLOW),
    // led)),
    // Map.entry(true, new InstantCommand(() -> led.setState(LED_STATE.BLUE),
    // led))),
    // this::isAutoAlign);

    shootCommands =
        new SelectCommand<>(
            Map.ofEntries(
                Map.entry(
                    SHOOT_STATE.PIVOT_PRESET,
                    new InstantCommand(() -> shooter.setFeedersRPM(1000))),
                Map.entry(
                    SHOOT_STATE.AIMBOT,
                    new SequentialCommandGroup(
                        new SetElevatorTarget(0, 1.5, elevator),
                        new AimbotTele(drive, driveController, shooter, pivot, led))),
                Map.entry(
                    SHOOT_STATE.AMP,
                    new SequentialCommandGroup(
                        // amp shoot
                        new InstantCommand(() -> shooter.setFeedersRPM(500))
                        // new WaitCommand(1.323)
                        )),
                // new InstantCommand(() -> shooter.setFlywheelRPMs(-900, -900)))),
                Map.entry(
                    SHOOT_STATE.TRAP,
                    new SequentialCommandGroup(
                        // trap shoot
                        new InstantCommand(() -> shooter.turnOnFan()),
                        new InstantCommand(() -> shooter.setFlywheelRPMs(1020, 1020)),
                        new WaitUntilCommand(() -> shooter.atFlywheelSetpoints()),
                        new WaitCommand(1.5),
                        new InstantCommand(() -> shooter.setFeedersRPM(1000))))),
            this::getShootState);

    // climbCommands =
    //     new SelectCommand<>(
    //         Map.ofEntries(
    //             Map.entry(
    //                 CLIMB_STATES.NONE,
    //                 new SetPivotTarget(90, pivot)
    //                     .andThen(climbStateMachine::advanceTargetState, elevator)),
    //             Map.entry(
    //                 CLIMB_STATES.PIVOT_CLIMB,
    //                 new SetElevatorTarget(20.969, 1.5, elevator)
    //                     .andThen(climbStateMachine::advanceTargetState, elevator)),
    //             Map.entry(
    //                 CLIMB_STATES.RETRACT_CLIMB,
    //                 new ParallelCommandGroup(
    //                         // new SetPivotTarget(
    //                         // Constants.PivotConstants.CLIMB_SETPOINT_ONE_DEG, pivot),
    //                         new SetElevatorTarget(0, 3, elevator))
    //                     .andThen(new InstantCommand(() -> pivot.setBrakeMode(true)))
    //                     // .andThen(new SetPivotTarget(90, pivot))
    //                     .andThen(climbStateMachine::advanceTargetState, elevator)),
    //             Map.entry(
    //                 CLIMB_STATES.ENGAGE_STATIC_HOOKS,
    //                 new SequentialCommandGroup(
    //                     new SetPivotTarget(90, pivot),
    //                     new ParallelCommandGroup(
    //                         new SetPivotTarget(
    //                             Constants.PivotConstants.CLIMB_SETPOINT_TWO_DEG, pivot),
    //                         new SetElevatorTarget(15.1, 1.5, elevator)),
    //                     new InstantCommand(climbStateMachine::advanceTargetState))),
    //             Map.entry(
    //                 CLIMB_STATES.ENGAGE_LOWER_SHOOTER_HOOKS,
    //                 new SequentialCommandGroup(
    //                     new SetPivotTarget(92.4554, pivot),
    //                     new ParallelCommandGroup(
    //                         new SetPivotTarget(85, pivot),
    //                         new SetElevatorTarget(-1, 2.26, elevator)),
    //                     new InstantCommand(climbStateMachine::advanceTargetState))),
    //             Map.entry(
    //                 CLIMB_STATES.ENGAGE_LOWER_SHOOTER_HOOKS_PART_TWO,
    //                 new SequentialCommandGroup(
    //                     new SetPivotTarget(87, pivot),
    //                     new SequentialCommandGroup(new SetElevatorTarget(5, 1.5, elevator)),
    //                     new InstantCommand(climbStateMachine::advanceTargetState))),
    //             Map.entry(
    //                 CLIMB_STATES.ALIGN_TO_TRAP,
    //                 new SequentialCommandGroup(
    //                         new SetElevatorTarget(20.75, .5, elevator),
    //                         new SetPivotTarget(111, pivot),
    //                         new SetShooterTargetRPM(502, 502, shooter))
    //                     .andThen(new InstantCommand(climbStateMachine::advanceTargetState))),
    //             Map.entry(
    //                 CLIMB_STATES.SHOOT_NOTE, new InstantCommand(() ->
    // shooter.setFeedersRPM(500))),
    //             Map.entry(CLIMB_STATES.DONE, new PrintCommand("hi"))),
    //         this::climbSelect);

    climbCommands =
        new SelectCommand<>(
            Map.ofEntries(
                Map.entry(
                    CLIMB_STATES.NONE,
                    new SetPivotTarget(90, pivot)
                        .andThen(climbStateMachine::advanceTargetState, elevator)),
                Map.entry(
                    CLIMB_STATES.EXTEND,
                    new SetElevatorTarget(
                            Constants.ElevatorConstants.EXTEND_SETPOINT_INCH, 1.5, elevator)
                        .andThen(climbStateMachine::advanceTargetState, elevator)),
                Map.entry(
                    CLIMB_STATES.RETRACT,
                    new SetElevatorTarget(0, 1.5, elevator)
                        .andThen(climbStateMachine::advanceTargetState, elevator))),
            this::climbSelect);

    trapCommands =
        new SelectCommand<>(
            Map.ofEntries(
                Map.entry(
                    TRAP_STATES.PIVOT,
                    new SetPivotTarget(Constants.PivotConstants.TRAP_SETPOINT_DEG, pivot)
                        .andThen(trapStateMachine::advanceTargetState, elevator)),
                Map.entry(
                    TRAP_STATES.EXTEND,
                    new SetElevatorTarget(
                            Constants.ElevatorConstants.EXTEND_SETPOINT_INCH, 1.5, elevator)
                        .andThen(new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.TRAP)))
                        .andThen(trapStateMachine::advanceTargetState, elevator)),
                Map.entry(
                    TRAP_STATES.RETRACT_STOW,
                    new SetElevatorTarget(0, 1.5, elevator)
                        .andThen(
                            new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))
                        .andThen(new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.AIMBOT)))
                        .andThen(trapStateMachine::advanceTargetState, elevator))),
            this::trapSelect);

    // PIVOT NAMED COMMANDS
    NamedCommands.registerCommand(
        "PivotShoot", new SetPivotTarget(Constants.PivotConstants.SUBWOOFER_SETPOINT_DEG, pivot));
    NamedCommands.registerCommand("PivotShootFar", new SetPivotTarget(39.4, pivot));
    NamedCommands.registerCommand(
        "PivotIntake",
        new PivotIntakeAuto(
                pivot, intake, shooter, led, Constants.PivotConstants.INTAKE_SETPOINT_DEG, false)
            .withTimeout(2));
    NamedCommands.registerCommand(
        "AutoPivotIntake", new PivotIntakeAuto(pivot, intake, shooter, led, 41.68, false));
    NamedCommands.registerCommand(
        "PivotSubwoofer",
        new SetPivotTarget(Constants.PivotConstants.SUBWOOFER_SETPOINT_DEG, pivot));

    // PIVOT NAMED COMMANDS (to speed up autoaim)
    NamedCommands.registerCommand("Pivot39", new SetPivotTarget(39, pivot));
    NamedCommands.registerCommand("Pivot45", new SetPivotTarget(45, pivot));
    NamedCommands.registerCommand("Pivot50", new SetPivotTarget(50, pivot));

    // SHOOT NOTE NAMED COMMANDS
    NamedCommands.registerCommand("ShootNoteCenter", new ShootNoteCenter(shooter));
    NamedCommands.registerCommand("ShootNoteSource", new ShootNoteSource(shooter));
    NamedCommands.registerCommand("ShootNoteAmp", new ShootNoteAmp(shooter));

    // FLYWHEEL NAMED COMMANDS
    NamedCommands.registerCommand(
        "StartFlywheelsCenter",
        new InstantCommand(() -> shooter.setFlywheelRPMs(4000.0, 4000.0), shooter));
    NamedCommands.registerCommand(
        "StartFlywheelsSource", new InstantCommand(() -> shooter.setFlywheelRPMSSource(), shooter));
    NamedCommands.registerCommand(
        "StartFlywheelsAmp", new InstantCommand(() -> shooter.setFlywheelRPMSAmp(), shooter));
    NamedCommands.registerCommand(
        "StartFlySlower", new InstantCommand(() -> shooter.setFlywheelRPMs(4050, 4050)));
    NamedCommands.registerCommand(
        "StopFlywheels", new InstantCommand(() -> shooter.stopFlywheels()));

    NamedCommands.registerCommand(
        "StartFeeders", new InstantCommand(() -> shooter.setFeedersRPM(4000), shooter));

    // STOP NAMED COMMANDS
    NamedCommands.registerCommand("StopIntakeFeed", new StopIntakeFeed(shooter, intake));
    NamedCommands.registerCommand(
        "StopFlywheels", new InstantCommand(shooter::stopFlywheels, shooter));
    NamedCommands.registerCommand("StopFeeders", new InstantCommand(shooter::stopFeeders, shooter));

    // OTHER NAMED COMMANDS
    NamedCommands.registerCommand(
        "PositionNoteInFeeder", new PositionNoteInFeeder(shooter, intake));
    NamedCommands.registerCommand(
        "RunIntake",
        new InstantCommand(
            () -> intake.runRollers(Constants.IntakeConstants.APPLIED_VOLTAGE), intake));

    // NOTE ALIGNMENT NAMED COMMANDS
    NamedCommands.registerCommand(
        "AlignToNote",
        new AlignToNoteAuto(led, drive, shooter, intake, pivot)
            .until(
                () ->
                    shooter.seesNote() == NoteState.SENSOR
                        || shooter.seesNote() == NoteState.CURRENT)
            // TODO:: adjust this delay
            .andThen(new InstantCommand(drive::stop))
            .andThen(new InstantCommand(() -> shooter.setFeedersRPM(500)))
            .andThen(new WaitCommand(0.15))
            .andThen(
                new InstantCommand(() -> intake.stopRollers())
                    .andThen(new InstantCommand(() -> shooter.stopFeeders())))
            .withTimeout(1.5));

    NamedCommands.registerCommand("C5", new InstantCommand(() -> drive.setNote(NOTE_POSITIONS.C5)));
    NamedCommands.registerCommand("C4", new InstantCommand(() -> drive.setNote(NOTE_POSITIONS.C4)));
    NamedCommands.registerCommand("C3", new InstantCommand(() -> drive.setNote(NOTE_POSITIONS.C3)));
    NamedCommands.registerCommand("C2", new InstantCommand(() -> drive.setNote(NOTE_POSITIONS.C2)));
    NamedCommands.registerCommand("C1", new InstantCommand(() -> drive.setNote(NOTE_POSITIONS.C1)));
    NamedCommands.registerCommand("B1", new InstantCommand(() -> drive.setNote(NOTE_POSITIONS.B1)));
    NamedCommands.registerCommand("B2", new InstantCommand(() -> drive.setNote(NOTE_POSITIONS.B2)));
    NamedCommands.registerCommand("B3", new InstantCommand(() -> drive.setNote(NOTE_POSITIONS.B3)));

    // AUTO AIM COMMANDS
    NamedCommands.registerCommand("TurnToSpeaker", new TurnToSpeaker(drive, driveController));
    NamedCommands.registerCommand("AngleShooter", new AngleShooter(drive, shooter, pivot));
    NamedCommands.registerCommand(
        "AngleShooterShoot",
        new AngleShooterShoot(drive, shooter, pivot)
            .andThen(
                new WaitCommand(1.5).andThen(new InstantCommand(() -> shooter.stopFeeders()))));
    NamedCommands.registerCommand(
        "AimbotStatic",
        new AimbotStatic(drive, driveController, shooter, pivot, led)
            .andThen(new InstantCommand(() -> led.setState(LED_STATE.BLUE))));
    NamedCommands.registerCommand("AimbotMoving", new AimbotAuto(drive, shooter, pivot, led));

    NamedCommands.registerCommand(
        "EnableOverride", new InstantCommand(() -> drive.enabledOverride()));
    NamedCommands.registerCommand(
        "DisableOverride", new InstantCommand(() -> drive.disableOverride()));

    NamedCommands.registerCommand(
        "test",
        new ConditionalCommand(
            new AlignToNoteAuto(led, drive, shooter, intake, pivot),
            drive
                .followPathCommand("c5 check", false)
                .andThen(new AlignToNoteAuto(led, drive, shooter, intake, pivot)),
            () -> drive.isNoteAt(FieldConstants.StagingLocations.centerlineTranslations[4])));

    // Set up auto routines
    autos = new SendableChooser<>();

    autos.addOption("$s!p-b3-c5-c4", AutoBuilder.buildAuto("$s!p-b3-c5-c4"));

    autos.addOption("$a!p-b1-c1-c2", AutoBuilder.buildAuto("$a!p-b1-c1-c2"));
    autos.addOption("$a!p-b1-c2-c3", AutoBuilder.buildAuto("$a!p-b1-c2-c3"));

    autos.addOption("$c!p-b3-b2-b1", AutoBuilder.buildAuto("$c!p-b3-b2-b1"));

    autos.addOption("test path", AutoBuilder.buildAuto("test path"));

    autos.addOption("$s!p-c5-c4", AutoBuilder.buildAuto("$s!p-c5-c4"));
    autos.addOption("Copy of $s!p-c5-c4", AutoBuilder.buildAuto("Copy of $s!p-c5-c4"));
    autos.addOption("$s!p-c4-c5", AutoBuilder.buildAuto("$s!p-c4-c5"));
    autos.addOption("$s!p-c4-c3", AutoBuilder.buildAuto("$s!p-c4-c3"));
    autos.addOption("$s!p-c3-c4", AutoBuilder.buildAuto("$s!p-c3-c4"));
    autos.addOption("$s!p-c3-c5", AutoBuilder.buildAuto("$s!p-c3-c5"));
    autos.addOption("$s!p-c5-c3", AutoBuilder.buildAuto("$s!p-c5-c3"));

    autos.addOption("$c!p-b2-c3", AutoBuilder.buildAuto("$c!p-b2-c3"));

    autos.addOption("New Auto", AutoBuilder.buildAuto("New Auto"));

    autos.addOption("New New Auto", AutoBuilder.buildAuto("New New Auto"));

    autos.addOption("conditional auto", AutoBuilder.buildAuto("conditional auto"));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", autos);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverControls();
    manipControls();

    // testControls();
  }

  private void testControls() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            () -> driveController.leftBumper().getAsBoolean(),
            () -> manipController.leftBumper().getAsBoolean()));

    driveController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    //     driveController.a().whileTrue(new TurnToSource(drive, driveController));

    // driveController.a().whileTrue(new DriveToChain(drive));
    driveController.a().onTrue(new InstantCommand(() -> shooter.setFlywheelRPMs(5700, 5300)));
    driveController.a().onFalse(new InstantCommand(() -> shooter.stopFlywheels()));

    driveController.x().onTrue(new SetPivotTarget(90, pivot));
    // manipController
    //     .a()
    //     .onTrue(
    //         new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.AMP))
    //             .andThen(new ScoreAmp(elevator, pivot, shooter, drive)));

    // manipController
    //     .a()
    //     .onFalse(
    //         new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.AIMBOT))
    //             .andThen(
    //                 new SequentialCommandGroup(
    //                     new InstantCommand(() -> shooter.turnOffFan()),
    //                     new SetElevatorTarget(0, 0.5, elevator),
    //                     new InstantCommand(() -> elevator.setConstraints(30, 85)),
    //                     new InstantCommand(() -> shooter.stopFlywheels(), shooter),
    //                     new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))));

    // driveController.b().whileTrue(new TurnToAmpCorner(drive, pivot, shooter,
    // driveController));

    // driveController.a().onTrue(new ScoreAmp(elevator, pivot, shooter, drive));
    // driveController
    // .a()
    // .onFalse(
    // new InstantCommand(() -> shooter.setFeedersRPM(200), shooter)
    // .andThen(new WaitCommand(2))
    // .andThen(new InstantCommand(() -> shooter.stopFlywheels(), shooter))

    // // .andThen(new SetElevatorTarget(10, elevator))
    // .andThen(new SetElevatorTarget(0, 1.5, elevator))
    // .andThen(new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG,
    // pivot))
    // .andThen(new InstantCommand(() -> shooter.stopFeeders(), shooter)));

    // driveController.x().onTrue(new InstantCommand(() ->
    // led.setState(LED_STATE.AUTO_ALIGN)));

    // driveController.b().onTrue(new InstantCommand(() ->
    // led.setState(LED_STATE.RED)));

    // driveController.a().onTrue(climbCommands);

    // driveController.a().whileTrue(new AlignToNoteAuto(drive, shooter, pivot,
    // intake, led,
    // 1.332));

    // driveController
    // .y()
    // .whileTrue(
    // Commands.startEnd(
    // () -> shooter.setFlywheelRPMs(flywheelSpeed.get(), flywheelSpeed.get()),
    // shooter::stopFlywheels,
    // shooter));
    // driveController.x().whileTrue(new TurnToSpeaker(drive, driveController));
    // driveController.x().onTrue(new InstantCommand(() ->
    // shooter.setFeedersRPM(1000)));
    // driveController.y().onTrue(new AimbotTele(drive, driveController, shooter,
    // pivot, led));
    driveController.y().onTrue(new SetPivotTarget(39, pivot));

    driveController.x().onTrue(new ScoreTrap(shooter, pivot));
    driveController
        .x()
        .onFalse(
            new InstantCommand(() -> shooter.turnOffFan(), shooter)
                .andThen(new InstantCommand(shooter::stopFeeders))
                .andThen(new InstantCommand(shooter::stopFlywheels)));
    // .andThen(new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot)));
    // driveController.a().onTrue(new TurnToSpeaker(drive, driveController));
    // driveController.a().onTrue(new SetPivotTarget(40, pivot));

    // drive
    // driveController
    // .y()
    // .onFalse(
    // new InstantCommand(shooter::stopFeeders)
    // .andThen(new InstantCommand(shooter::stopFlywheels)));
    // driveController.b().whileTrue(new AngleShooter(drive, shooter, pivot));
    // driveController
    // .b()
    // .onFalse(
    // new InstantCommand(() -> shooter.setFeedersRPM(1000))
    // .andThen(new WaitCommand(1))
    // .andThen(new InstantCommand(shooter::stopFlywheels))
    // .andThen(new InstantCommand(shooter::stopFeeders)));

    // driveController.a().onTrue(new PivotSource(pivot, intake, shooter, led));
    // driveController
    // .a()
    // .onFalse(
    // new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot)
    // .andThen(new InstantCommand(shooter::stopFeeders))
    // .andThen(new InstantCommand(shooter::stopFlywheels)));

    driveController
        .rightBumper()
        .onTrue(new PivotIntakeTele(pivot, intake, shooter, led, false, false));
    driveController
        .rightBumper()
        .onFalse(
            new InstantCommand(() -> shooter.setFeedersRPM(500))
                .andThen(new WaitCommand(0.15))
                .andThen(new InstantCommand(shooter::stopFeeders))
                .andThen(
                    new InstantCommand(shooter::stopFeeders)
                        .andThen(new InstantCommand(intake::stopRollers))));

    driveController
        .leftBumper()
        .whileTrue(new PivotIntakeAuto(pivot, intake, shooter, led, 41, true));
    driveController
        .leftBumper()
        .onFalse(
            new InstantCommand(intake::stopRollers)
                .andThen(new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))
                .andThen(new InstantCommand(() -> shooter.stopFeeders())));

    driveController
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> shooter.setFlywheelRPMs(flywheelSpeed.get(), flywheelSpeed.get())));
    driveController.rightTrigger().onFalse(new InstantCommand(() -> shooter.stopFlywheels()));

    // driveController.leftTrigger().onTrue(new InstantCommand(() -> shooter.setFeedersRPM(1000)));
    // driveController.leftTrigger().onTrue(new InstantCommand(() -> shooter.setFeedersRPM(200)));
    driveController.leftTrigger().onTrue(new InstantCommand(() -> shooter.setFeedersRPM(500)));
    driveController.leftTrigger().onFalse(new InstantCommand(() -> shooter.stopFeeders()));

    driveController.b().onTrue(new SetAmpBarTarget(10, 0, elevator));
    driveController
        .b()
        .onFalse(
            new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.AIMBOT))
                .andThen(
                    new SequentialCommandGroup(
                        new SetAmpBarTarget(5, 3, elevator),
                        new InstantCommand(() -> shooter.turnOffFan()),
                        new SetElevatorTarget(0, 0.5, elevator),
                        new InstantCommand(() -> elevator.setConstraints(30, 85)),
                        new InstantCommand(() -> shooter.stopFlywheels(), shooter),
                        new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))));
  }

  // TODO:: change drive controls to match changed test controls
  private void driverControls() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driveController.getLeftY(),
            () -> -driveController.getLeftX(),
            () -> -driveController.getRightX(),
            driveLeftBumper,
            manipLeftBumper));

    driveRightBumper.onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> climbStateMachine.setClimbState(CLIMB_STATES.NONE)),
            new InstantCommand(() -> trapStateMachine.setTargetState(TRAP_STATES.PIVOT)),
            new SetElevatorTarget(0, 1.5, elevator),
            DriveCommands.intakeCommand(
                drive,
                shooter,
                pivot,
                intake,
                led,
                driveController,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> -driveController.getRightX(),
                () -> false,
                manipLeftBumper)));

    driveRightBumper.onFalse(
        new InstantCommand(() -> led.setState(LED_STATE.BLUE))
            .andThen(new InstantCommand(() -> intake.changeLEDBoolFalse()))
            .andThen(new InstantCommand(() -> shooter.setFeedersRPM(500)))
            .andThen(new WaitCommand(0.02))
            .andThen(
                new ConditionalCommand(
                    new WaitCommand(0.1),
                    new WaitCommand(0.06),
                    () -> (shooter.getLastNoteState() == NoteState.CURRENT)))
            .andThen(
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.stopRollers(), intake),
                        new InstantCommand(() -> shooter.stopFeeders()),
                        new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))
                    .andThen(new PositionNoteInFeeder(shooter, intake))));

    driveStartButton.onTrue(
        Commands.runOnce(
                () ->
                    drive.setGyroPose(
                        new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                drive)
            .ignoringDisable(true));

    driveLeftBumper.onTrue(
        new SequentialCommandGroup(
            new InstantCommand(() -> climbStateMachine.setClimbState(CLIMB_STATES.NONE)),
            new InstantCommand(() -> trapStateMachine.setTargetState(TRAP_STATES.PIVOT)),
            new SetElevatorTarget(0, 1.5, elevator),
            DriveCommands.intakeCommand(
                drive,
                shooter,
                pivot,
                intake,
                led,
                driveController,
                () -> -driveController.getLeftY(),
                () -> -driveController.getLeftX(),
                () -> -driveController.getRightX(),
                () -> true,
                manipLeftBumper)));
    driveLeftBumper.onFalse(
        new InstantCommand(() -> led.setState(LED_STATE.BLUE))
            .andThen(new InstantCommand(() -> intake.changeLEDBoolFalse()))
            .andThen(new InstantCommand(() -> shooter.setFeedersRPM(500)))
            .andThen(new WaitCommand(0.02))
            .andThen(
                new ConditionalCommand(
                    new WaitCommand(0.1),
                    new WaitCommand(0.06),
                    () -> (shooter.getLastNoteState() == NoteState.CURRENT)))
            .andThen(
                new ParallelCommandGroup(
                        new InstantCommand(() -> intake.stopRollers(), intake),
                        new InstantCommand(() -> shooter.stopFeeders()),
                        new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))
                    .andThen(new PositionNoteInFeeder(shooter, intake))));

    driveLeftTrigger.whileTrue(new PivotIntakeTele(pivot, intake, shooter, led, true, false));
    driveLeftTrigger.onFalse(
        new InstantCommand(intake::stopRollers)
            .andThen(new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))
            .andThen(new InstantCommand(() -> shooter.stopFeeders())));

    driveRightTrigger.onTrue(shootCommands);
    driveRightTrigger.onFalse(
        new InstantCommand(() -> shooter.stopFeeders(), shooter)
            .andThen(new InstantCommand(() -> led.setState(LED_STATE.BLUE)))
            .andThen(new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))
            .andThen(
                new SetElevatorTarget(
                    Constants.ElevatorConstants.RETRACT_SETPOINT_INCH,
                    Constants.ElevatorConstants.THRESHOLD,
                    elevator))
            .andThen(new WaitCommand(0.5))
            .andThen(new InstantCommand(shooter::stopFlywheels))
            .andThen(new InstantCommand(() -> shooter.turnOffFan(), shooter)));

    driveAButton.onTrue(climbCommands);

    driveXButton.onTrue(trapCommands);
    // driveController
    //     .b()
    //     .onTrue(
    //         new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.AMP))
    //             .andThen(new ScoreAmp(elevator, pivot, shooter, drive)));

    // driveController
    //     .b()
    //     .onFalse(
    //         new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.AIMBOT))
    //             .andThen(
    //                 new SequentialCommandGroup(
    //                     new SetAmpBarTarget(5, 3, elevator),
    //                     new InstantCommand(() -> shooter.turnOffFan()),
    //                     new SetElevatorTarget(0, 0.5, elevator),
    //                     new InstantCommand(() -> elevator.setConstraints(30, 85)),
    //                     new InstantCommand(() -> shooter.stopFlywheels(), shooter),
    //                     new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))));
    // driveController
    //     .rightBumper()
    //     .whileTrue(
    //         new SequentialCommandGroup(
    //             new InstantCommand(() -> climbStateMachine.setClimbState(CLIMB_STATES.NONE)),
    //             new InstantCommand(() -> trapStateMachine.setTargetState(TRAP_STATES.PIVOT)),
    //             new SetElevatorTarget(0, 1.5, elevator),
    //             new AlignToNoteTele(intake, pivot, shooter, drive, led)));

    // driveController
    //     .rightBumper()
    //     .onFalse(
    //         new InstantCommand(() -> led.setState(LED_STATE.BLUE))
    //             .andThen(new InstantCommand(() -> shooter.setFeedersRPM(500)))
    //             .andThen(new WaitCommand(0.02))
    //             .andThen(
    //                 new ConditionalCommand(
    //                     new WaitCommand(0.24),
    //                     new WaitCommand(0.06),
    //                     () -> (shooter.getLastNoteState() == NoteState.CURRENT)))
    //             .andThen(
    //                 new ParallelCommandGroup(
    //                         new InstantCommand(() -> intake.stopRollers(), intake),
    //                         new InstantCommand(() -> shooter.stopFeeders()),
    //                         new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG,
    // pivot))
    //                     .andThen(new PositionNoteInFeeder(shooter, intake))));
  }

  private void manipControls() {
    // manipController
    //     .a()
    //     .onTrue(
    //         new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.AMP))
    //             .andThen(new ScoreAmp(elevator, pivot, shooter, drive)));
    // manipController
    //     .a()
    //     .onFalse(
    //         new InstantCommand(() -> shooter.setFeedersRPM(200), shooter)
    //             .andThen(new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.AIMBOT)))
    //             .andThen(new WaitCommand(2))
    //             .andThen(new InstantCommand(() -> shooter.stopFlywheels(), shooter))

    //             // .andThen(new SetElevatorTarget(10, elevator))
    //             .andThen(new SetElevatorTarget(0, 1.5, elevator))
    //             .andThen(new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))
    //             .andThen(new InstantCommand(() -> shooter.stopFeeders(), shooter)));

    manipAButton.onTrue(
        new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.AMP))
            .andThen(new ScoreAmp(elevator, pivot, shooter, drive)));

    manipAButton.onFalse(
        new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.AIMBOT))
            .andThen(
                new SequentialCommandGroup(
                    // new SetAmpBarTarget(5, 3, elevator),
                    new InstantCommand(() -> shooter.turnOffFan()),
                    new SetElevatorTarget(0, 0.5, elevator),
                    new InstantCommand(() -> elevator.setConstraints(30, 85)),
                    new InstantCommand(() -> shooter.stopFlywheels(), shooter),
                    new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))));
    // manipAButton.onTrue(
    //     new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.AMP))
    //         .andThen(new ScoreAmp(elevator, pivot, shooter, drive)));

    // manipAButton.onFalse(
    //     new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.AIMBOT))
    //         .andThen(
    //             new SequentialCommandGroup(
    //                 new InstantCommand(() -> shooter.turnOffFan()),
    //                 new SetElevatorTarget(0, 0.5, elevator),
    //                 new InstantCommand(() -> elevator.setConstraints(30, 85)),
    //                 new InstantCommand(() -> shooter.stopFlywheels(), shooter),
    //                 new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))));

    manipBButton.onTrue(
        new ParallelCommandGroup(
                new SetPivotTarget(Constants.PivotConstants.SUBWOOFER_SETPOINT_DEG, pivot),
                new SetShooterTargetRPM(
                    Constants.ShooterConstants.FLYWHEEL_SHOOT_RPM,
                    Constants.ShooterConstants.FLYWHEEL_SHOOT_RPM,
                    shooter))
            .andThen(new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.PIVOT_PRESET))));
    manipBButton.onFalse(
        new ParallelCommandGroup(
                new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot),
                new SetShooterTargetRPM(0, 0, shooter))
            .andThen(new InstantCommand(() -> pivot.setShootState(SHOOT_STATE.AIMBOT))));

    // manipController
    // .leftBumper()
    // .onTrue(new InstantCommand(() -> shooter.setFlywheelRPMs(3000, 3000),
    // shooter));
    // manipController.leftBumper().onFalse(new
    // InstantCommand(shooter::stopFlywheels, shooter));

    //     manipController
    //         .y()
    //         .onTrue(
    //             new ParallelCommandGroup(
    //                     new SetPivotTarget(
    //                         Constants.PivotConstants.REVERSE_SUBWOOFER_SETPOINT_DEG, pivot),
    //                     new SetShooterTargetRPM(
    //                         Constants.ShooterConstants.FLYWHEEL_SHOOT_RPM,
    //                         Constants.ShooterConstants.FLYWHEEL_SHOOT_RPM,
    //                         shooter))
    //                 .andThen(new InstantCommand(() -> pivot.setAimbot(false))));
    //     manipController
    //         .y()
    //         .onFalse(
    //             new ParallelCommandGroup(
    //                     new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot),
    //                     new SetShooterTargetRPM(0, 0, shooter))
    //                 .andThen(new InstantCommand(() -> pivot.setAimbot(true))));

    // manipRightTrigger.onTrue(new SetFeedersTargetRPM(1000, shooter));

    // manipRightTrigger.onFalse(new InstantCommand(() -> shooter.stopFeeders(), shooter));

    // manipLeftTrigger.whileTrue(new TurnToAmp(drive, driveController));

    // manipController.x().onTrue(new PivotSource(pivot, intake, shooter, led));

    // manipController
    //     .x()
    //     .onFalse(
    //         new InstantCommand(() -> led.setState(LED_STATE.BLUE), led)
    //             .andThen(new InstantCommand(shooter::stopFeeders, shooter))
    //             .andThen(new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))
    //             .andThen(new InstantCommand(shooter::stopFlywheels, shooter))
    //             .andThen(new InstantCommand(() -> shooter.setFeedersRPM(100)))
    //             .andThen(new WaitCommand(1))
    //             .andThen(new InstantCommand(shooter::stopFeeders, shooter)));

    manipRightBumper.whileTrue(new TurnToAmpCorner(drive, pivot, shooter, driveController));

    manipRightBumper.onFalse(
        new ParallelCommandGroup(
                new InstantCommand(shooter::stopFlywheels, shooter),
                new SetPivotTarget(Constants.PivotConstants.STOW_SETPOINT_DEG, pivot))
            .andThen(new InstantCommand(shooter::stopFeeders, shooter)));

    // manipController
    //     .leftBumper()
    //     .onTrue(new InstantCommand(() -> led.setState(LED_STATE.FLASHING_WHITE)));
    // manipController.leftBumper().onFalse(new InstantCommand(() -> led.setState(LED_STATE.BLUE)));

    // manipController.povUp().whileTrue(new TurnToSource(drive, driveController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Shooter getShooter() {
    return shooter;
  }

  public Pivot getPivot() {
    return pivot;
  }

  public Intake getIntake() {
    return intake;
  }

  public LED getLED() {
    return led;
  }

  public Drive getDrive() {
    return drive;
  }

  public ClimbStateMachine getClimbStateMachine() {
    return climbStateMachine;
  }
}
