package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ELEVATOR_POSITIONS;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO elevator;
  private final AmpBarIO ampBar;

  private final ElevatorIOInputsAutoLogged eInputs = new ElevatorIOInputsAutoLogged();
  private final AmpBarIOInputsAutoLogged aInputs = new AmpBarIOInputsAutoLogged();

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP");
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI");

  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS");
  private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG");
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV");
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA");

  // amp bar gains

  private static final LoggedTunableNumber barkP = new LoggedTunableNumber("Bar/kP");
  private static final LoggedTunableNumber barkV = new LoggedTunableNumber("Bar/kV");
  private static final LoggedTunableNumber barkG = new LoggedTunableNumber("Bar/kG");

  private TrapezoidProfile extenderProfile;
  private TrapezoidProfile.Constraints extenderConstraints =
      new TrapezoidProfile.Constraints(30, 85);
  private TrapezoidProfile.State extenderGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State extenderCurrent = new TrapezoidProfile.State();

  private static double maxVelocityDegPerSec;
  private static double maxAccelerationDegPerSecSquared;

  private TrapezoidProfile barProfile;
  private TrapezoidProfile.Constraints barConstraints;

  private TrapezoidProfile.State barGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State barCurrent = new TrapezoidProfile.State();

  private double goal;
  // private double barGoalPos;
  private final ElevatorFeedforward elevatorFFModel;
  private final ArmFeedforward barFFmodel;

  public Elevator(ElevatorIO elevator, AmpBarIO ampBar) {
    this.elevator = elevator;
    this.ampBar = ampBar;

    switch (Constants.getMode()) {
      case REAL:
        kS.initDefault(0);
        kG.initDefault(0.25);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(0.44);
        kI.initDefault(0);

        barkP.initDefault(0.25);
        barkV.initDefault(0.2);
        barkG.initDefault(0);
        break;
      case REPLAY:
        kS.initDefault(0);
        kG.initDefault(0.13);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(15);
        kI.initDefault(0);

        barkP.initDefault(0);
        barkV.initDefault(0);
        barkG.initDefault(0);
        break;
      case SIM:
        kS.initDefault(0);
        kG.initDefault(0.04);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(1);
        kI.initDefault(0);

        barkP.initDefault(0);
        barkV.initDefault(0);
        barkG.initDefault(0);
        break;
      default:
        kS.initDefault(0);
        kG.initDefault(0.13);
        kV.initDefault(0);
        kA.initDefault(0);

        kP.initDefault(15);
        kI.initDefault(0);

        barkP.initDefault(0);
        barkV.initDefault(0);
        barkG.initDefault(0);
        break;
    }

    setExtenderGoal(0.162);
    extenderProfile = new TrapezoidProfile(extenderConstraints);
    extenderCurrent = extenderProfile.calculate(0, extenderCurrent, extenderGoal);

    maxVelocityDegPerSec = 1200;
    maxAccelerationDegPerSecSquared = 1550;

    barConstraints =
        new TrapezoidProfile.Constraints(maxVelocityDegPerSec, maxAccelerationDegPerSecSquared);
    barProfile = new TrapezoidProfile(barConstraints);
    barCurrent = barProfile.calculate(0, barCurrent, barGoal);

    this.elevator.configurePID(kP.get(), 0, 0);
    elevatorFFModel = new ElevatorFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    this.ampBar.configurePID(barkP.get(), 0, 0);
    barFFmodel = new ArmFeedforward(0, barkG.get(), barkV.get(), 0);
  }

  public boolean atGoal() {
    return (Math.abs(eInputs.elevatorPosition - goal) <= Constants.ElevatorConstants.THRESHOLD);
  }

  public boolean barAtGoal() {
    return (Math.abs(aInputs.barPositionDegrees - barGoal.position)
        <= Constants.ElevatorConstants.BAR_THRESHOLD);
  }

  public double getElevatorPosition() {
    return eInputs.elevatorPosition;
  }

  private double getElevatorError() {
    return eInputs.positionSetpoint - eInputs.elevatorPosition;
  }

  private double getBarError() {

    return aInputs.barPositionSetpointDegrees - aInputs.barPositionDegrees;
  }

  private double getbarErrorToGoal() {
    return barGoal.position - aInputs.barPositionDegrees;
  }

  public boolean elevatorAtSetpoint() {
    return (Math.abs(getElevatorError()) <= Constants.ElevatorConstants.THRESHOLD);
  }

  public boolean ampBarAtGoal() {

    return (Math.abs(getbarErrorToGoal()) <= Constants.ElevatorConstants.BAR_THRESHOLD);
  }

  public void setBarBrakeMode(boolean bool) {
    ampBar.setBrakeMode(bool);
  }

  public double getBarPositionRotations() {
    return aInputs.barPositionDegrees;
  }

  public void setBarPosition(double positionDegrees, double velocityDegsPerSec) {

    // positionRotations = MathUtil.clamp(positionRotations, 0, 20);
    Logger.recordOutput("bar PositionDegrees", positionDegrees);
    ampBar.setPositionSetpoint(positionDegrees, 0);
    // barFFmodel.calculate(Math.toRadians(positionDegrees), Math.toRadians(velocityDegsPerSec))
  }

  public void stopAmpBar() {
    ampBar.stop();
  }

  public void setBarGoal(double barGoalDegrees) {

    barGoal = new TrapezoidProfile.State(barGoalDegrees, 0);
    Logger.recordOutput("bar goal", barGoalDegrees);
  }

  // public void setbarCurrent(double current) {

  //   barCurrent = new TrapezoidProfile.State(current, 0);
  // }

  public void setExtenderGoal(double setpoint) {
    goal = setpoint;
    extenderGoal = new TrapezoidProfile.State(setpoint, 0);
  }

  public void setExtenderPosition(Constants.ELEVATOR_POSITIONS position) {
    setExtenderGoal(getExtenderPosition(position));
  }

  public int getExtenderPosition(Constants.ELEVATOR_POSITIONS position) {
    if (position == ELEVATOR_POSITIONS.POS1) {
      return 1;
    } else if (position == ELEVATOR_POSITIONS.POS2) {
      return 2;
    } else {
      return 3;
    }
  }

  public void setPositionExtend(double position, double velocity) {
    elevator.setPositionSetpoint(position, elevatorFFModel.calculate(velocity));
  }

  public void elevatorStop() {
    elevator.stop();
  }

  public double calculateAngle() {
    double angle = 0.0;
    return angle;
  }

  public void setConstraints(
      double maxVelocityMetersPerSec, double maxAccelerationMetersPerSecSquared) {
    extenderConstraints =
        new TrapezoidProfile.Constraints(
            maxVelocityMetersPerSec, maxAccelerationMetersPerSecSquared);
    extenderProfile = new TrapezoidProfile(extenderConstraints);
  }

  public boolean isExtended() {
    return extenderGoal.position == Constants.ElevatorConstants.EXTEND_SETPOINT_INCH;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Alliance", DriverStation.getAlliance().isPresent());

    elevator.updateInputs(eInputs);
    ampBar.updateInputs(aInputs);

    extenderCurrent =
        extenderProfile.calculate(Constants.LOOP_PERIOD_SECS, extenderCurrent, extenderGoal);

    barCurrent = barProfile.calculate(Constants.LOOP_PERIOD_SECS, barCurrent, barGoal);

    setPositionExtend(extenderCurrent.position, extenderCurrent.velocity);

    setBarPosition(barCurrent.position, barCurrent.velocity);

    Logger.processInputs("Elevator", eInputs);
    Logger.processInputs("Amp bar inputs", aInputs);

    Logger.recordOutput("amp bar error", getBarError());
    Logger.recordOutput("amp bar goal", barGoal.position);

    Logger.recordOutput("amp bar currentPos", barCurrent.position);
    if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode())) {
      elevator.configurePID(kP.get(), kI.get(), 0);
    }
    if (barkP.hasChanged(hashCode())
        || barkV.hasChanged(hashCode())
        || barkG.hasChanged(hashCode())) {
      ampBar.configurePID(barkP.get(), 0, 0);
    }
  }
}
