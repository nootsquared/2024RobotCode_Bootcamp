// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SHOOT_STATE;
import org.littletonrobotics.junction.Logger;

public class Pivot extends SubsystemBase {
  private final PivotIO pivot;
  private final PivotIOInputsAutoLogged pInputs = new PivotIOInputsAutoLogged();

  private static double kP;
  private static double kG;
  private static double kV;

  private static double maxVelocityDegPerSec;
  private static double maxAccelerationDegPerSecSquared;

  private TrapezoidProfile pivotProfile;
  private TrapezoidProfile.Constraints pivotConstraints;

  private TrapezoidProfile.State pivotGoal = new TrapezoidProfile.State();
  private TrapezoidProfile.State pivotCurrent = new TrapezoidProfile.State();

  double currTarget;
  double lastTarget;

  double goal;

  boolean isAimbot;
  SHOOT_STATE shootState;

  private ArmFeedforward pivotFFModel;

  /** Creates a new Pivot. */
  public Pivot(PivotIO pivot) {
    this.pivot = pivot;
    switch (Constants.getMode()) {
      case REAL:
        kG = 0.29;
        kV = 1;
        kP = 1.123;
        break;
      case REPLAY:
        kG = 0.29;
        kV = 1;
        kP = 1.123;
        ;
        break;
      case SIM:
        kG = 0.29;
        kV = 1;
        kP = 1.123;
        break;
      default:
        kG = 0.29;
        kV = 1;
        kP = 1.123;
        break;
    }

    isAimbot = true;

    shootState = SHOOT_STATE.AIMBOT;

    maxVelocityDegPerSec = 150;
    maxAccelerationDegPerSecSquared = 226;
    // maxAccelerationDegPerSecSquared = 180;

    pivotConstraints =
        new TrapezoidProfile.Constraints(maxVelocityDegPerSec, maxAccelerationDegPerSecSquared);
    pivotProfile = new TrapezoidProfile(pivotConstraints);

    // setPivotGoal(90);
    // setPivotCurrent(getPivotPositionDegs());
    pivotCurrent = pivotProfile.calculate(0, pivotCurrent, pivotGoal);

    pivot.configurePID(kP, 0, 0);
    pivotFFModel = new ArmFeedforward(0, kG, kV, 0);
  }

  public void setBrakeMode(boolean bool) {
    pivot.setBrakeMode(bool);
  }

  public double getPivotPositionDegs() {
    return pInputs.positionDegs;
  }

  public boolean atSetpoint() {
    return (Math.abs(getPivotError()) <= Constants.PivotConstants.THRESHOLD);
  }

  public boolean atGoal() {
    return (Math.abs(pInputs.positionDegs - goal) <= Constants.PivotConstants.THRESHOLD);
  }

  private double getPivotError() {
    return pInputs.positionSetpointDegs - pInputs.positionDegs;
  }

  public void setPositionDegs(double positionDegs, double velocityDegsPerSec) {
    positionDegs = MathUtil.clamp(positionDegs, 33, 120);
    pivot.setPositionSetpointDegs(
        positionDegs,
        pivotFFModel.calculate(Math.toRadians(positionDegs), Math.toRadians(velocityDegsPerSec)));
  }

  public void pivotStop() {
    pivot.stop();
  }

  public void setPivotGoal(double setpoint) {
    goal = setpoint;
    pivotGoal = new TrapezoidProfile.State(setpoint, 0);
  }

  // Create setPivotPosition here

  // Create getPivotPosition here
  // Intake -> 59.0
  // Stow -> 50.7

  public void setPivotPosition(Constants.PIVOT_POSITIONS position) {
    setPivotGoal(getPivotPosition(position));
  }

  public double getPivotPosition(Constants.PIVOT_POSITIONS position) {
    if (position == Constants.PIVOT_POSITIONS.INTAKE) {
      return 59.0;
    } else {
      return 50.7;
    }
  } 

  public void setPivotCurrent(double current) {
    pivotCurrent = new TrapezoidProfile.State(current, 0);
  }

  public boolean isAimbot() {
    return isAimbot;
  }

  public SHOOT_STATE getShootState() {
    return shootState;
  }

  public void setShootState(SHOOT_STATE shootState) {
    this.shootState = shootState;
  }

  public void setAimbot(boolean isAimbot) {
    this.isAimbot = isAimbot;
  }

  @Override
  public void periodic() {
    pivot.updateInputs(pInputs);

    pivotCurrent = pivotProfile.calculate(Constants.LOOP_PERIOD_SECS, pivotCurrent, pivotGoal);

    setPositionDegs(pivotCurrent.position, pivotCurrent.velocity);

    Logger.processInputs("Pivot", pInputs);
    Logger.recordOutput("pivot error", getPivotError());

    Logger.recordOutput("pivot goal", goal);
    // This method will be called once per scheduler run
  }
}
