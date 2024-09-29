// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeRollerIO roller;

  private boolean ledBool;

  private final IntakeRollerIOInputsAutoLogged rInputs = new IntakeRollerIOInputsAutoLogged();

  public Intake(IntakeRollerIO roller) {
    this.roller = roller;
    ledBool = false;
  }

  public void runRollers(double volts) {
    roller.setVoltage(volts);
  }

  // make a new method called setRollerState with the input type being the enum you created in Constants.java
  // depending on the enum passed in, set the roller voltage to different values
  // intake -> 12 volts
  // outtake -> -12 volts
  // none -> 0 volts

  public void stopRollers() {
    roller.stop();
  }

  public void changeLEDBoolFalse() {
    ledBool = false;
  }

  public void changeLEDBoolTrue() {
    ledBool = true;
  }

  public boolean getLEDBool() {
    return ledBool;
  }

  public double getVolts() {
    return rInputs.appliedVolts;
  }

  public double getAmps() {
    return rInputs.currentAmps;
  }

  public double getRPM() {
    return rInputs.rollerVelocityRPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    roller.updateInputs(rInputs);

    Logger.processInputs("Intake", rInputs);
  }
}
