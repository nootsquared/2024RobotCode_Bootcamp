// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED_STATE;
public class LED extends SubsystemBase {
  private final LED_IO ledIO;
  private final LED_IOInputsAutoLogged lInputs = new LED_IOInputsAutoLogged(); //All the inputs from the LED_IO class (this class filters for the inputs we want) (or in other words the LED inputs/data it gathers in general)

  public LED(LED_IO led) {
    // Set the LED_IO object to the one passed in
  }

  @Override
  public void periodic() {
    ledIO.updateInputs(lInputs);
    
    setState(lInputs.ledState);

    Logger.processInputs("LED Inputs", lInputs);
  }

  public void setState(/*take in LEDState as a parameter*/) {
    //using the ledIO object, set state to parameter you chose above
    Logger.recordOutput("Set State", state);
  }
}
