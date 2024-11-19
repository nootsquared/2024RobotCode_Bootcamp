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

  //add two static doubles here for the max velocity and max acceleration, make sure to include units in the name
  //dont define these yet 
  

  //make a TrapezoidProfile object called pivotProfile and a TrapezoidProfile.constraints object called pivotConstraints 
  //to make a constraint object just use "TrapezoidProfile.Constraints someConstraints;""
  //dont define these yet
 

  //make two TrapezoidProfile.state objects and give them empty objects
  //TrapezoidProfile.State someState --syntax
  // to make an empty object just set the object = new className(); keep the constructor empty 
  //be carefule when declaring this 
 

  //make one double called goalDegs or goalDegrees, this will be your "goal" position 
  

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

    //set your max velocity to 150 and max acceleration to 226 

    

    //finish the pivotConstraints object, put the max velocity and acceleration into its parameters 
    pivotConstraints =
        new TrapezoidProfile.Constraints();

        //finish the pivotProfile object and pass pivotConstraints in the parameter 
    pivotProfile = 

  
    //declare the pivots current pos state to pivotProfile.calculate(0, the current position state, the goal state)
    //very similar to a pid controller 
    pivotCurrentPosDegrees = 

    pivot.configurePID(kP, 0, 0);
    pivotFFModel = new ArmFeedforward(0, kG, kV, 0);
  }

  public void setBrakeMode(boolean bool) {
    pivot.setBrakeMode(bool);
  }


  public double getPivotPositionDegs() {
    return pInputs.positionDegs;
  }

 
  //we want to figure out if the pivot is within the range of our goal position, make a boolean atGoal method that returns whether the absolute value of
  //the current position - the goalDegs is less than or equal to Constants.PivotConstants.THRESHOLD
  public boolean atGoal() {
    return 
  }



 

  //set the position in degrees, pass in desired pos in degs and velocity in degs per second
  //use the MathUtil.clamp method to clamp the desired pos between 33 and 100
  //use the setPositionSetpointDegs method here, pass in positionDegs and velocityDegs in RADIANS before using the feedforward model
  public void setPositionDegs(double positionDegs, double velocityDegsPerSec) {
    positionDegs = MathUtil.clamp();
    pivot.setPositionSetpointDegs();
      
  }

  public void pivotStop() {
    pivot.stop();
  }

  //make this.goalDegs = the parameter 
  //pivotGoal should now be goalDegs , 0 Velocity 
  //make a new object to do that 
  public void setPivotGoalDegrees(double goalDegs) {
  }

  //set the current position state to currentPosDegs and 0 velocity 
  public void setPivotCurrentDegrees(double currentPosDegs) {
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
  
    //set the pivotCurrentPosDegrees to the profile.calculate(CONSTANTS.LOOP_PERIOD_SECS, pivotCurrentPosDegrees, pivotGoal)
    pivotCurrentPosDegrees = 

    //use the setPositionDegs method but pass in the currentPos's position and velocity 
    setPositionDegs();

    Logger.processInputs("Pivot", pInputs);
    Logger.recordOutput("pivot error", getPivotError());

    //log the goal degrees 
    
    // This method will be called once per scheduler run
  }
}
