package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
   private final ElevatorPivotIO pivot;
   private final ElevatorExtenderIO elevator;

   private final ElevatorPivotIOInputsAutoLogged pInputs = new ElevatorPivotIOInputsAutoLogged();
   private final ElevatorIOInputsAutoLogged eInputs = new ElevatorIOInputsAutoLogged();
   
   public Elevator(ElevatorPivotIO pivot, ElevatorExtenderIO elevator ){
      this.pivot = pivot;
      this.elevator = elevator;
   }

   public void setPositionElevator(double position) {
      elevator.setPosition(position);
   }

   public void setPositionPivot(double position) {
      pivot.setPosition(position);
   }

   public void setPivotVelocity(double pivotVelocity) {
      pivot.setVelocity(pivotVelocity);
   }

   public void setElevatorVelocity(double elevatorVelocity) {
      pivot.setVelocity(elevatorVelocity);
   }

   public void pivotStop(){
      pivot.stop();
   }

   public void elevatorStop(){
      elevator.stop();
   }

   public double convertTicksToDegrees(double ticks){
      return (ticks % 2048) * (360/2048);
   }

   public double convertAnglesToTicks(double angle){
      return (angle % 360) * 2048/360;
   }

   public double findDistance(double angle){
      return Constants.SHOOTER_LENGTH * Math.cos(convertTicksToDegrees(angle));
   }

   public double findHeight(double angle){
      return (Constants.SHOOTER_LENGTH * Math.sin(convertTicksToDegrees(angle))) + Constants.PIVOT_HEIGHT;
   }

   public double findAverageAngle(double currentPosition, double distance) {
      double shooterHeight = findHeight(convertTicksToDegrees(currentPosition)) + Constants.PIVOT_HEIGHT;
      double heightPerfect = 83.063 - (9.6 + shooterHeight);
      double minAngle = Math.atan2(heightPerfect, distance - 17.791);
      double maxAngle = Math.atan2(heightPerfect + 9.6, distance - 17.791);
      double averageAngle = (minAngle + maxAngle) / 2;
   
      return averageAngle;
   }

   @Override
   public void periodic() {
      pivot.updateInputs(pInputs);
      elevator.updateInputs(eInputs);

      Logger.processInputs("pivot Motor", pInputs);
      Logger.processInputs("elevate motor", eInputs);
   }
}
