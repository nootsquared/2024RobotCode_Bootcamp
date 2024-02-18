package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private final DCMotor simGearbox = DCMotor.getNeoVortex(1);
  private DCMotorSim sim = new DCMotorSim(simGearbox, 1, 0.7);
  private PIDController pid = new PIDController(0.2, 0, 0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double velocity = 0.0;
  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRPM()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.rollerVelocity = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVelocity(double velocity) {
    closedLoop = true;
    this.velocity = velocity;
    pid.setSetpoint(velocity);
  }

  @Override
  public void stop() {
    setVelocity(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}