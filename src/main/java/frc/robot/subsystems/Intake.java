// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase implements AutoCloseable {
  private final CANSparkMax m_motor = new CANSparkMax(IntakeConstants.kMotorPort,MotorType.kBrushless);
  private final DoubleSolenoid m_piston =
      new DoubleSolenoid(0,
          PneumaticsModuleType.CTREPCM,
          IntakeConstants.kSolenoidPorts[0],
          IntakeConstants.kSolenoidPorts[1]);

  /** Returns a command that deploys the intake, and then runs the intake motor indefinitely. */
  public CommandBase intakeCommand() {
    return runOnce(() -> m_piston.set(DoubleSolenoid.Value.kForward))
        .andThen(run(() -> m_motor.set(0.5)))
        .withName("Intake");
  }

  /** Returns a command to deploy the intake. */
  public CommandBase deployCommand() {
    return runOnce(() -> m_piston.set(DoubleSolenoid.Value.kForward))
        .withName("Deploy");
  }

  /** Returns a command that turns off and retracts the intake. */
  public CommandBase retractCommand() {
    return runOnce(
            () -> {
              m_motor.disable();
              m_piston.set(DoubleSolenoid.Value.kReverse);
            })
        .withName("Retract");
  }

  /** Returns a command that turns off and retracts the intake. */
  public CommandBase activateCommand(DoubleSupplier speed) {
    return run(
            () -> {
              if(isDeployed()) {
                m_motor.set(speed.getAsDouble());
              } else {
                m_motor.disable();
              }
              m_piston.set(DoubleSolenoid.Value.kReverse);
            })
        .withName("Activate");
  }

  public boolean isDeployed() {
    return m_piston.get() == DoubleSolenoid.Value.kForward;
  }

@Override
public void close() throws Exception {
  m_motor.close();
  m_piston.close();
}
}
