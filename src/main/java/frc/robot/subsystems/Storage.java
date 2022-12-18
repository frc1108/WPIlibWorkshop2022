// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.StorageConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.pantherlib.PicoColorSensor;

public class Storage extends SubsystemBase {
  private final CANSparkMax m_motor = new CANSparkMax(StorageConstants.kMotorPort,MotorType.kBrushed);
  private final DigitalInput m_ballSensor = new DigitalInput(StorageConstants.kBallSensorPort);
   
  private final PicoColorSensor m_pico = new PicoColorSensor();
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = new Color(0.16, 0.427, 0.419);
  private final Color kRedTarget = new Color(0.561, 0.114, 0.34);
  private final Color kGreenTarget = new Color(0.197, 0.22, 0.59);
  private final Color kYellowTarget = new Color(0.33, 0.113, 0.55);
  /** Creates a new ColorSubsystem. */
  /** Create a new Storage subsystem. */
  public Storage() { 
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    // Set default command to turn off the storage motor and then idle
    setDefaultCommand(runOnce(m_motor::disable).andThen(run(() -> {})).withName("Idle"));
  }

  /** Whether the ball storage is full. */
  public boolean isFull() {
    if (m_pico.isSensor1Connected()) {
      Color detectedColor = m_pico.convertRawToColor(m_pico.getRawColor1());
      ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
      return (match.color == kRedTarget);
    } else {
    return false;
    }
  }

  /** Returns a command that runs the storage motor indefinitely. */
  public CommandBase runCommand() {
    return run(() -> m_motor.set(0.5)).withName("run");
  }
  /** Returns a command that runs the storage motor indefinitely. */
  public CommandBase stopCommand() {
    return run(() -> m_motor.set(0)).withName("stop");
  }
}
