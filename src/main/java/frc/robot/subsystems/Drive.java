// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class Drive extends SubsystemBase {
  private final CANSparkMax m_leftMain = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_leftFollow = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
  private final CANSparkMax m_rightMain = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
  private final CANSparkMax m_rightFollow = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMain, m_rightMain);

  // The left-side drive encoder
  private final RelativeEncoder m_leftEncoder;
  

  // The right-side drive encoder
  private final RelativeEncoder m_rightEncoder;
  

  /** Creates a new Drive subsystem. */
  public Drive() {
    m_leftEncoder = m_leftMain.getEncoder();
    m_rightEncoder = m_rightMain.getEncoder();
    m_leftFollow.follow(m_leftMain);
    m_rightFollow.follow(m_rightMain);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMain.setInverted(true);

    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
  }

  /**
   * Returns a command that drives the robot with arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public CommandBase arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
    // A split-stick arcade command, with forward/backward controlled by the left
    // hand, and turning controlled by the right.
    return run(() -> m_drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
        .withName("arcadeDrive");
  }

  /**
   * Returns a command that drives the robot forward a specified distance at a specified speed.
   *
   * @param distanceMeters The distance to drive forward in meters
   * @param speed The fraction of max speed at which to drive
   */
  public CommandBase driveDistanceCommand(double distanceMeters, double speed) {
    return runOnce(
            () -> {
              // Reset encoders at the start of the command
              m_leftEncoder.setPosition(0);
              m_rightEncoder.setPosition(0);
            })
        // Drive forward at specified speed
        .andThen(run(() -> m_drive.arcadeDrive(speed, 0)))
        // End command when we've traveled the specified distance
        .until(
            () ->
                Math.max(m_leftEncoder.getPosition(), m_rightEncoder.getPosition())
                    >= distanceMeters)
        // Stop the drive when the command ends
        .finallyDo(interrupted -> m_drive.stopMotor());
  }
}
