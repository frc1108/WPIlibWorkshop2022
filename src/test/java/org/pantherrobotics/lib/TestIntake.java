// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.pantherrobotics.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
// import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestIntake {
  static final float DELTA = 1e-2F; // acceptable deviation range
  Intake m_intake;
  // SimDeviceSim m_simMotor;
  PWMSim m_simMotor;
  DoubleSolenoidSim m_simPiston;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    m_intake = new Intake(); // create our intake
    m_simMotor = new PWMSim(0);
        //new SimDeviceSim("CANSparkMax["+IntakeConstants.kMotorPort+"]"); // create our simulation PWM motor controller
    m_simPiston =
        new DoubleSolenoidSim(
            PneumaticsModuleType.CTREPCM,
            IntakeConstants.kSolenoidPorts[0],
            IntakeConstants.kSolenoidPorts[1]); // create our simulation solenoid
  }

  @SuppressWarnings("PMD.SignatureDeclareThrowsException")
  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    m_intake.close(); // destroy our intake object
  }

  @Test // marks this method as a test
  void doesntWorkWhenClosed() {
    m_intake.retractCommand(); // close the intake
    m_intake.activateCommand(() -> 0.5); // try to activate the motor
    assertEquals(0.0,
        // m_simMotor.getDouble("Velocity").get()
        m_simMotor.getSpeed(), DELTA,"Failed motor not running while retracted test"); // make sure that the value set to the motor is 0
  }

  @Test
  void worksWhenOpen() {
    m_intake.deployCommand();
    m_intake.activateCommand(() -> 0.5);
    assertEquals(0.5, 
    // m_simMotor.getDouble("Velocity").get()
    m_simMotor.getSpeed(), DELTA, "Failed to run motor while deployed test");
  }

  @Test
  void retractTest() {
    m_intake.retractCommand();
    assertEquals(DoubleSolenoid.Value.kForward, m_simPiston.get());
  }

  @Test
  void deployTest() {
    m_intake.deployCommand();
    assertEquals(DoubleSolenoid.Value.kForward, m_simPiston.get());
  }
}
