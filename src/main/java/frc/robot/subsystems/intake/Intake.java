// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /* Define constants here rather than common constants file for modularity */
  private static final int kIntakeMotorCANId = 0;
  private static final double kIntakeMaxVoltage = 10.0;

  private final WPI_TalonSRX m_intakeMotor = new WPI_TalonSRX(kIntakeMotorCANId);

  /** Creates a new Intake. */
  public Intake() {
    m_intakeMotor.configFactoryDefault();
    m_intakeMotor.configVoltageCompSaturation(kIntakeMaxVoltage);
    m_intakeMotor.enableVoltageCompensation(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
