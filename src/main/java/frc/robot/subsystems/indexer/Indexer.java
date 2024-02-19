// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.indexer;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
  /* Define constants here rather than common constants file for modularity */
  private static final int kIndexerMotorCANId = 14;
  private static final double kIndexerMaxVoltage = 10.0;

  private final WPI_TalonSRX m_indexerMotor = new WPI_TalonSRX(kIndexerMotorCANId);

  /** Creates a new Indexer. */
  public Indexer() {
    m_indexerMotor.configFactoryDefault();
    m_indexerMotor.configVoltageCompSaturation(kIndexerMaxVoltage);
    m_indexerMotor.enableVoltageCompensation(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
