// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.arm.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class SysIdRoutineBot {
  // The robot's subsystems
  private final Arm m_arm = new Arm();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);

  /**
   * Use this method to define bindings between conditions and commands. These are
   * useful for
   * automating robot behaviors based on button and sensor input.
   *
   * <p>
   * Should be called during {@link Robot#robotInit()}.
   *
   * <p>
   * Event binding methods are available on the {@link Trigger} class.
   */
  public void configureBindings() {
    m_arm.setDefaultCommand(m_arm.seekArmZero());
    m_driverController.leftBumper().whileTrue(m_arm.moveArm(() -> -m_driverController.getLeftY()));
    m_driverController.y().onTrue(m_arm.servoArmToTarget(ShooterConstants.kAmpPivotAngle));
    m_driverController.b().onTrue(m_arm.servoArmToTarget(ShooterConstants.kPodiumPivotAngle));
    m_driverController.a().onTrue(m_arm.servoArmToTarget(ShooterConstants.kSubwooferPivotAngle));
    // Use Commands factory to avoid requiring the arm subsystem.
    m_driverController.rightBumper().onTrue(Commands.runOnce(m_arm::hardSetEncoderToZero));
  }

  /**
   * Use this to define the command that runs during autonomous.
   *
   * <p>
   * Scheduled during {@link Robot#autonomousInit()}.
   */
  public Command getAutonomousCommand() {
    // Do nothing except for subsystem initializations.
    return m_arm.runOnce(m_arm::hardSetEncoderToZero);
  }
}
