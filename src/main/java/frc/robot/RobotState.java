// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.NextShot;

/**
 * An instance of this class can evaluate if the robot subsystems are ready to
 * execute a game task.
 */
public class RobotState {
    public static final double kRobotHeadingTolerance = 2.0;
    public static final double kArmAngleTolerance = 1.0;
    public static final double kShooterVelocityTolerance = 10.0;
    private final DoubleSupplier m_robotHeadingSupplier;
    private final DoubleSupplier m_armAngleSupplier;
    private final DoubleSupplier m_shooterVelocitySupplier;
    private final GameState m_gameState = GameState.getInstance();

    /**
     * Constructed, typically in RobotContainer, with suppliers for up to date robot
     * state from the subsystems.
     * 
     * @param robotHeadingSupplier    a supplier of the robots current heading.
     * @param armAngleSupplier        a supplier of the current arm angle.
     * @param shooterVelocitySupplier a supplier of the current shooter velocity.
     */
    public RobotState(
            final DoubleSupplier robotHeadingSupplier,
            final DoubleSupplier armAngleSupplier,
            final DoubleSupplier shooterVelocitySupplier) {
        m_robotHeadingSupplier = robotHeadingSupplier;
        m_armAngleSupplier = armAngleSupplier;
        m_shooterVelocitySupplier = shooterVelocitySupplier;
    }

    /**
     * @return true if the current arm angle and shooter velocity are both within
     *         tolerance for the needs of the next desired shot.
     */
    public boolean isShooterReady() {
        final NextShot shot = m_gameState.getNextShot();
        return MathUtil.isNear(shot.m_armAngle, m_armAngleSupplier.getAsDouble(), kArmAngleTolerance)
                && MathUtil.isNear(shot.m_shooterVelocity, m_shooterVelocitySupplier.getAsDouble(),
                        kShooterVelocityTolerance);
    }

    /**
     * @return true if isShooterReady and the robot is turned correctly for the next
     *         shot.
     */
    public boolean isRobotReady() {
        return isShooterReady()
                && MathUtil.isNear(m_gameState.getNextShotRobotAngle(), m_robotHeadingSupplier.getAsDouble(),
                        kRobotHeadingTolerance);
    }

    /**
     * On our robot, the intake is physically dependent on the arm position for
     * intaking to work.
     * 
     * @return ture if we do not already have a note and the arm is in the proper
     *         intaking position.
     */
    public boolean isReadyToIntake() {
        return !m_gameState.hasNote()
                && MathUtil.isNear(0.0, m_armAngleSupplier.getAsDouble(), kArmAngleTolerance);
    }
}
