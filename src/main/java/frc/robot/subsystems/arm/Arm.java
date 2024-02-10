// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The arm is used to move the indexer/shooter mechanism to the proper angle for
 * scoring. For return to 0, velocity control and hard stop detection is used.
 * For other movements, motion magic is used on the motor controller.
 */
public class Arm extends SubsystemBase {
  /* Define constants here rather than common constants file for modularity */
  private static final int kArmMotorCANId = 12;

  private static final int kMeasuredTicksWhenHorizontal = 0;
  private static final int kEncoderTickPerEncoderRotation = 4096;
  private static final double kEncoderToArmGearRatio = 60.0 / 16.0;
  private static final double kEncoderTicksPerArmRotation = kEncoderTickPerEncoderRotation * kEncoderToArmGearRatio;
  private static final double kEncoderTicksPerDegreeOfArmMotion = kEncoderTicksPerArmRotation / 360.0;
  private static final int kMotionMagicSlot = 0;
  private static final int kVelocitySlot = 1;
  private static final double kMaxGravityFF = 0.26; // In percent output [1.0:1.0]
  private static final double kF = 0.2;
  private static final double kPMotionMagic = 4.0;
  private static final double kPVelocity = 0.8;
  private static final double kIMotionMagic = 0.0;
  private static final double kDMotionMagic = 0.0;
  private static final double kCruiseVelocity = 1000.0; // ticks per 100ms
  private static final double kMaxAccel = 1000.0; // Accel to cruise in 1 sec
  private static final double kServoToleranceDegrees = 1.0; // +/- 1.0 for 2.0 degree window
  /** Velocity for safely zeroing arm encoder in native units (ticks) per 100ms */
  private static final double kZeroEncoderVelocity = -kEncoderTicksPerDegreeOfArmMotion * 5.0;
  private static final double kZeroingWaitForMoveSec = 2.0;
  private static final double kZeroingVelocityTolerance = 10.0;

  /** The motor controller for the arm. */
  private final WPI_TalonSRX m_armMotor = new WPI_TalonSRX(kArmMotorCANId);

  /** True when servo control active and false otherwise. */
  private boolean m_isServoControl = false;
  /** The last requested servo target for target checking. */
  private double m_lastServoTarget = 0.0;
  private boolean m_restingAtZero = false;

  /**
   * The motion magic parameters are configured in slot 0. Slot 1 is configured
   * for velocity control for zeroing.
   */
  public Arm() {
    m_armMotor.setSensorPhase(true);
    m_armMotor.config_kP(kMotionMagicSlot, kPMotionMagic);
    m_armMotor.config_kI(kMotionMagicSlot, kIMotionMagic);
    m_armMotor.config_kD(kMotionMagicSlot, kDMotionMagic);
    m_armMotor.config_kF(kMotionMagicSlot, kF);
    m_armMotor.configMotionCruiseVelocity(kCruiseVelocity);
    m_armMotor.configMotionAcceleration(kMaxAccel);

    m_armMotor.config_kP(kVelocitySlot, kPVelocity);
    m_armMotor.config_kI(kVelocitySlot, 0.0);
    m_armMotor.config_kD(kVelocitySlot, 0.0);
    m_armMotor.config_kF(kVelocitySlot, kF);
  }

  /**
   * Creates a command to seek the arm's zero position. This command is designed
   * to always be used for returning to and settling at zero. It should be the
   * subsystem's default command. The encoder will be reset to 0.
   * 
   * @return a command that will move the arm toward 0, and stop when stalled.
   */
  public Command seekArmZero() {
    return runOnce(() -> selectPIDSlot(kVelocitySlot))
        .andThen(run(() -> m_armMotor.set(
            ControlMode.Velocity,
            kZeroEncoderVelocity,
            DemandType.ArbitraryFeedForward,
            calculateGravityFeedForward()))
            .raceWith(Commands
                .waitSeconds(kZeroingWaitForMoveSec)
                .andThen(detectStallAtHardStop())))
        .andThen(restingAtZero())
        .withName("seekArmZero");
  }

  /**
   * Creates a command to detect stall at the hard stop during
   * {@link #seekArmZero()}.
   * 
   * @return the hard stop detection command.
   */
  private Command detectStallAtHardStop() {
    final Debouncer stallDebouncer = new Debouncer(1.0, DebounceType.kRising);
    return Commands.waitUntil(() -> stallDebouncer
        .calculate(MathUtil.isNear(
            0.0,
            m_armMotor.getSelectedSensorVelocity(kVelocitySlot),
            kZeroingVelocityTolerance)));
  }

  /**
   * Should only be called as the last step of {@link #seekArmZero()}. The encoder
   * is reset to 0.
   * 
   * @return a command to rest at the hard stop at 0 and on target.
   */
  private Command restingAtZero() {
    return runOnce(() -> {
      m_restingAtZero = true;
    })
        .andThen(run(() -> m_armMotor.set(0.0)).withTimeout(1.0))
        .andThen(this::hardSetEncoderToZero)
        .andThen(run(() -> m_armMotor.set(-0.1)))
        .finallyDo(() -> m_restingAtZero = false);
  }

  public void hardSetEncoderToZero() {
    m_armMotor.setSelectedSensorPosition(0);
  }

  /**
   * Creates a command to servo the arm to a desired angle. Note that 0 is
   * parallel to the ground. The entire operation is run with
   * {@link m_isServoControl} set to true to enable on target checking. See
   * {@link #isServoOnTarget(double)}.
   * 
   * <p>
   * If the target is 0 or less, the command from {@link #seekArmZero()} is
   * returned.
   * 
   * @param degrees the target degrees from 0. Must be positive.
   * @return a command that will servo the arm and will not end until interrupted
   *         by another command.
   */
  public Command servoArmToTarget(final double degrees) {
    if (degrees <= 0.0) {
      return seekArmZero();
    }
    final double targetSensorUnits = degrees * kEncoderTicksPerDegreeOfArmMotion;
    return runOnce(() -> {
      m_lastServoTarget = degrees;
      m_isServoControl = true;
      selectPIDSlot(kMotionMagicSlot);
    })
        .andThen(run(() -> m_armMotor.set(
            ControlMode.MotionMagic,
            targetSensorUnits,
            DemandType.ArbitraryFeedForward,
            calculateGravityFeedForward())))
        .finallyDo(() -> m_isServoControl = false)
        .withName("servoArmToTarget: " + degrees);
  }

  /**
   * This method should rarely be used. It is for pure manual control (no encoder
   * usage) which should be avoided.
   *
   * @param percentOutput the commanded output [-1.0..1.0]. Positive is up.
   * @return a command that drives the arm via double supplier.
   */
  public Command moveArm(final DoubleSupplier percentOutput) {
    return run(() -> m_armMotor.set(percentOutput.getAsDouble() * 0.4))
        .withName("moveArm");
  }

  /**
   * Assuming a properly zeroed arm (0 degrees is parallel to the ground), return
   * the current angle adjusted gravity feed forward.
   * 
   * @return the current angle adjusted gravity feed forward.
   */
  private double calculateGravityFeedForward() {
    final double degrees = getDegrees();
    final double radians = java.lang.Math.toRadians(degrees);
    final double cosineScalar = java.lang.Math.cos(radians);
    return kMaxGravityFF * cosineScalar;
  }

  /**
   * Assuming a properly zeroed arm (0 degrees is parallel to the ground), return
   * the current arm angle.
   * 
   * @return the current arm angle in degrees from 0.
   */
  private double getDegrees() {
    final double currentPos = m_armMotor.getSelectedSensorPosition();
    return (currentPos - kMeasuredTicksWhenHorizontal) / kEncoderTicksPerDegreeOfArmMotion;
  }

  /**
   * Returns true if the arm is under servo control and we are close to the last
   * requested target degrees.
   * 
   * @return true if under servo control and close, false otherwise.
   */
  public boolean isServoOnTarget() {
    return m_restingAtZero
        || (m_isServoControl
            && MathUtil.isNear(m_lastServoTarget, getDegrees(), kServoToleranceDegrees));
  }

  /**
   * Selects the specified slot for the next PID controlled arm movement. Always
   * selected for primary closed loop control.
   * 
   * @param slot the PID slot
   */
  private void selectPIDSlot(final int slot) {
    m_armMotor.selectProfileSlot(slot, 0);
  }

  /**
   * Updates the dashboard with critical arm data.
   */
  @Override
  public void periodic() {
    // TODO reduce this to essentials.
    SmartDashboard.putNumber("Arm degrees", getDegrees());
    SmartDashboard.putNumber("Arm current", m_armMotor.getStatorCurrent());
    SmartDashboard.putBoolean("Arm on target", isServoOnTarget());
    final Command currentCommand = getCurrentCommand();
    SmartDashboard.putString("Arm command", currentCommand != null ? currentCommand.getName() : "<null>");
    SmartDashboard.putNumber("Arm zeroing velocity", m_armMotor.getSelectedSensorVelocity(kVelocitySlot));
    SmartDashboard.putBoolean("Arm resting", m_restingAtZero);
    SmartDashboard.putBoolean("Servo control", m_isServoControl);
  }
}
