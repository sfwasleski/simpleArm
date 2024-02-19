// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * 
 * <p>
 * NOTE: For modularity and ease of merging code from different subsystems,
 * contants use only within one subsystem (most of them), should be define in
 * the subsystem's implementation and not here.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class ShooterConstants {
    public static final double kSubwooferPivotAngle = 0.0;
    public static final double kPodiumPivotAngle = 45.0;
    public static final double kAmpPivotAngle = 90.0;
  }

  /**
   * An enumeration of known shot locations and data critical to executing the
   * shot. TODO decide on shooter velocity units and tune angles.
   */
  public enum NextShot {
    AMP(-90.0, 90.0, 90.0, 100.0),
    SPEAKER_AMP(45.0, -45.0, 0.0, 1000.0),
    SPEAKER_CENTER(0.0, 0.0, 0.0, 1000.0),
    SPEAKER_PODIUM(-45.0, 45.0, 0.0, 1000.0),
    PODIUM(-30.0, 30.0, 45.0, 2000.00);

    public final double m_blueSideBotHeading;
    public final double m_redSideBotHeading;
    public final double m_armAngle;
    public final double m_shooterVelocity;

    private NextShot(
        final double blueSideBotHeading,
        final double redSideBotHeading,
        final double armAngle,
        final double shooterVelocity) {
      m_blueSideBotHeading = blueSideBotHeading;
      m_redSideBotHeading = redSideBotHeading;
      m_armAngle = armAngle;
      m_shooterVelocity = shooterVelocity;
    }
  }
}
