// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.NextShot;

/**
 * The single game state object holds information on the current state of our
 * game play. It includes that next desired shot and if we do or do not
 * currently hold a note.
 */
public class GameState {
    private static final GameState INSTANCE = new GameState();

    /** Do we have a note onboard? Start auto holding a note. */
    private boolean m_hasNote = true;
    /** What is our next planned shot? Default for basic autos. */
    private NextShot m_nextShot = NextShot.SPEAKER_CENTER;

    private GameState() {
    }

    public static GameState getInstance() {
        return INSTANCE;
    }

    /**
     * Sets the next shot to take. If null is passed, the shot is reset to its
     * initial speaker center state. The next shot setting is never allowed to be
     * null.
     * 
     * @param nextShot the desired next shot.
     */
    public void setNextShot(final NextShot nextShot) {
        m_nextShot = nextShot != null ? nextShot : NextShot.SPEAKER_CENTER;
    }

    /**
     * @return the next desired shot. Never null.
     */
    public NextShot getNextShot() {
        return m_nextShot;
    }

    /**
     * @return the angle the robot needs to be turned to for the next shot. This
     *         value is alliance adjusted. If the FMS is misbehaving, we assume
     *         blue.
     */
    public double getNextShotRobotAngle() {
        final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        final NextShot nextShot = getNextShot();
        return alliance == Alliance.Blue ? nextShot.m_blueSideBotHeading : nextShot.m_redSideBotHeading;
    }

    /**
     * Generally called by the indexer with true and the shooter with false.
     * 
     * @param hasNote the new setting for has note.
     */
    public void setHasNote(final boolean hasNote) {
        m_hasNote = hasNote;
    }

    /**
     * @return true if the robot is currently holding a note. False, otherwise.
     */
    public boolean hasNote() {
        return m_hasNote;
    }
}
