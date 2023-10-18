// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class RotaryArm extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  private final Solenoid m_solenoid;

  private Boolean defaultState;

  public RotaryArm() {
    m_solenoid = new Solenoid(Constants.RotaryArm.CAN_PNM, PneumaticsModuleType.CTREPCM , Constants.RotaryArm.PNM_Rotary);

    // default should be whatever retracted is
    defaultState = false;
    m_solenoid.set(defaultState);
  }

  /**
   * Toggles the pnumatic for the pistons
   */
  public void toggle() {
    m_solenoid.toggle();
  }
  
  /**
   * Extends the pneumatic (opposite of retracted which we are saying is the default state)
   */
  public void extend() {
    m_solenoid.set(!defaultState);
  }

  /**
   * Retracts the small pneumatic (same as the default state)
   */
  public void retract() {
    m_solenoid.set(defaultState);
  }

  /**
   * @return the state of the small pneumatic (retracted should be false)
   */
  public boolean isExtended() {
    return m_solenoid.get() ^ defaultState;
  }

  /**
   * setter for the pneumatics state
   * @param state false is retracted, true is extended
   */
  private void setState(boolean state) {
    m_solenoid.set(state ^ defaultState);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addBooleanProperty("isExtended", this::isExtended, this::setState);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
