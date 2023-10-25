// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.extensionarm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExtensionArm;

/** An example command that uses an example subsystem. */
public class BaseExtension extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExtensionArm m_ExtensionArm;
  private final RobotContainer m_robotContainer;
  private final double extensionFactor = 0.0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public BaseExtension(ExtensionArm subsystem, RobotContainer container) {
    m_ExtensionArm = subsystem;
    m_robotContainer = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  public double rawMotorSpeed(double y) {
    if (m_ExtensionArm.getkExtensionDeadband() >= Math.abs(y)) { // if the fetched joystick value is less than the deadband value, then set speed to 0
      return 0;
    }
    if (allowedToMovePastEnds(y)) { // if arm hits the ends and is trying to move past the ends, then set speed to zero
      return 0;
    }
    return y;
  }

  public boolean allowedToMovePastEnds(double y){
    if (m_ExtensionArm.LimitSwitch() && (y <= 0)) { // if trying to move past the limit switch, return false
      return false;
    }
    else if ((m_ExtensionArm.getPosition() >= m_ExtensionArm.getMaxExtensionTicks()) && (y >= 0)) { // if trying to move past maxTicks, return false
      return false;
    }
    return true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_ExtensionArm.LimitSwitch()){
      m_ExtensionArm.resetEncoders();
      m_ExtensionArm.setZeroed();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = RobotContainer.m_WeaponsGamepad.getRawAxis(1); // get value from joystick
    m_ExtensionArm.setSpeed(rawMotorSpeed(y)); //set speed based on joystick value
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


