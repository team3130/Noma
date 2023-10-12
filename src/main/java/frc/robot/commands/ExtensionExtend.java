// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ExtensionArm;

/** An example command that uses an example subsystem. */
public class ExtensionExtend extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExtensionArm m_ExtensionArm;
  private final RobotContainer m_robotContainer;
  private final double extensionFactor = 0.0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExtensionExtend(ExtensionArm subsystem, RobotContainer container) {
    m_ExtensionArm = subsystem;
    m_robotContainer = container;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_ExtensionArm.LimitSwitch()){
      m_ExtensionArm.resetEncoders();
      m_ExtensionArm.setZeroed(true);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ExtensionArm.isZeroed()&&(m_ExtensionArm.slowZone() == 0)){
      m_ExtensionArm.resetEncoders();
      double y = RobotContainer.m_WeaponsGamepad.getRawAxis(1);
      if (Constants.Extension.kExtensionDeadband >= Math.abs(y)) {
        y = 0;
      }
      if (m_ExtensionArm.LimitSwitch() && y <= 0) {
        y = 0;
      }
      if (m_ExtensionArm.getPosition() >= Constants.Extension.maxExtensionTicks && y >= 0) {
        y = 0;
      }
      m_ExtensionArm.Extension(y * m_ExtensionArm.slowZoneFactor());

    }
    else{
      double y = RobotContainer.m_WeaponsGamepad.getRawAxis(1);
      if (Constants.Extension.kExtensionDeadband >= Math.abs(y)) {
        y = 0;
      }
      if (m_ExtensionArm.LimitSwitch() && y <= 0) {
        y = 0;
      }
      if (m_ExtensionArm.getPosition() >= Constants.Extension.maxExtensionTicks && y >= 0) {
        y = 0;
      }
      m_ExtensionArm.Extension(Constants.Extension.slowExtensionSpeed);
    }

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
