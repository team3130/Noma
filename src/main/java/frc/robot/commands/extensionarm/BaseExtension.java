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

  public BaseExtension(ExtensionArm subsystem, RobotContainer container) {
    m_ExtensionArm = subsystem;
    m_robotContainer = container;
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_ExtensionArm.LimitSwitch()){ // if limit switch is hit, then zero encoders
      m_ExtensionArm.resetEncoders();
      m_ExtensionArm.setZeroed();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double y = RobotContainer.m_WeaponsGamepad.getRawAxis(1); // get value of joystick
    m_ExtensionArm.runMotor(m_ExtensionArm.rawMotorSpeed(y)); //set speed based on joystick value; if arm moves to the ends, then set speed to zero and don't let them go further
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


