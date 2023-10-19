// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.manipulator;

import frc.robot.subsystems.Manipulator;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class OuttakeCone extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Manipulator m_manipulator;

  public OuttakeCone(Manipulator manipulator) {
    m_manipulator = manipulator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(manipulator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_manipulator.setIntakeMode(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_manipulator.Manipulate(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_manipulator.StopManipulator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
