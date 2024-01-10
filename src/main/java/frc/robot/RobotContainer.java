// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.manipulator.OuttakeCube;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.framework.Autos;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Manipulator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Manipulator m_manipulator = new Manipulator();
  public static XboxController m_DriverGamepad = new XboxController(0);
  public static Joystick m_WeaponsGamepad = new Joystick(1);


  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_WeaponsGamepad = new Joystick(1);
    m_DriverGamepad = new XboxController(0);

    // Configure the trigger bindings
    configureBindings();
    //m_extension.setDefaultCommand(new BaseExtension(m_extension, this));
  }


  public Manipulator getManipulator() {
    return m_manipulator;
  }
  public void vomitShuffleBoardData() {
      ShuffleboardTab tab = Shuffleboard.getTab("Subsystems");
      tab.add(m_manipulator);
  }


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //new POVButton(m_WeaponsGamepad, Constants.XBOXButtons.LST_POV_N).whileTrue(new DumbExtend(m_extension, this));
    //new POVButton(m_WeaponsGamepad, Constants.XBOXButtons.LST_POV_S).whileTrue(new DumbRetract(m_extension, this));
    new JoystickButton(m_WeaponsGamepad, 6).whileTrue(new OuttakeCube(getManipulator()));
  }

  /**
   * Schedules a command to zero the extension arm
   */

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}