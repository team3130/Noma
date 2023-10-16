// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.manipulator.IntakeCone;
import frc.robot.commands.manipulator.IntakeCube;
import frc.robot.commands.manipulator.OuttakeCone;
import frc.robot.commands.manipulator.OuttakeCube;
import frc.robot.commands.*;
import frc.robot.commands.extensionarm.AutoZeroExtensionArm;
import frc.robot.commands.extensionarm.ExtensionExtend;
import frc.robot.commands.framework.Autos;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Manipulator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ExtensionArm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Chassis m_chassis = new Chassis();
  private final Manipulator m_manipulator = new Manipulator();
  private final ExtensionArm m_extension = new ExtensionArm();
  public static Joystick m_DriverGamepad = new Joystick(0);
  public static Joystick m_WeaponsGamepad = new Joystick(1);

  private final XboxController m_weaponsGamepad;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_weaponsGamepad = new XboxController(1);

    // Configure the trigger bindings
    configureBindings();
    m_chassis.setDefaultCommand(new Drive(m_chassis, this));
    m_extension.setDefaultCommand(new ExtensionExtend(m_extension,this));
  }


  public Manipulator getManipulator() {
    return m_manipulator;
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


    new JoystickButton(m_WeaponsGamepad, 1).whileTrue(new AutoZeroExtensionArm(m_extension));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());


    new JoystickButton(m_weaponsGamepad, 3).whileTrue(new IntakeCone(getManipulator()));
    new JoystickButton(m_weaponsGamepad, 4).whileTrue(new IntakeCube(getManipulator()));
    new JoystickButton(m_weaponsGamepad, 5).whileTrue(new OuttakeCone(getManipulator()));
    new JoystickButton(m_weaponsGamepad, 6).whileTrue(new OuttakeCube(getManipulator()));
  }

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