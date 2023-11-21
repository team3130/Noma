package frc.robot.commands.extensionarm;import edu.wpi.first.wpilibj2.command.CommandBase;import frc.robot.RobotContainer;import frc.robot.subsystems.ExtensionArm;/** An example command that uses an example subsystem. */public class BaseExtension extends CommandBase {@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})private final ExtensionArm m_ExtensionArm;private final RobotContainer m_robotContainer;public BaseExtension(ExtensionArm subsystem, RobotContainer container) {m_ExtensionArm = subsystem;m_robotContainer = container;addRequirements(subsystem);}@Override public void initialize() {if(m_ExtensionArm.LimitSwitch()){ m_ExtensionArm.resetEncoders();m_ExtensionArm.setZeroed();}}@Override public void execute() {double y = RobotContainer.m_WeaponsGamepad.getRawAxis(1); m_ExtensionArm.setSpeed(m_ExtensionArm.rawMotorSpeed(y)); }@Override public void end(boolean interrupted) {}@Override public boolean isFinished() {return false;}}