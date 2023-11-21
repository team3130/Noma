package frc.robot.commands.extensionarm;import edu.wpi.first.wpilibj2.command.CommandBase;import frc.robot.RobotContainer;import frc.robot.subsystems.ExtensionArm;/** An example command that uses an example subsystem. */public class DumbRetract extends CommandBase {@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})private final ExtensionArm m_ExtensionArm;public DumbRetract(ExtensionArm subsystem, RobotContainer container) {m_ExtensionArm = subsystem;addRequirements(subsystem);}@Override public void initialize() {}/** try to downsize some conditions to seperate methods in your subsystem -Giorgia*/@Override public void execute() {m_ExtensionArm.dumbSetSpeed(-1); }@Override public void end(boolean interrupted) {m_ExtensionArm.stop();}@Override public boolean isFinished() {return false;}}