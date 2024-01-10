// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {

  private final WPI_TalonSRX m_manipulatorMotor8; // we should probably change these names once we learn more
    private final WPI_TalonSRX m_manipulatorMotor9; // we should probably change these names once we learn more

    private double intakeConeSpeed = .5;
  private double intakeCubeSpeed = -.5;
  private double outtakeConeSpeed = -.5;
  private double outtakeCubeSpeed = .5;
  private String intakeMode = "";


  public Manipulator() {
    m_manipulatorMotor8 = new WPI_TalonSRX(Constants.CAN.intakeMotor);
    m_manipulatorMotor9 = new WPI_TalonSRX(Constants.CAN.intake);

      m_manipulatorMotor8.configFactoryDefault();
    m_manipulatorMotor9.setInverted(true);
  }

  public void runMotor8() {
      m_manipulatorMotor8.set(ControlMode.PercentOutput, 0.5);
  }

    public void runMotor9() {
        m_manipulatorMotor9.set(ControlMode.PercentOutput, 0.5);
    }

  public double getSpeedIntakeCone() {
      return intakeConeSpeed;
  }

  public double getSpeedIntakeCube() {
      return intakeCubeSpeed;
  }

  public double getSpeedOuttakeCone() {
      return outtakeConeSpeed;
  }

  public double getSpeedOuttakeCube() {
      return outtakeCubeSpeed;
  }
  public String getIntakeMode() {
      return intakeMode;
  }

  public void setSpeedIntakeCone(double x) {
      intakeConeSpeed = x;
  }

  public void setSpeedIntakeCube(double x) {
      intakeCubeSpeed = x;
  }

  public void setSpeedOuttakeCone(double x) {
      outtakeConeSpeed = x;
  }

  public void setSpeedOuttakeCube(double x) {
      outtakeCubeSpeed = x;
  }

  public void StopManipulator() {
    m_manipulatorMotor9.set(ControlMode.PercentOutput, 0);
    m_manipulatorMotor8.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("Motor Speed: Intake Cone", this::getSpeedIntakeCone, this::setSpeedIntakeCone);
      builder.addDoubleProperty("Motor Speed: Intake Cube", this::getSpeedIntakeCube, this::setSpeedIntakeCube);
      builder.addDoubleProperty("Motor Speed: Outtake Cone", this::getSpeedOuttakeCone, this::setSpeedOuttakeCone);
      builder.addDoubleProperty("Motor Speed: Outtake Cube", this::getSpeedOuttakeCube, this::setSpeedOuttakeCube);
      builder.addStringProperty("Intake Mode", this::getIntakeMode, null);
  }

  @Override
  public void simulationPeriodic() {

  }
}
