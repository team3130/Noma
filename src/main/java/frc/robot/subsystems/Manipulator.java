// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {

  private final WPI_TalonFX m_manipulatorMotor; // we should probably change these names once we learn more
  private double intakeConeSpeed;
  private double intakeCubeSpeed;
  private double outtakeConeSpeed;
  private double outtakeCubeSpeed;
  private double speed;
  private String intakeMode;


  public Manipulator() {
    m_manipulatorMotor = new WPI_TalonFX(Constants.CAN.manipulatorMotor);
    m_manipulatorMotor.configFactoryDefault();
    m_manipulatorMotor.setInverted(false);
  }

  public void setSpeed(double newSpeed) {
      speed = newSpeed;
      m_manipulatorMotor.set(ControlMode.PercentOutput, newSpeed);
  }

  public double getSpeed() {
      return speed;
  }

  public void setIntakeMode(int mode) {
      if(mode == 1) {
          intakeMode="Intake Cone";
      }
      else if(mode == 2) {
          intakeMode="Intake Cube";
      }
      else if(mode == 3 ){
          intakeMode="Outtake Cone";
      }
      else if(mode == 4) {
          intakeMode="Outtake Cone";
      }
  }
  public void intakeCone() {
      setSpeed(intakeConeSpeed);
      setIntakeMode(1);
      speed = intakeConeSpeed;
  }
  public void intakeCube() {
      setSpeed(intakeCubeSpeed);
      setIntakeMode(2);
      speed = intakeCubeSpeed;
  }
  public void outtakeCone() {
      setSpeed(outtakeConeSpeed);
      setIntakeMode(3);
      speed = outtakeConeSpeed;
  }
  public void outtakeCube() {
      setSpeed(outtakeCubeSpeed);
      setIntakeMode(4);
      speed = outtakeCubeSpeed;
  }
  public String getIntakeMode() {
      return intakeMode;
  }

  public void StopManipulator() {
    m_manipulatorMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
  }

  public void initSendable(SendableBuilder builder) {
      builder.addDoubleProperty("Motor Speed", this::getSpeed, this::setSpeed);
      builder.addStringProperty("Intake Mode", this::getIntakeMode, null);
  }

  @Override
  public void simulationPeriodic() {

  }
}
