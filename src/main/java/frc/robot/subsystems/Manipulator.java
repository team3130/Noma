// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {

    private double motorSpeed;
    private final WPI_TalonFX m_manipulatorMotor; // we should probably change these names once we learn more


  public Manipulator() {
    m_manipulatorMotor = new WPI_TalonFX(Constants.Manipulator.CAN_manipulatorMotor);
    m_manipulatorMotor.configFactoryDefault();
    m_manipulatorMotor.setInverted(false);
    motorSpeed = 0.0;
  }

  public void setSpeed(double x) {
    motorSpeed = x;
    m_manipulatorMotor.set(x);
  }
  public double getSpeed() {
    return motorSpeed;
  }

  @Override
  public void periodic() {}

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty("speed", this::getSpeed, this::setSpeed);
  }

  @Override
  public void simulationPeriodic() {

  }
}
