// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final TalonFX leftFlywheel9; // we should probably change these names once we learn more
  private final TalonFX rightFlywheel8; // we should probably change these names once we learn more
  private double speed8 = 0.85;
  private double speed9 = 0.85;


  final VoltageOut leftFlywheelVoltReq = new VoltageOut(0);
  final VoltageOut rightFlywheelVoltReq = new VoltageOut(0);
  final double leftFlywheelVolt = 12 * .85;
  final double rightFlywheelVolt = 12 * .85;


  Slot0Configs slot0Configs; // gains for specific slot

  // Note: phoenix 6 velocity is rotations / second
  final VelocityVoltage velocity = new VelocityVoltage(0); // class instance

  /*
  alternative way
  / / create a velocity closed-loop request, voltage output, slot 0 configs
  final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
   */
   */

  public Shooter() {
  leftFlywheel9 = new TalonFX(9);
  rightFlywheel8 = new TalonFX(8);

  leftFlywheel9.getConfigurator().apply(new TalonFXConfiguration()); // config factory default
  rightFlywheel8.getConfigurator().apply(new TalonFXConfiguration()); // config factory default
  leftFlywheel9.setNeutralMode(NeutralModeValue.Coast);
  rightFlywheel8.setNeutralMode(NeutralModeValue.Coast);

  rightFlywheel8.setInverted(true);

  slot0Configs = new Slot0Configs(); // gains for specific slot
  slot0Configs.kS = 0.05; // Add 0.05 V output to overcome static friction
  slot0Configs.kV = 0.12; // 1/(rps) - A velocity target of 1 rps results in 0.12 V output
  slot0Configs.kP = 0.11; // 1/rps - An error of 1 rps results in 0.11 V output
  slot0Configs.kI = 0; // 1/rot - output per unit of integrated error in velocity (output/rotation)
  slot0Configs.kD = 0; // output per unit of error derivative in velocity (output/ (rps/s))
  leftFlywheel9.getConfigurator().apply(new Slot0Configs());
  }

  public void setFlywheelVelocity() {
    velocity.Slot = 0;
    leftFlywheel9.setControl(velocity.withVelocity(8));

    // ALT way: set velocity to 8 rps, add 0.5 V to overcome gravity
    // m_talonFX.setControl(velocityRequest.withVelocity(8).withFeedForward(0.5));
  }

  public double getLeftFlyVelocity() {
     return leftFlywheel9.getVelocity().getValue(); // rotations per second
  }

  public double getRightFlyVelocity() {
    return rightFlywheel8.getVelocity().getValue(); // rotations per second
  }

  public double getRightFlyVoltSupply() {
    return rightFlywheel8.getSupplyVoltage().getValue();
  }

  public double getLeftFlywheelVoltSupply() {
    return leftFlywheel9.getSupplyVoltage().getValue();
  }

  public double getRightFlyVoltSupply() {
    return rightFlywheel8.getSupplyVoltage().getValue();
  }

  public void runMotors() {
    leftFlywheel9.setControl(leftFlywheelVoltReq.withOutput(leftFlywheelVolt));
    rightFlywheel8.setControl(rightFlywheelVoltReq.withOutput(rightFlywheelVolt));
  }

  public void stopShooter() {
    leftFlywheel9.setControl(leftFlywheelVoltReq.withOutput(0));
    rightFlywheel8.setControl(rightFlywheelVoltReq.withOutput(0));
  }

  @Override
  public void periodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");

    builder.addDoubleProperty("velocity left", this::getLeftFlyVelocity, null);
    builder.addDoubleProperty("velocity right", this::getRightFlyVelocity, null);

    builder.addDoubleProperty("voltage supply left", this::getLeftFlywheelVoltSupply, null);
    builder.addDoubleProperty("voltage supply right", this::getRightFlyVoltSupply, null);
  }

  public double getSpeed8() {
    return speed8;
  }
  public double getSpeed9() {
    return speed9;
  }
  public void setSpeed8(double x) {
    speed8 = x;
  }
  public void setSpeed9(double x){
    speed9 =x;
  }

  @Override
  public void simulationPeriodic() {

  }
}
