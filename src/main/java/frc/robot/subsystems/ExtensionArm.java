// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.RobotContainer;

public class ExtensionArm extends SubsystemBase {
  private final WPI_TalonFX m_leftMotor;
  private final WPI_TalonFX m_rightMotor;

  private final MotorControllerGroup m_extensionMotors;

  private final DigitalInput m_limitSwitch;


  /** Creates a new ExampleSubsystem. */
  public ExtensionArm() {
    m_leftMotor = new WPI_TalonFX(Constants.CAN.leftMotor);
    m_rightMotor = new WPI_TalonFX(Constants.CAN.rightMotor);

    m_limitSwitch = new DigitalInput(CAN.limitSwitch);

    m_rightMotor.configFactoryDefault();
    m_leftMotor.configFactoryDefault();

    m_leftMotor.configVoltageCompSaturation(Constants.Extension.maxVoltage);
    m_rightMotor.configVoltageCompSaturation(Constants.Extension.maxVoltage);

    m_leftMotor.enableVoltageCompensation(true);
    m_rightMotor.enableVoltageCompensation(true);

    m_rightMotor.setInverted(false);
    m_leftMotor.setInverted(true);

    m_extensionMotors = new MotorControllerGroup(m_leftMotor, m_rightMotor);

    m_leftMotor.setNeutralMode(NeutralMode.Brake);
    m_rightMotor.setNeutralMode(NeutralMode.Brake);

    m_leftMotor.follow(m_rightMotor);
  }

  public boolean LimitSwitch(){
    return m_limitSwitch.get();
  }

  public double getPosition(){
    return m_leftMotor.getSelectedSensorPosition();
  }

  public void moveExtensionArm(double speed){
    m_rightMotor.set(ControlMode.PercentOutput, speed);
}
  public void stopExtensionArm(){
    m_rightMotor.set(ControlMode.PercentOutput, 0);
  }
  public void resetEncoders(){
    m_rightMotor.setSelectedSensorPosition(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
