// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {
    private final WPI_TalonFX m_manipulatorMotor8; // we should probably change these names once we learn more
    private final WPI_TalonFX m_manipulatorMotor9; // we should probably change these names once we learn more
    private double outtakeCubeSpeed = 0.2;


  public Manipulator() {
    m_manipulatorMotor8 = new WPI_TalonFX(Constants.CAN.intakeMotor);
    m_manipulatorMotor9 = new WPI_TalonFX(Constants.CAN.intake);
    m_manipulatorMotor8.configFactoryDefault();
      m_manipulatorMotor9.configFactoryDefault();

    m_manipulatorMotor8.enableVoltageCompensation(true);
    m_manipulatorMotor8.configVoltageCompSaturation(10);

      m_manipulatorMotor9.enableVoltageCompensation(true);
      m_manipulatorMotor9.configVoltageCompSaturation(10);

    m_manipulatorMotor9.setInverted(true);
  }

  public void runMotor8() {
      m_manipulatorMotor8.set(ControlMode.PercentOutput, outtakeCubeSpeed);
  }

    public void runMotor9() {
        m_manipulatorMotor9.set(ControlMode.PercentOutput, outtakeCubeSpeed);
    }

  public void StopManipulator() {
    m_manipulatorMotor9.set(ControlMode.PercentOutput, 0);
    m_manipulatorMotor8.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void simulationPeriodic() {

  }
}
