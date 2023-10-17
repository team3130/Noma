// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {

  private final WPI_TalonFX m_manipulatorMotor; // we should probably change these names once we learn more


  public Manipulator() {
    m_manipulatorMotor = new WPI_TalonFX(Constants.CAN.manipulatorMotor);
    m_manipulatorMotor.configFactoryDefault();
    m_manipulatorMotor.setInverted(false);
  }

  public void Manipulate (double speed){
      /** "speed" is perf to make an actual variable (rather than a parameter) and export in shuffleboard -Giorgia*/

      m_manipulatorMotor.set(ControlMode.PercentOutput, speed);
     }
 public void StopManipulator (){
    m_manipulatorMotor.set(ControlMode.PercentOutput, 0); }
  @Override
  public void periodic() {

  }

  @Override
  public void simulationPeriodic() {

  }
}
