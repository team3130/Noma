// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private final WPI_TalonFX motor8; // we should probably change these names once we learn more
    private final WPI_TalonFX motor9; // we should probably change these names once we learn more

  private double speed8 = 0.85;
    private double speed9 = 0.85;


    public Shooter() {
    motor8 = new WPI_TalonFX(9);
    motor9 = new WPI_TalonFX(8);

    motor8.configFactoryDefault();
      motor9.configFactoryDefault();

      motor8.setNeutralMode(NeutralMode.Coast);
      motor9.setNeutralMode(NeutralMode.Coast);

      motor9.setInverted(true);
  }

  public void runMotor8() {
      motor8.set(ControlMode.PercentOutput, speed8);
  }

    public void runMotor9() {
        motor9.set(ControlMode.PercentOutput, speed9);
      motor8.set(ControlMode.PercentOutput, speed8);
    }

  public void runMotors() {
    motor9.set(ControlMode.PercentOutput, speed9);
    motor8.set(ControlMode.PercentOutput, speed8);

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

  public void StopShooter() {
    motor9.set(ControlMode.PercentOutput, 0);
    motor8.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");

    builder.addDoubleProperty("speed 8", this::getSpeed8, null);
    builder.addDoubleProperty("speed 9", this::getSpeed9, null);

  }

  @Override
  public void simulationPeriodic() {

  }
}
