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
  private boolean isZeroed = false;

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


  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  public boolean LimitSwitch(){
    return m_limitSwitch.get();
  }

  public double getPosition(){
    return m_leftMotor.getSelectedSensorPosition();
  }
  public double getDistance(){
    return getPosition() * Constants.Extension.extensionTicksToArmDistance;
  }
  public double getSpeed(){
    int t = 0; // TO-DO
    return t;
  }

  public void Extension(double speed){
    m_rightMotor.set(ControlMode.PercentOutput, speed);
    m_leftMotor.set(ControlMode.PercentOutput, speed);
}
  public void stop(){
    m_rightMotor.set(ControlMode.PercentOutput,0);
    m_leftMotor.set(ControlMode.PercentOutput,0);
  }
  public void resetEncoders(){
    m_leftMotor.setSelectedSensorPosition(0);
    m_rightMotor.setSelectedSensorPosition(0);
  }
  public void setZeroed(){
    isZeroed = true;
  }
  public boolean isZeroed(){
    return isZeroed;
  }
  public boolean inSlowZone(){
    if((getPosition()<=Constants.Extension.slowExtensionEndsDistance)&&(Constants.Extension.maxExtensionTicks-getPosition()<=Constants.Extension.slowExtensionEndsDistance)){
      return false;
    }
    else {
      return true;
    }
  }
  public double slowZoneFactor(){
    /** try to downsize some conditions to seperate methods -Giorgia*/
    double factor = 1;
    double minDistance = Constants.Extension.slowExtensionEndsDistance;
    double maxDistance = Constants.Extension.maxExtensionTicks;
    double distance = getPosition();
    if (distance > minDistance && distance < (maxDistance - minDistance)) {
      return factor;
    }
    else if (distance/maxDistance < .05){
      if (RobotContainer.m_WeaponsGamepad.getRawAxis(1) <= 0){
        factor = (getPosition()/Constants.Extension.extensionFactorScalar);
      }
      else if (RobotContainer.m_WeaponsGamepad.getRawAxis(1) > 0) {}
      }
    else if (distance/maxDistance > .95) {
      if (RobotContainer.m_WeaponsGamepad.getRawAxis(1) >= 0){
        factor = ((maxDistance-getPosition())/Constants.Extension.extensionFactorScalar);
      }
      else if (RobotContainer.m_WeaponsGamepad.getRawAxis(1) < 0) {}
    }
    return factor;
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
