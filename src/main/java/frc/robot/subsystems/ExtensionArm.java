// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CAN;
import frc.robot.RobotContainer;

public class ExtensionArm extends SubsystemBase {
  private final WPI_TalonSRX m_leftMotor;
  private final WPI_TalonSRX m_rightMotor;

  private final MotorControllerGroup m_extensionMotors;

  private final DigitalInput m_limitSwitch;
  private boolean isZeroed = false;
/**Extension arm testing constants*/
  public static double maxExtensionTicks = 100; // TODO
  public static double kExtensionDeadband = 0.05; //The % of max extension where it will slow down (works on both ends)
  public static double percentage = 15;
  public static double slowExtensionEndsDistance = (percentage / 100) * maxExtensionTicks; // TODO // the distance from the ends of the arm required to start slowing the motor down
  public static double extensionTicksToArmDistance = 0; // TODO // conversion factor from ticks to distance of arm extension
  public static double extensionFactorScalar = slowExtensionEndsDistance; // TODO
  public static double dumbSpeed = .15;

  public ExtensionArm() {
    m_leftMotor = new WPI_TalonSRX(Constants.CAN.leftMotor);
    m_rightMotor = new WPI_TalonSRX(Constants.CAN.rightMotor);

    m_leftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    m_rightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);

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

  /**returns if the limit switch has been hit*/
  public boolean LimitSwitch(){
    return m_limitSwitch.get();
  }

  public double getPosition(){
    return m_leftMotor.getSelectedSensorPosition() * extensionTicksToArmDistance;
  }

  public double getDumbSpeed(){
    return dumbSpeed;
  }

  public void setDumbSpeed(double x){
    dumbSpeed = x;
  }

  public void runDumbExtend(){
    m_rightMotor.set(ControlMode.PercentOutput, dumbSpeed);
    m_leftMotor.set(ControlMode.PercentOutput, dumbSpeed);
  }

  public void runDumbRetract(){
    m_rightMotor.set(ControlMode.PercentOutput, -dumbSpeed);
    m_leftMotor.set(ControlMode.PercentOutput, -dumbSpeed);
  }

  public void runMotor(double speed){
    m_leftMotor.set(ControlMode.PercentOutput, speed);
    m_rightMotor.set(ControlMode.PercentOutput, speed);
  }

  public void stop(){
    m_rightMotor.set(ControlMode.PercentOutput,0);
    m_leftMotor.set(ControlMode.PercentOutput,0);
  }

  /**zeroes encoders*/
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

  /**returns if the arm has retracted or extended into the danger(slow) zones*/
  public boolean inSlowZone() {
    if((getPosition()<=slowExtensionEndsDistance)&&(getPosition()>=maxExtensionTicks-slowExtensionEndsDistance)){
      return true;
    }
    else {
      return false;
    }
  }

  public double slowZoneFactor() {
    double factor = 1;
    double maxDistance = maxExtensionTicks;
    double distance = getPosition();
    double slowZoneDistance = slowExtensionEndsDistance;
    double joystickInput = RobotContainer.m_WeaponsGamepad.getRawAxis(1);

    if (distance <= slowZoneDistance) {
      if (joystickInput <= 0){ // if the joystick extends the arm then
        factor = (distance / extensionFactorScalar); // if retracting while in 1st slow zone, slow down arm
      }
    } // if extending while in 1st slow zone, put no limits on speed
    else if (distance >= (maxDistance - slowZoneDistance)) {
      if (joystickInput >= 0) {
        factor = ((maxDistance - distance) / extensionFactorScalar); // if extending while in 2nd slow zone, slow down arm
      }
    } // if retracting while in 2nd slow zone, put no limits on speed
    return factor;
  }

  public double rawMotorSpeed(double y) {
    if (kExtensionDeadband >= Math.abs(y)) { // if the fetched joystick value is less than the deadband value, then set speed to 0
      return 0;
    }
    if (!allowedToMovePastEnds(y)) { // if arm hits the ends and is trying to move past the ends, then set speed to zero
      return 0;
    }
    return y;
  }

  public boolean allowedToMovePastEnds(double y) {
    if (LimitSwitch() && (y <= 0)) { // if trying to move past the limit switch, return false
      return false;
    } else if ((getPosition() >= maxExtensionTicks) && (y >= 0)) { // if trying to move past maxTicks, return false
      return false;
    }
    return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**Shuffleboard getters and setters for constants*/
  /**these will be used in Extension Arm Commands until constant testing is over*/
  public double getMaxExtensionTicks(){
    return maxExtensionTicks;
  }
  public void setMaxExtensionTicks(double x){
    maxExtensionTicks = x;
  }

  public double getkExtensionDeadband(){
    return kExtensionDeadband;
  }

  public void setkExtensionDeadband(double x){
    kExtensionDeadband = x;
  }

  public double getSlowExtensionEndsDistance(){
    return slowExtensionEndsDistance;
  }

  public void setSlowExtensionEndsDistance(double x){
    slowExtensionEndsDistance = x;
  }

  public double getExtensionTicksToArmDistance(){
    return extensionTicksToArmDistance;
  }

  public void setExtensionTicksToArmDistance(double x){
    extensionTicksToArmDistance = x;
  }

  public double getExtensionFactorScalar(){
    return extensionFactorScalar;
  }

  public void setExtensionFactorScalar(double x){
    extensionFactorScalar = x;
  }

  public double getPercentage() {
    return percentage;
  }


  @Override
  public void initSendable(SendableBuilder builder){
    builder.addDoubleProperty("Position", this::getPosition, null);
    builder.addDoubleProperty("Dumb Speed", this::getDumbSpeed, this::setDumbSpeed);
    builder.addBooleanProperty("Hit Limit Switch", this::LimitSwitch, null);
    builder.addDoubleProperty("CMax Extension Ticks", this::getMaxExtensionTicks, this::setMaxExtensionTicks);
    builder.addDoubleProperty("CExtension Deadband", this::getkExtensionDeadband, this::setkExtensionDeadband);
    builder.addDoubleProperty("CSlow Zone Extension Percentage", this::getPercentage, this::setSlowExtensionEndsDistance);
    builder.addDoubleProperty("CExtension Ticks to Arm Distance Conversion Factor", this::getExtensionTicksToArmDistance, this::setExtensionTicksToArmDistance); // idk why we have this here - cant we just use formulas to find this?
    builder.addDoubleProperty("CExtension Factor Scalar", this::getExtensionFactorScalar, this::setExtensionFactorScalar);
    //titles with "C" in front are constants that need to be determined through experimentation
  }
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
