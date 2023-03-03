/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

// import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;


/**
 * Add your docs here.
 */
public class DriveSubsystem extends SubsystemBase {

  double kp = 0.1, ki = 0, kd = 0;

  float pidTolerance = 0.1f;


  PIDController headingPIDControl = new PIDController(kp, ki, kd);


  AHRS ahrs = new AHRS(SPI.Port.kMXP);
  
  TalonSRX Left1 = new TalonSRX(RobotMap.Left1);
  TalonSRX Left2 = new TalonSRX(RobotMap.Left2);
  TalonSRX Right1 = new TalonSRX(RobotMap.Right1);
  TalonSRX Right2 = new TalonSRX(RobotMap.Right2);

  public DriveSubsystem(){
    headingPIDControl.enableContinuousInput(-180, 180);
    headingPIDControl.setTolerance(pidTolerance);
  }
  

  public void TankDrive (double left, double right) {

    double yaw = ahrs.getAngle();
    double pitch = ahrs.getPitch();
    double roll = ahrs.getRoll(); 

    double output = headingPIDControl.calculate(yaw, 0);

    Left1.set(ControlMode.PercentOutput, left - output);
    Left2.set(ControlMode.PercentOutput, left - output);
    Right1.set(ControlMode.PercentOutput, -(right + output));
    Right2.set(ControlMode.PercentOutput, -(right + output));


    double absAngle = ahrs.getAngle();

    

    SmartDashboard.putNumber("pitch", pitch);
    SmartDashboard.putNumber("roll", roll);
    SmartDashboard.putNumber("yaw", yaw);
    SmartDashboard.putNumber("PID", output);

    SmartDashboard.putNumber("Angle", absAngle);

    System.out.println("pitch: " + pitch + "  roll: " + roll + "  yaw: " + yaw + " absAngle: " + absAngle + " output: " + output);
    
  }

  public void ArcadeDrive (double speed, double turn) {
    TankDrive((speed - turn) * 0.5, (speed + turn) * 0.5);
  }
}

  // static final double kOffBalanceAngleThresholdDegrees = 10;
  // static final double kOonBalanceAngleThresholdDegrees = 5;

    
  // public void AutoBalance() {

  //   ahrs.reset();
     
  //   if (!autoBalanceXMode && (Math.abs(yawAngleDegrees) >= Math.abs(kOffBalanceAngleThresholdDegrees))) {
  //       autoBalanceXMode = true;
  //   } else if (autoBalanceXMode && (Math.abs(yawAngleDegrees) <= Math.abs(kOonBalanceAngleThresholdDegrees))) {
  //       autoBalanceXMode = false;
  //   }
  //   // if (!autoBalanceYMode && (Math.abs(pitchAngleDegrees) >= Math.abs(kOffBalanceAngleThresholdDegrees))) {
  //   //     autoBalanceYMode = true;
  //   // } else if (autoBalanceYMode && (Math.abs(pitchAngleDegrees) <= Math.abs(kOonBalanceAngleThresholdDegrees))) {
  //   //     autoBalanceYMode = false;
  //   // }



  //   // Control drive system automatically,
  //   // driving in reverse direction of pitch/roll angle,
  //   // with a magnitude based upon the angle

  //   if (autoBalanceXMode) {
  //       double yawAngleRadians = yawAngleDegrees * (Math.PI / 180.0);
  //       xSpeed = Math.sin(yawAngleRadians) * -0.5;
  //   }
  //   // if (autoBalanceYMode) {
  //   //     double rollAngleRadians = rollAngleDegrees * (Math.PI / 180.0);
  //   //     ySpeed = Math.sin(rollAngleRadians) * -1;
  //   // }

  //   try {
  //     Left1.set(ControlMode.PercentOutput, xSpeed);
  //     Left2.set(ControlMode.PercentOutput, xSpeed);
  //     Right1.set(ControlMode.PercentOutput, xSpeed);
  //     Right2.set(ControlMode.PercentOutput, xSpeed);
  //   } catch (RuntimeException ex) {
  //       String err_string = "Drive system error:  " + ex.getMessage();
  //       DriverStation.reportError(err_string, true);
  //   }

  //   System.out.println("xSpeed: " + xSpeed + " yaw: " + yawAngleDegrees);
  // }
