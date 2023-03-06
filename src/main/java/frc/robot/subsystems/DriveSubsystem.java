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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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

  static boolean autoBalanceBoolean = false;


  
  /*
  TalonSRX Left1 = new TalonSRX(RobotMap.Left1);
  TalonSRX Left2 = new TalonSRX(RobotMap.Left2);
  TalonSRX Right1 = new TalonSRX(RobotMap.Right1);
  TalonSRX Right2 = new TalonSRX(RobotMap.Right2); 
  */

  CANSparkMax Left1 = new CANSparkMax(RobotMap.Left1, MotorType.kBrushless);
  CANSparkMax Left2 = new CANSparkMax(RobotMap.Left2, MotorType.kBrushless);
  CANSparkMax Right1 = new CANSparkMax(RobotMap.Right1, MotorType.kBrushless);
  CANSparkMax Right2 = new CANSparkMax(RobotMap.Right2, MotorType.kBrushless);
  
  public DriveSubsystem(){
    headingPIDControl.enableContinuousInput(-180, 180);
    headingPIDControl.setTolerance(pidTolerance);
  }
  
  public void TankDrive (double left, double right) {

    double yaw = ahrs.getAngle();
    double pitch = ahrs.getPitch();
    double roll = ahrs.getRoll(); 

    double output;


    if(autoBalanceBoolean) {
      output = headingPIDControl.calculate(yaw, 0);
    } else {
      output = 0;
    }

    Left1.set(-(left - output));
    Left2.set(-(left - output));
    Right1.set(right + output);
    Right2.set(right + output);


    double absAngle = ahrs.getAngle();

    

    SmartDashboard.putNumber("pitch", pitch);
    SmartDashboard.putNumber("roll", roll);
    SmartDashboard.putNumber("yaw", yaw);
    SmartDashboard.putNumber("PID", output);

    SmartDashboard.putNumber("Angle", absAngle);

    System.out.println("pitch: " + pitch + "  roll: " + roll + "  yaw: " + yaw + " absAngle: " + absAngle + " output: " + output + " is Auto Balance On?: " + autoBalanceBoolean);
    
    
  }

  public void ArcadeDrive (double speed, double turn) {
    TankDrive((speed - turn) * 0.5, (speed + turn) * 0.5);
  }

  public void toggleAutoBalance (boolean b) {
    autoBalanceBoolean = b;
  }
}
