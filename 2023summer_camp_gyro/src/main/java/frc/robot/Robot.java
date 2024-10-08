package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;


public class Robot extends TimedRobot {
  private Joystick m_stick = new Joystick(0);

  private final TalonFX lf = new TalonFX(1);
  private final TalonFX lb = new TalonFX(13);
  private final TalonFX rf = new TalonFX(0);
  private final TalonFX rb = new TalonFX(15);
  private final TalonFX hf = new TalonFX(2);
  private final TalonFX hb = new TalonFX(3);

  //limit switch
  private final DigitalInput toplimit = new DigitalInput(1);
  private final DigitalInput bottomlimit = new DigitalInput(0);
  private final TalonFX hand = new TalonFX(14);

  //PID&gyro
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
  double kp = 0.021;
  double ki = 0.08;
  double kd = 0.0015;

  double tp = 0.03;

  double last = 0, lasterror = 0, iLimit = 0.2, errorate = 0, errorsum = 0, anglerate = 0;
  

  @Override
  public void robotInit() {}

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    double error_y = m_gyro.getAngle();
    double error_x = 90 - error_y;
    double current = Timer.getFPGATimestamp();

    boolean right = true;
    boolean left = true;

    if (Math.abs(m_stick.getRawAxis(5)) > 0.08 && Math.abs(m_stick.getRawAxis(4)) < 0.08) {
      right = false;
      left = false;
      //double output_y = kp * error_y + ki * errorsum + kd * errorate;
      //double output_x = tp * error_x;
      double output_y = kp * error_y;

      lf.set(ControlMode.PercentOutput, -m_stick.getRawAxis(5) * 0.1 - output_y);
      rf.set(ControlMode.PercentOutput, m_stick.getRawAxis(5) * 0.1 - output_y);
      lb.set(ControlMode.PercentOutput, -m_stick.getRawAxis(5) * 0.1 - output_y);
      rf.set(ControlMode.PercentOutput, m_stick.getRawAxis(5) * 0.1 - output_y);
      hf.set(ControlMode.PercentOutput, 0);
      hb.set(ControlMode.PercentOutput, 0);
    }

    if (Math.abs(m_stick.getRawAxis(5)) < 0.08 && Math.abs(m_stick.getRawAxis(4)) > 0.08) {
      right = false;
      left = false;

      hf.set(ControlMode.PercentOutput, m_stick.getRawAxis(4) * 0.4);
      hb.set(ControlMode.PercentOutput, -m_stick.getRawAxis(4) * 0.4);
      lf.set(ControlMode.PercentOutput, 0);
      lb.set(ControlMode.PercentOutput, 0);
      rf.set(ControlMode.PercentOutput, 0);
      rb.set(ControlMode.PercentOutput, 0);
    }

    if (Math.abs(m_stick.getRawAxis(5)) > 0.08 && Math.abs(m_stick.getRawAxis(4)) > 0.08) {
      right = false;
      left = false;

        double output_y = kp * error_y + ki * errorsum + kd * errorate;
        double output_x = tp * error_x;
      hf.set(ControlMode.PercentOutput, m_stick.getRawAxis(4) * 0.4);
      hb.set(ControlMode.PercentOutput, -m_stick.getRawAxis(4) * 0.4);
      lf.set(ControlMode.PercentOutput, -m_stick.getRawAxis(5) * 0.1 - output_y);
      lb.set(ControlMode.PercentOutput, -m_stick.getRawAxis(5) * 0.1 - output_y);
      rf.set(ControlMode.PercentOutput, m_stick.getRawAxis(5) * 0.1 - output_y);
      rb.set(ControlMode.PercentOutput, m_stick.getRawAxis(5) * 0.1 - output_y);

      last = current;
      lasterror = error_y;
    }

    if (Math.abs(m_stick.getRawAxis(5)) < 0.05 && Math.abs(m_stick.getRawAxis(4)) < 0.05 && right == true && left == true && m_stick.getRawButton(2) == false && m_stick.getRawButton(3) == false) {
      hf.set(ControlMode.PercentOutput, 0);
      hb.set(ControlMode.PercentOutput, 0);
      lf.set(ControlMode.PercentOutput, 0);
      lb.set(ControlMode.PercentOutput, 0);
      rf.set(ControlMode.PercentOutput, 0);
      rb.set(ControlMode.PercentOutput, 0);
      m_gyro.reset();
    }

    if (m_stick.getRawButton(2)) {
      hf.set(ControlMode.PercentOutput, 0);
      hb.set(ControlMode.PercentOutput, 0);
      lf.set(ControlMode.PercentOutput, 0.1);
      lb.set(ControlMode.PercentOutput, 0.1);
      rf.set(ControlMode.PercentOutput, 0.1);
      rb.set(ControlMode.PercentOutput, 0.1);

      right = true;
      left = false;
    }

    if (m_stick.getRawButton(3)) {
      hf.set(ControlMode.PercentOutput, 0);
      hb.set(ControlMode.PercentOutput, 0);
      lf.set(ControlMode.PercentOutput, -0.1);
      lb.set(ControlMode.PercentOutput, -0.1);
      rf.set(ControlMode.PercentOutput, -0.1);
      rb.set(ControlMode.PercentOutput, -0.1);

      right = false;
      left = true;
    }

    SmartDashboard.putNumber("Axis5", m_stick.getRawAxis(5));
    SmartDashboard.putNumber("Axis4", m_stick.getRawAxis(4));
    SmartDashboard.putNumber("stright speed", m_stick.getRawAxis(5) * 0.4);
    SmartDashboard.putNumber("turn speed", m_stick.getRawAxis(4) * 0.4);
    SmartDashboard.putNumber("error_y", error_y);
    SmartDashboard.putNumber("error_x", error_x);

    setMotorSpeed(m_stick.getRawAxis(1) * -0.3);
  }
  
  public void setMotorSpeed(double speed) {
    if (speed < 0) {
      if (toplimit.get()) {
        hand.set(ControlMode.PercentOutput, speed);
      } else {
        hand.set(ControlMode.PercentOutput, speed);
      }
    } else {
      if (bottomlimit.get()) {
        hand.set(ControlMode.PercentOutput, speed);
      } else {
        hand.set(ControlMode.PercentOutput, speed);
      }
    }
  }
 
  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}