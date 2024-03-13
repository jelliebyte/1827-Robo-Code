// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// imported libraries
#include <iostream>
#include <frc/TimedRobot.h>
#include <frc/Timer.h>
#include <frc/XboxController.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <ctre/Phoenix.h>
#include <cameraserver/CameraServer.h>
#include <cscore_oo.h>
#include <frc/DigitalInput.h>
#include <frc/Encoder.h>
#include <string>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>

// 800 max range

class Robot : public frc::TimedRobot
{
public:
  Robot()
  {
    wpi::SendableRegistry::AddChild(&m_robotDrive, &rightside_leader);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &leftside_leader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    leftside_follower.Follow(leftside_leader); // follow the leader
    rightside_follower.Follow(rightside_leader);
    rightside_leader.SetInverted(true);
    intake_motor.SetInverted(true);

    shooter_follower.Follow(shooter_leader);

    arm_follower.Follow(arm_leader);
    arm_leader.SetInverted(true);
    arm_leader.SetNeutralMode(Brake);
    arm_follower.SetNeutralMode(Brake);

    m_robotDrive.SetExpiration(100_ms);
    m_timer.Start(); // autonomous timer
    m_shoot_timer.Start();
    m_chooser.SetDefaultOption(kAutoLeftPos, kAutoLeftPos);
    m_chooser.AddOption(kAutoMidPos, kAutoMidPos);          // custom autonomous mode 1
    m_chooser.AddOption(kAutoRightPos, kAutoRightPos);      // custom autonomous mode 2
    frc::SmartDashboard::PutData("Auto modes", &m_chooser); // pushes autonomous modes to the smartdashboard as selectable options
    frc::CameraServer::StartAutomaticCapture();
    m_encoder.Reset();
  }

  void RobotPeriodic() override
  {
    double encoderRate = m_encoder.GetRate();
    double distancePerPulse = m_encoder.GetDistancePerPulse();
    double encoderCount = m_encoder.Get();
    double encoderDistance = m_encoder.GetDistance();

    frc::SmartDashboard::PutNumber("Encoder distance: ", encoderDistance);
    frc::SmartDashboard::PutNumber("Encoder Distance Per Pulse: ", distancePerPulse);
    frc::SmartDashboard::PutNumber("Encoder count: ", encoderCount);
    frc::SmartDashboard::PutNumber("Encoder rate: ", encoderRate);
  }

  double calculateError(int desired)
  {                                                   // gradual slowdown
    double error = desired - m_encoder.GetDistance(); // error is the margin between the current position and the desired position
    double motorOutput = p * error;
    if (motorOutput < -1 * throttle)
    {
      motorOutput = -1 * throttle;
    }
    else if (motorOutput > throttle)
    {
      motorOutput = throttle;
    }
    return motorOutput;
  }

  void intakeSystem()
  {
    if (m_armController.GetLeftBumper())
    {
      intake_motor.Set(1);
    }
    else if (m_armController.GetRightBumper())
    {
      intake_motor.Set(-1);
    }
    else
    {
      intake_motor.Set(0);
    }
  }

  void armMovement()
  {

    if (m_armController.GetYButton())
    {
      desired = -962;
    }
    else if (m_armController.GetXButton())
    {
      desired = -25; // change
    }
    else if (m_armController.GetAButton())
    {
      desired = -160;
    }
    else if (m_armController.GetBButton())
    {
      desired = -760;
    }
  }

  void speakerShoot()
  {
    if (isShooting != true)
    {
      m_shoot_timer.Restart();
      isShooting = true;
    }

    if (m_shoot_timer.Get() < 0.2_s)
    {
      intake_motor.Set(-0.2);
    }
    else if (m_shoot_timer.Get() < 1_s)
    {
      intake_motor.StopMotor();
      shooter_leader.Set(1);
    }
    else if (m_shoot_timer.Get() < 3_s)
    {
      intake_motor.Set(1);
    }
    else
    {
      intake_motor.StopMotor();
      shooter_leader.StopMotor();
    }
    // new timer for shooting, reverse intake (0.2), run shooter, run intake
  }

  void AutonomousInit() override
  {
    m_timer.Restart();                        // restarts time for autonomous modes
    m_autoSelected = m_chooser.GetSelected(); // sets autonomous modes
    std::cout << "Auto selected: " << m_autoSelected << "\n";
    isShooting = false;
  }

  void AutonomousPeriodic() override
  {
    // middle field position
    if (m_autoSelected == kAutoMidPos)
    {
      if (m_timer.Get() < 1_s) //sets arm down
      {
        desired = -160;
      }
      else if (m_timer.Get() < 4_s)
      {
        speakerShoot(); //shoots
      }
      else if (m_timer.Get() < 7_s)
      {
        intake_motor.Set(1);
        shooter_leader.StopMotor();
        m_robotDrive.ArcadeDrive(-0.3, 0.0, false); //forward & pick-up
        desired = -25;
        isShooting = false;
      }
      else if (m_timer.Get() < 9_s)
      {
        desired = -160;
        intake_motor.StopMotor();
        m_robotDrive.ArcadeDrive(0.3, 0.0, false); //drive backward
      }
      else if (m_timer.Get() < 12_s)
      {
        m_robotDrive.ArcadeDrive(0.0, 0.0, false); //shoots
        speakerShoot();
      }
      else {
        isShooting = false;
        shooter_leader.StopMotor();
        intake_motor.StopMotor();
      }

    }
    // right field position
    else if (m_autoSelected == kAutoRightPos)
    {
      if (m_timer.Get() < 3_s)
      {
        speakerShoot();
      }
    }
    // left field position
    else
    {
      std::cout << "Left position"
                << "\n";
    }
    arm_leader.Set(calculateError(desired));
  }

  void TeleopInit() override
  {
    desired = m_encoder.GetDistance();
  }

  void TeleopPeriodic() override
  {
    m_robotDrive.ArcadeDrive(m_driveController.GetLeftY(), m_driveController.GetRightX(), false);

    if (m_armController.GetRightTriggerAxis() > 0.9)
    {
      speakerShoot();
    }
    else if (m_armController.GetLeftTriggerAxis())
    {
      intake_motor.Set(1);
      shooter_leader.Set(0.4);
    }
    else
    {
      intakeSystem();
      shooter_leader.Set(-1 * m_armController.GetLeftY());
      isShooting = false;
    }
    armMovement();

    arm_leader.Set(calculateError(desired));
  }

  void TestInit() override
  {
    std::cout << "Reset!"
              << "\n";
    desired = m_encoder.GetDistance();
  }

  void TestPeriodic() override
  { // move all to teleop eventually
    m_robotDrive.ArcadeDrive(m_driveController.GetLeftY() / 2, m_driveController.GetRightX() / 2, false);

    if (m_armController.GetLeftTriggerAxis())
    {
      shooter_leader.Set(0.5);
    }
    if (m_armController.GetRightTriggerAxis())
    {
      shooter_leader.Set(1);
    }

    armMovement();
    intakeSystem();

    arm_leader.Set(calculateError(desired));
  }

private:
  // Robot drive system

  // motor controllers
  WPI_TalonSRX rightside_leader{1};
  WPI_VictorSPX rightside_follower{2};
  WPI_VictorSPX leftside_leader{3};
  WPI_VictorSPX leftside_follower{4};

  WPI_VictorSPX shooter_leader{5};  // shooter 1
  WPI_TalonSRX shooter_follower{6}; // shooter 2
  WPI_VictorSPX intake_motor{7};    // grab

  WPI_VictorSPX arm_follower{8};
  WPI_VictorSPX arm_leader{9};

  // drive modes
  frc::DifferentialDrive m_robotDrive{
      [&](double output)
      { rightside_leader.Set(output); },
      [&](double output)
      { leftside_leader.Set(output); }};

  frc::XboxController m_driveController{0};
  frc::XboxController m_armController{1};
  frc::Timer m_timer; // creation of autonomous timer
  frc::Timer m_shoot_timer;
  frc::DigitalInput m_limitswitch{2}; // limit switch uses pin 2
  frc::Encoder m_encoder{0, 1};       // encoder uses pins 0-1

  frc::SendableChooser<std::string> m_chooser; // autonomous modes
  const std::string kAutoLeftPos = "Position_Left";
  const std::string kAutoMidPos = "Position_Mid";
  const std::string kAutoRightPos = "Position_Right";
  std::string m_autoSelected;

  double position = 0;
  int desired = 0; // desired is the desired distance we want the motor to reach
  double p = 0.005;
  double throttle = 0.5;

  bool isShooting = false;

  // Intake intakeVar;
  // Shooter shooterVar;
  // armClass armVar;
};

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif