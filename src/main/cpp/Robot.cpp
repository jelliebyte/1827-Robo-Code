// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

//2550 is the max range it can go

class Robot : public frc::TimedRobot {
 public:
  Robot() {
    wpi::SendableRegistry::AddChild(&m_robotDrive, &rightside_leader);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &leftside_leader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    leftside_follower.Follow(leftside_leader);
    rightside_follower.Follow(rightside_leader);
    rightside_leader.SetInverted(true);
    armMotor2.Follow(armMotor1);
    m_robotDrive.SetExpiration(100_ms);
    m_timer.Start();
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    m_chooser.AddOption(kAutoNameCustom2, kAutoNameCustom2);
    frc::SmartDashboard::PutData("Auto modes", &m_chooser);
    
    // frc::CameraServer::StartAutomaticCapture();
  }

class armClass {
  public:
    int position;
    int speed;
    void moveArm(){
      return;
    }
};

class Shooter: public armClass {
  int x = 0;
};

class Intake: public armClass{
  int x = 0;
};  

 double calculateError(int desired){
  double error = desired - m_encoder.GetDistance();
  double motorOutput = p * error;
  return motorOutput;
}

  void AutonomousInit() override { 
    m_timer.Restart();
    m_autoSelected = m_chooser.GetSelected();
    fmt::print("Auto selected: {}\n", m_autoSelected);
    }

  void AutonomousPeriodic() override {
    //custom auto
    if (m_autoSelected == kAutoNameCustom) {
    std::cout << "The custom auto was selected!\n";
  } 
  else if (m_autoSelected == kAutoNameCustom2){
    std::cout << "The 3rd auto was selected!\n";
  }
  else { //default auto
        if (m_timer.Get() < 1_s) {
      m_robotDrive.ArcadeDrive(0.5, 0.0
      , false);
      //  std::cout << encoderDistance << "\n";
    } else {
      // Stop robot
      m_robotDrive.StopMotor();
      }
    }
  }

  void TeleopInit() override {
    m_encoder.Reset();
  }

  void TeleopPeriodic() override {
    m_robotDrive.ArcadeDrive(m_driveController.GetLeftX() * 0.5, m_driveController.GetLeftY() * 0.5);
  }

  void TestInit() override {
    m_encoder.Reset();
  }

  void TestPeriodic() override { // move all to teleop eventually

  double encoderRate = m_encoder.GetRate();
  double distancePerPulse = m_encoder.GetDistancePerPulse();
  double encoderCount = m_encoder.Get();
  double encoderDistance = m_encoder.GetDistance();
  double ArcadeMoveA;
  double ArcadeMoveB;

  frc::SmartDashboard::PutNumber("Encoder distance: ", encoderDistance);
  frc::SmartDashboard::PutNumber("Encoder Distance Per Pulse: ", distancePerPulse);
  frc::SmartDashboard::PutNumber("Encoder count: ", encoderCount);
  frc::SmartDashboard::PutNumber("Encoder rate: ", encoderRate);


  // m_robotDrive.ArcadeDrive(calculateError(desired), 0.0, false);

  m_robotDrive.ArcadeDrive(m_driveController.GetLeftX()*0.5, m_driveController.GetRightY()*0.5, false);

  m_armDrive.ArcadeDrive(m_armController.GetLeftY(), 0.0, false);

  // if (m_controller.GetYButton()){
  //   desired = 2190; 
  // }
  // if (m_controller.GetXButton()){
  //   desired = 4000;
  // }
    
  }

 private:
  // Robot drive system
  WPI_TalonSRX rightside_leader{1};
  WPI_VictorSPX rightside_follower{2};
  WPI_VictorSPX leftside_leader{3};
  WPI_VictorSPX leftside_follower{4};
  WPI_VictorSPX armMotor1{5};
  WPI_TalonSRX armMotor2{6};
  WPI_VictorSPX motor_7{7};
 // WPI_VictorSPX motor_8{8}; Motor 8 was clicking
//WPI_VictorSPX motor_9{9};


  frc::DifferentialDrive m_robotDrive{
      [&](double output) { rightside_leader.Set(output); },
      [&](double output) { leftside_leader.Set(output); }};


  frc::DifferentialDrive m_armDrive{
      [&](double output) { armMotor1.Set(output); },
      [&](double output) { armMotor2.Set(output); }};

  frc::XboxController m_driveController{0};
  frc::XboxController m_armController{1};
  frc::Timer m_timer;
  frc::DigitalInput m_limitswitch{2};
  frc::Encoder m_encoder{0,1};

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  const std::string kAutoNameCustom2 = "My Auto 2";
  std::string m_autoSelected;

  double position = 0;
  int desired = 0;
  double p = 0.03; //wobbly, fix later
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif