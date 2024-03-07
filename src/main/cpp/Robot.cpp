// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//imported libraries
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

//800 max range

class Robot : public frc::TimedRobot {
 public:
  Robot() {
    wpi::SendableRegistry::AddChild(&m_robotDrive, &rightside_leader);
    wpi::SendableRegistry::AddChild(&m_robotDrive, &leftside_leader);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    leftside_follower.Follow(leftside_leader); //follow the leader
    rightside_follower.Follow(rightside_leader);
    rightside_leader.SetInverted(true);
    intake_motor_3.SetInverted(true);

    intake_motor_2.Follow(intake_motor_1); //follows intake motor
    intake_motor_1.SetInverted(true);
    intake_motor_2.SetInverted(true);

    arm_motor_1.Follow(arm_motor_2);
    arm_motor_2.SetInverted(true);
    arm_motor_2.SetNeutralMode(Brake);
    arm_motor_1.SetNeutralMode(Brake);

    m_robotDrive.SetExpiration(100_ms);
    m_timer.Start(); //autonomous timer
    m_chooser.SetDefaultOption(kAutoLeftPos, kAutoLeftPos);
    m_chooser.AddOption(kAutoMidPos, kAutoMidPos); //custom autonomous mode 1
    m_chooser.AddOption(kAutoRightPos, kAutoRightPos);// custom autonomous mode 2
    frc::SmartDashboard::PutData("Auto modes", &m_chooser); //pushes autonomous modes to the smartdashboard as selectable options
    
    // frc::CameraServer::StartAutomaticCapture();
  }

  double calculateError(int desired){ //gradual slowdown
  double error = desired - m_encoder.GetDistance(); //error is the margin between the current position and the desired position
  double motorOutput = p * error;
  if (motorOutput < -1 * throttle){
    motorOutput = -1 * throttle;
  }
  else if (motorOutput > throttle){
    motorOutput = throttle;
  }
  return motorOutput;
}

// class armClass {
//   public:
//     int position;
//     int speed;

//     void moveArm(){
//  if (m_armController.GetYButton()){
//     desired = 200;
//   }
//   if (m_armController.GetXButton()){
//     desired = 300;
//   }
//   if (m_armController.GetAButton()){
//     desired = 600;
//   }
//   if (m_armController.GetBButton()){
//     desired = 400;
//   }
//     }
// };

// class Intake: public armClass{
//   intakeNote(){
//     if (m_armController.GetLeftBumper){
//       desired = 200;
//       m_shooterDrive.ArcadeDrive(calculateError(desired), 0.0, false);
//     }
//   }
// };

// class Shooter: public armClass{
//   shootNote(){
//     if (m_armController.GetRightBumper){
//       desired = -200;
//       m_shooterDrive.ArcadeDrive(calculateError(desired), 0.0, false);
//     }
//   }
// };

  void AutonomousInit() override { 
    m_timer.Restart(); //restarts time for autonomous modes
    m_autoSelected = m_chooser.GetSelected(); //sets autonomous modes
    std::cout << "Auto selected: " << m_autoSelected << "\n";
    }

  void AutonomousPeriodic() override {
    //custom auto
    if (m_autoSelected == kAutoMidPos) {
    std::cout << "Middle Position Auto!\n";
  } 
  else if (m_autoSelected == kAutoRightPos){
    std::cout << "Right Position Auto!\n";
  }
  else { //default auto
        if (m_timer.Get() < 1_s) {
      m_robotDrive.ArcadeDrive(0.5, 0.0
      , false);
      // std::cout << m_timer << "\n";
    } else {
      // stop robot
      m_robotDrive.StopMotor();
      }
    }
  }

  void TeleopInit() override {
    m_encoder.Reset(); //resets every match
  }

  void TeleopPeriodic() override {
  if (m_driveController.GetLeftY() != 0){
    m_robotDrive.ArcadeDrive(m_driveController.GetLeftX() * 0.25, m_driveController.GetLeftY() * 0.1, false);
  }
  else{
    m_robotDrive.ArcadeDrive(m_driveController.GetLeftX() * 0.25, 0.1, false);
  }
  arm_motor_2.Set(calculateError(desired));
  // m_armDrive.ArcadeDrive(calculateError(desired), 0.0, false); //black to green, red to white 

  double encoderRate = m_encoder.GetRate();
  double distancePerPulse = m_encoder.GetDistancePerPulse();
  double encoderCount = m_encoder.Get();
  double encoderDistance = m_encoder.GetDistance();

  m_robotDrive.ArcadeDrive(m_driveController.GetLeftX()*0.25, m_driveController.GetRightY()*0.25, false);
  arm_motor_2.Set(calculateError(desired));

        if (m_armController.GetYButton()){
    std::cout << encoderDistance << "\n";
  }


  // m_armDrive.ArcadeDrive(calculateError(desired), 0.0, false);

  // intakeVar.intakeNote();
  // shooterVar.shootNote();
  // armVar.moveArm();
    
  }

  void TestInit() override {
    m_encoder.Reset();
  }

  void TestPeriodic() override { // move all to teleop eventually

  double encoderRate = m_encoder.GetRate();
  double distancePerPulse = m_encoder.GetDistancePerPulse();
  double encoderCount = m_encoder.Get();
  double encoderDistance = m_encoder.GetDistance();

  frc::SmartDashboard::PutNumber("Encoder distance: ", encoderDistance);
  frc::SmartDashboard::PutNumber("Encoder Distance Per Pulse: ", distancePerPulse);
  frc::SmartDashboard::PutNumber("Encoder count: ", encoderCount);
  frc::SmartDashboard::PutNumber("Encoder rate: ", encoderRate);

  m_robotDrive.ArcadeDrive(m_driveController.GetLeftY()/2, m_driveController.GetRightX()/2, false);

  // m_armDrive.ArcadeDrive(m_armController.GetLeftY()*0.8, 0.0, false);

  m_shooterDrive.ArcadeDrive(m_armController.GetRightY(), 0.0, false);


  if (m_armController.GetLeftBumper()){
    intake_motor_3.Set(1);
    }
  else if (m_armController.GetRightBumper()){
    intake_motor_3.Set(1);
  }
  else if (m_armController.GetLeftBumperReleased()){
    intake_motor_3.Set(0);
  }
  else if (m_armController.GetRightBumperReleased()){
    intake_motor_3.Set(0);
  }

  if (m_armController.GetYButton()){
    desired = -100;
  }
  else if (m_armController.GetXButton()){
    desired = -30; //change
  }
  else if (m_armController.GetAButton()){
    desired = -300;
  }
  else if (m_armController.GetBButton()){
    desired = -760;
  }
  
  arm_motor_2.Set(calculateError(desired));
  }

 private:
  // Robot drive system

  //motor controllers
  WPI_TalonSRX rightside_leader{1};
  WPI_VictorSPX rightside_follower{2};
  WPI_VictorSPX leftside_leader{3};
  WPI_VictorSPX leftside_follower{4};

  WPI_VictorSPX intake_motor_1{5}; //shooter 1
  WPI_TalonSRX intake_motor_2{6}; //shooter 2
  WPI_VictorSPX intake_motor_3{7};//grab

  WPI_VictorSPX arm_motor_1{8}; 
  WPI_VictorSPX arm_motor_2{9};

// drive modes
  frc::DifferentialDrive m_robotDrive{
      [&](double output) { rightside_leader.Set(output); },
      [&](double output) { leftside_leader.Set(output); }};

  frc::DifferentialDrive m_armDrive{
      [&](double output) { arm_motor_1.Set(output); },
      [&](double output) { arm_motor_2.Set(output); }};

  frc::DifferentialDrive m_shooterDrive{
      [&](double output) { intake_motor_1.Set(output); },
      [&](double output) { intake_motor_2.Set(output); }};

  frc::XboxController m_driveController{0};
  frc::XboxController m_armController{1};
  frc::Timer m_timer; //creation of autonomous timer
  frc::DigitalInput m_limitswitch{2};// limit switch uses pin 2
  frc::Encoder m_encoder{0,1}; //encoder uses pins 0-1

  frc::SendableChooser<std::string> m_chooser; //autonomous modes
  const std::string kAutoLeftPos = "Position_Left";
  const std::string kAutoMidPos = "Position_Mid";
  const std::string kAutoRightPos = "Position_Right";
  std::string m_autoSelected;

  double position = 0;
  int desired = 0; //desired is the desired distance we want the motor to reach
  double p = 0.005;
  double throttle = 0.5;

  // Intake intakeVar;
  // Shooter shooterVar;
  // armClass armVar;
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif