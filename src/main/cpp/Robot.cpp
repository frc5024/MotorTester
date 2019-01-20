#include "Robot.h"

#include <iostream>

#define LOG(x) { std::cout << x << std::endl; }

/**
 *
 */
void Robot::RobotInit()
{
	iMotorId = 1;
	iCounter = 0;
	dMotorSpeed = 0.0;
	IsInverted = false;
	IsPhased = false;
	IsClosedMode = false;
	SetMotor(iMotorId);

	this->pXboxController = new frc::XboxController(0);

	return;
}

/**
 *
 */
void Robot::SetMotor(int motor_id)
{
	LOG("Setting ID: " << motor_id);

	// delete previous motor if initialized
	if (this->pTalonSRX != nullptr)
	{
		this->pTalonSRX->Set(ControlMode::PercentOutput, 0);
		delete this->pTalonSRX;
		delete this->pFaults;
	}

	this->pTalonSRX = new WPI_TalonSRX(motor_id);
	this->pTalonSRX->SetInverted(false);
	this->IsInverted = false;

	this->pTalonSRX->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 100);
	this->pTalonSRX->SetSensorPhase(false);
	this->IsPhased = false;

	/* set the peak and nominal outputs, 12V means full */
	this->pTalonSRX->ConfigNominalOutputForward(0, 100);
	this->pTalonSRX->ConfigNominalOutputReverse(0, 100);
	this->pTalonSRX->ConfigPeakOutputForward(1, 100);
	this->pTalonSRX->ConfigPeakOutputReverse(-1, 100);

	this->pTalonSRX->ConfigAllowableClosedloopError(0, 0, 100);

	/* set closed loop gains in slot0 */
	this->pTalonSRX->Config_kP(0, 0.05, 100);
	this->pTalonSRX->Config_kI(0, 0.00, 100);
	this->pTalonSRX->Config_kD(0, 0.00, 100);
	this->pTalonSRX->Config_kF(0, 0.00, 100);

// 	int absolutePosition = this->pTalonSRX->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
	int absolutePosition = this->pTalonSRX->GetSensorCollection().GetPulseWidthPosition();
	this->pTalonSRX->SetSelectedSensorPosition(absolutePosition, 0, 100);
	this->IsClosedMode = false;
	
	this->pFaults = new Faults();
	pTalonSRX->GetFaults(*pFaults);

	return;
}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() 
{
	// switch the motor with the left/right bumpers
	if (this->pXboxController->GetBumperPressed(frc::GenericHID::JoystickHand::kLeftHand))
	{
		iMotorId--;
		if (iMotorId < 1) iMotorId = 1;
		SetMotor(iMotorId);
	}
	else if (this->pXboxController->GetBumperPressed(frc::GenericHID::JoystickHand::kRightHand))
	{
		iMotorId++;
		if (iMotorId > 8) iMotorId = 8;
		SetMotor(iMotorId);
	}

	// invert the talon with the A button
	// when dMotorSpeed is positive the motor should be running forward
	// use the trace to confirm
	if (this->pXboxController->GetAButtonPressed())
	{
		this->IsInverted = !this->IsInverted;
		this->pTalonSRX->SetInverted(IsInverted);
	}

	// change the phase with the B Button
	// if the talon has an encoder use the trace to check the position
	// the position number should increase/decrease when the motor is running forward/reverse
	// use the trace to confirm
	if (this->pXboxController->GetBButtonPressed())
	{
		IsPhased = !IsPhased;
		this->pTalonSRX->SetSensorPhase(IsPhased);
	}

	// enter position closed loop when start button pressed
	// this will only work if the talon has an encoder
	if (this->pXboxController->GetStartButtonPressed())
	{
		this->pTalonSRX->SetSelectedSensorPosition(0, 0, 100);
		double targetPositionRotations = 5.0 * 8192; /* 5 Rotations in either direction */
		this->pTalonSRX->Set(ControlMode::Position, targetPositionRotations);
		this->IsClosedMode = true;
	}
	else if (!this->IsClosedMode)
	{
		double rightTriggerAxis = this->pXboxController->GetTriggerAxis(frc::XboxController::kRightHand);
		double leftTriggerAxis  = this->pXboxController->GetTriggerAxis(frc::XboxController::kLeftHand);

		// dMotorSpeed should be positive when only right trigger is pressed
		// we also want the motor to run forward when dMotorSpeed is positive so it may
		// need to be inverted. confirm this with the trace.  
		// if dMotorSpeed is negative, multply by -1 to make it positive
		this->dMotorSpeed = rightTriggerAxis - leftTriggerAxis;
		this->pTalonSRX->Set(ControlMode::PercentOutput, dMotorSpeed);
	}		

	if (iCounter++ == 10)
	{
//		Robot::Trace();
		LOG("ID: " << iMotorId
			<< " QP: " << this->pTalonSRX->GetSensorCollection().GetQuadraturePosition()
			<< " QV: " << this->pTalonSRX->GetSensorCollection().GetQuadratureVelocity()
			<< " SP: " << this->pTalonSRX->GetSelectedSensorPosition(0)
			<< " SV: " << this->pTalonSRX->GetSelectedSensorVelocity(0)
			<< " IN: " << (IsInverted ? "True" : "False")
			<< " IP: " << (IsPhased ? "True" : "False")
			<< " IC: " << (IsClosedMode ? "True" : "False")
			<< " MS: " << dMotorSpeed
		);
		iCounter = 0;
	}

	return;
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
