#ifndef _ROBOT_H
#define _ROBOT_H


#include <string>
#include <frc/IterativeRobot.h>
#include <frc/WPILib.h>
#include <ctre/Phoenix.h>
#include <frc/GenericHID.h>

class Robot : public frc::IterativeRobot {
public:

	void RobotInit() override;
	void RobotPeriodic() override;
	void AutonomousInit() override;
	void AutonomousPeriodic() override;
	void TeleopInit() override;
	void TeleopPeriodic() override;
	void TestPeriodic() override;

	void SetMotor(int);

 private:
	int    iMotorId;
	int    iCounter;
	double dMotorSpeed;
	bool   IsInverted;
	bool   IsPhased;
	bool   IsClosedMode;

	WPI_TalonSRX* pTalonSRX;
	Faults* pFaults;

	frc::XboxController* pXboxController;
};

#endif