
#include "IRIScontrollerHeader.h"
#include <stdio.h>

#define STOP 0
#define IMPEDANCE_CONTROL 1
#define ADMITTANCE_CONTROL 2

int main()
{
	// 1 : IMPEDANCE CONTROL
	// 2 : ADMITTANCE CONTROL
	int control_type = 1;

	Queue q_pos;
	Queue q_vel;
	Im_Structure Im_main_;
	Ad_Structure Ad_main_;
	AWFilter Filter_vel;
	AWFilter Filter_acc;
	C_state state_;

	
	VariableInitialize(&Im_main_, &Ad_main_, &Filter_vel, &Filter_acc, &state_, &q_pos, &q_vel);

	for(;;)
	{
		switch(control_type)
		{
			case IMPEDANCE_CONTROL:
			{
				velo = _IQtoF(gMotorVars.SpeedQEP_krpm)*5.235987; // krpm -> m/s
				position += velo * INT_DT;
				Im_main_.Im_velo = velo;
				state_.state_ = TORQUE_CONTROL;
				Impedance_controller(&Im_main_,&Filter_vel,&q_pos,&Filter_acc,&q_vel,Mass_,&state_);
				break;
			}
			case ADMITTANCE_CONTROL:
			{
				state_.state_ = SPEED_CONTROL;
			    Ad_main_.Ad_velo =  _IQtoF(gMotorVars.SpeedQEP_krpm)*5.235987; // krpm -> m/s
			    Admittance_controller(&Ad_main_,Mass_,filteredLoad,&state_);
			    gMotorVars.SpeedRef_krpm = _IQ(Ad_main_.Ad_SpeedRef_krpm);
				break;
			}
			default :
			{
				state_.state_ = STOP;
			}
		}
	}

	return 0;

}