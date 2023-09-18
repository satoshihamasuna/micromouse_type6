/*
 * run_task.cpp
 *
 *  Created on: 2023/06/16
 *      Author: sato1
 */

#include "../../Module/Include/index.h"
#include "../../Module/Include/typedef.h"
#include "run_task.h"
#include "../../Module/Include/macro.h"
#include "turn_table.h"
#include "sensing_task.h"
#include "motion.h"
#define BRAKE_TIME_LIMIT (200)

void RunTask::MotionFree(float *run_time,float run_time_limit)
{
	is_runTask = True;
	run_mode_state = NOP_MODE;
	Motor_SetDuty_Right(500);
	Motor_SetDuty_Left(500);
	*run_time = *run_time + 1.0f;
	if(*run_time > run_time_limit)
	{
		is_runTask = False;
	}
}

void RunTask::search_straight(t_motion_param mt_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms)
{

	is_wallControl_Enable = Enable_st;
	is_runTask = True;
	set_run_mode_state(STRAIGHT_MODE);
	float deccel_length = 1000*(mt_param.max_velo*mt_param.max_velo
								-mt_param.end_velo*mt_param.end_velo)
								/(2.0*ABS(mt_param.deccel));

	if(deccel_length < ( mt_param.length - machine_->length ))
	{
		target_->accel = mt_param.accel;
		target_->velo  = target_->velo + target_->accel*delta_t_ms/1000.0;
		if(target_->velo > mt_param.max_velo)
		{
			target_->velo = mt_param.max_velo;
			target_->accel = 0.0;
		}
		if(SensingTask::getInstance().Division_Wall_Correction() == True)
		{
			if(mt_param.length == 90.0f)
			{
				machine_->length = (machine_->length + 45.0)/2.0f;
				Indicate_LED(0xff);
			}
		}

	}
	else if(mt_param.length > machine_->length)
	{
		target_->accel = mt_param.deccel;
		target_->velo  = target_->velo + target_->accel*delta_t_ms/1000.0;

		if(mt_param.end_velo == 0.0f)
		{
			if(target_->velo < 0.15)
			{
				is_wallControl_Enable = Non_controll;
				target_->velo = 0.15;
				target_->accel = 0.0;
				target_->rad_velo = 0.0;
				target_->rad_accel = 0.0;
				set_run_mode_state(NOP_MODE);
			}
		}
		else if(target_->velo < mt_param.end_velo)
		{
			target_->velo = mt_param.end_velo;
			target_->accel = 0.0;

		}

	}
	else
	{
		if(mt_param.end_velo == 0.0f)
		{
			target_->velo = 0.0f;
			target_->accel = 0.0;
			target_->rad_velo = 0.0;
			target_->rad_accel = 0.0;
			is_wallControl_Enable = Non_controll;
			set_run_mode_state(NOP_MODE);
		}
		else
		{
			is_runTask = False;
			target_->accel = 0.0;
		}
	}

	if(target_->velo == 0.0f)
	{
		is_wallControl_Enable = Non_controll;
		is_runTask = True;
		brake_time++;
		if(brake_time > BRAKE_TIME_LIMIT)
		{
			is_runTask = False;
			brake_time = 0;
		}
	}

	if(is_wallControl_Enable != Non_controll)
	{
		SensingTask::getInstance().SetWallControll_RadVelo(&(*target_),&(*machine_),1.0);
	}
	target_->radian = target_->radian + target_->rad_velo*delta_t_ms/1000.0f;
}

void RunTask::straight(t_motion_param mt_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms)
{

	is_wallControl_Enable = Enable_st;
	is_runTask = True;
	set_run_mode_state(STRAIGHT_MODE);

	float deccel_length = 1000*(mt_param.max_velo*mt_param.max_velo
								-mt_param.end_velo*mt_param.end_velo)
								/(2.0*ABS(mt_param.deccel)) + 0.0;
	if(mt_param.max_velo > mt_param.end_velo) deccel_length += 10.0;
	if(deccel_length < ( mt_param.length - machine_->length ))
	{
		target_->accel = mt_param.accel;
		target_->velo  = target_->velo + target_->accel*delta_t_ms/1000.0;
		if(target_->velo > mt_param.max_velo)
		{
			target_->velo = mt_param.max_velo;
			target_->accel = 0.0;
		}


	}
	else if(mt_param.length > machine_->length)
	{
		target_->accel = mt_param.deccel;
		target_->velo  = target_->velo + target_->accel*delta_t_ms/1000.0;

		if(mt_param.end_velo == 0.0f)
		{
			if(target_->velo < 0.15)
			{
				is_wallControl_Enable = Non_controll;
				target_->velo = 0.15;
				target_->accel = 0.0;
				target_->rad_velo = 0.0;
				target_->rad_accel = 0.0;
				set_run_mode_state(NOP_MODE);
			}
		}
		else if(target_->velo < mt_param.end_velo)
		{
			target_->velo = mt_param.end_velo;
			target_->accel = 0.0;


		}

	}
	else
	{
		if(mt_param.end_velo == 0.0f)
		{
			is_wallControl_Enable = Non_controll;
			target_->velo = 0.0f;
			target_->accel = 0.0;
			target_->rad_velo = 0.0;
			target_->rad_accel = 0.0;
			set_run_mode_state(NOP_MODE);
		}
		else
		{
			is_runTask = False;
			target_->accel = 0.0;
		}
	}

	if(target_->velo == 0.0f)
	{
		is_wallControl_Enable = Non_controll;
		target_->rad_velo = 0.0;
		target_->rad_accel = 0.0;
		is_runTask = True;
		brake_time++;
		if(brake_time > BRAKE_TIME_LIMIT)
		{
			is_runTask = False;
			brake_time = 0;
		}
	}

	if(is_wallControl_Enable != Non_controll)
	{
		SensingTask::getInstance().SetWallControll_RadVelo(&(*target_),&(*machine_),1.0);
	}
	target_->radian = target_->radian + target_->rad_velo*delta_t_ms/1000.0f;
}

void RunTask::diagonal(t_motion_param mt_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms)
{

	is_wallControl_Enable = Enable_di;
	is_runTask = True;
	set_run_mode_state(DIAGONAL_MODE);
	motion_task::getInstance().ct.speed_ctrl.Gain_Set(6.0, 0.1, 0.0);//7.0/0.3
	motion_task::getInstance().ct.omega_ctrl.Gain_Set(0.05, 0.01, 0.00);
	float deccel_length = 1000*(mt_param.max_velo*mt_param.max_velo
								-mt_param.end_velo*mt_param.end_velo)
								/(2.0*ABS(mt_param.deccel));
	if(deccel_length < ( mt_param.length - machine_->length ))
	{
		target_->accel = mt_param.accel;
		target_->velo  = target_->velo + target_->accel*delta_t_ms/1000.0;
		if(target_->velo > mt_param.max_velo)
		{
			target_->velo = mt_param.max_velo;
			target_->accel = 0.0;
		}


	}
	else if(mt_param.length > machine_->length)
	{
		target_->accel = mt_param.deccel;
		target_->velo  = target_->velo + target_->accel*delta_t_ms/1000.0;

		if(mt_param.end_velo == 0.0f)
		{
			if(target_->velo < 0.15)
			{
				target_->velo = 0.15;
				target_->accel = 0.0;
			}
		}
		else if(target_->velo < mt_param.end_velo)
		{
			target_->velo = mt_param.end_velo;
			target_->accel = 0.0;

		}

	}
	else
	{
		if(mt_param.end_velo == 0.0f)
		{
			target_->velo = 0.0f;
			target_->accel = 0.0;
			target_->rad_velo = 0.0;
			target_->rad_accel = 0.0;
		}
		else
		{
			is_runTask = False;
			target_->accel = 0.0;
		}
	}

	if(target_->velo == 0.0f)
	{
		is_runTask = True;
		brake_time++;
		target_->rad_velo = 0.0;
		target_->rad_accel = 0.0;
		if(brake_time > BRAKE_TIME_LIMIT)
		{
			is_runTask = False;
			brake_time = 0;
		}
	}

	if(is_wallControl_Enable != Non_controll)
	{
		SensingTask::getInstance().SetWallControll_RadVelo(&(*target_),&(*machine_),1.0);
	}
	target_->radian = target_->radian + target_->rad_velo*delta_t_ms/1000.0f;
}

void RunTask::pivotturn(t_motion_param mt_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms)
{

	is_wallControl_Enable = Non_controll;
	is_runTask = True;
	set_run_mode_state(SPIN_TURN_MODE);
	target_->velo  = 0.0;
	target_ ->accel = 0.0;
	machine_->x_point = 0.0f;
	float deccel_radian = (mt_param.rad_max_velo*mt_param.rad_max_velo)/(2.0*ABS(mt_param.rad_accel));


	if(ABS(deccel_radian) < ( ABS(mt_param.radian) - ABS(machine_->radian)))
	{
		target_->rad_accel = mt_param.rad_accel;
		target_->rad_velo  = target_->rad_velo + target_->rad_accel*delta_t_ms/1000.0;
		if(ABS(target_->rad_velo) > ABS(mt_param.rad_max_velo))
		{
			target_->rad_velo = mt_param.rad_max_velo;
		}
	}

	else if(ABS(mt_param.radian) > ABS(machine_->radian))
	{
		target_->rad_accel = mt_param.rad_deccel;
		target_->rad_velo  = target_->rad_velo + target_->rad_accel*delta_t_ms/1000.0;

		if(target_->rad_velo <= 0.0 && mt_param.radian > 0.0)
		{
			target_->rad_velo = 0.0 ;
		}
		else if(target_->rad_velo >= 0.0 && mt_param.radian < 0.0)
		{
			target_->rad_velo = 0.0 ;
		}

	}
	else
	{
		target_->rad_velo = 0.0 ;
		target_->accel = 0.0f;
		target_->velo = 0.0f;
		target_->rad_accel = 0.0f;
		target_->radian = 0.0f;
		//is_runTask = False;
	}
	target_->radian = target_->radian + target_->rad_velo*delta_t_ms/1000.0f;


	if(target_->rad_velo == 0.0f)
	{
		is_runTask = True;
		brake_time++;
		if(brake_time > BRAKE_TIME_LIMIT)
		{
			is_runTask = False;
			brake_time = 0;
		}
	}

}

void RunTask::search_slalom(t_motion_param *mt_param,const t_param *turn_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms)
{
	is_runTask = True;
	target_->velo = turn_param->param->velo;
	target_->accel = 0.0;
	//if(mt_param->radian ==  0.0 && mt_param->turn_d == Turn_None)
	if(mt_param->turn_d == Prev_Turn)
	{
		is_wallControl_Enable = Enable_st;
		set_run_mode_state(STRAIGHT_MODE);
		run_turn_table_time = 0.0f;
		if(machine_->length < turn_param->param->Lstart)
		{
			target_->velo = turn_param->param->velo;


			if(SensingTask::getInstance().sen_fr.is_wall == True && SensingTask::getInstance().sen_fl.is_wall == True)
			{
				float len_sens = 90.0-(SensingTask::getInstance().sen_fr.distance + SensingTask::getInstance().sen_fl.distance)/2.0 + 0.0;
				//if(len_sens > 0.0)  						machine_->length = len_sens;
				//else if(len_sens < 0.0)
				machine_->length = 0.5*len_sens + 0.5*machine_->length;
			}


		}
		else
		{
			mt_param->turn_d =  turn_param->param->turn_dir;
		}
		Indicate_LED(0xff);
	}

	//if(mt_param->radian ==  DEG2RAD(turn_param->param->degree) && mt_param->turn_d == Turn_None)
	if(mt_param->turn_d == Post_Turn)
	{
		is_wallControl_Enable = Enable_st;
		set_run_mode_state(STRAIGHT_MODE);
		run_turn_table_time = 0.0f;
		if(machine_->length < (turn_param->param->Lend+post_run_fix))
		{
			target_->velo = turn_param->param->velo;
		}
		else
		{
			is_runTask = False;
		}
		Indicate_LED(0x02);
	}

	if( mt_param->turn_d ==  turn_param->param->turn_dir)
	{
		is_wallControl_Enable = Non_controll;
		set_run_mode_state(TURN_MODE);
		float turn_time_limit = DEG2RAD(turn_param->param->degree)/(accel_Integral*mt_param->rad_max_velo);
		machine_->length = 0.0;
		if(run_turn_table_time <= (turn_time_limit*1000.0f))
		{
			int std_a = (int)(run_turn_table_time/turn_time_limit);
			int std_b = std_a + 1;
			float m = run_turn_table_time/turn_time_limit - (float)(std_a);
			float n = (float)(std_b) - run_turn_table_time/turn_time_limit;
			float set_rad_velo =  mt_param->rad_max_velo*(n*accel_table[std_a] + m*accel_table[std_b]);

			float next_time = (run_turn_table_time + delta_t_ms);
			std_a = (int)(next_time/turn_time_limit);
			std_b = std_a + 1;
			m = next_time/turn_time_limit - (float)(std_a);
			n = (float)(std_b) - next_time/turn_time_limit;

			float next_rad_velo = 0.0;
			if(next_time < (turn_time_limit*1000.0f))
				next_rad_velo = mt_param->rad_max_velo*(n*accel_table[std_a] + m*accel_table[std_b]);

			target_->rad_accel =(next_rad_velo - set_rad_velo)*1000.0f/delta_t_ms;//(set_rad_velo - target_->rad_velo)*1000.0f/delta_t_ms;
			target_->rad_velo = set_rad_velo;
			//target_->accel = ABS(machine_->accel);
			Indicate_LED(0x04+0x08);
		}
		run_turn_table_time = run_turn_table_time + delta_t_ms;
		if(run_turn_table_time > (turn_time_limit*1000.0f))
		{
			mt_param->radian =  DEG2RAD(turn_param->param->degree);
			mt_param->turn_d = Post_Turn;
			machine_->length = 0.0;
			target_->rad_velo = 0.0f;
			target_->rad_accel = 0.0f;
			target_->radian = 0.0f;
			machine_->radian = 0.0f;
			machine_->x_point = 0.0f;
			Indicate_LED(0x04);

		}

	}
	if(is_wallControl_Enable != Non_controll)
	{
		SensingTask::getInstance().SetWallControll_RadVelo(&(*target_),&(*machine_),1.0);
	}
	target_->radian = target_->radian + target_->rad_velo*delta_t_ms/1000.0f;

}


void RunTask::turn_in(t_motion_param *mt_param,const t_param *turn_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms)
{
	is_runTask = True;
	target_->velo = turn_param->param->velo;
	target_->accel = 0.0;
	//if(mt_param->radian ==  0.0 && mt_param->turn_d == Turn_None)
	if(mt_param->turn_d == Prev_Turn)
	{
		is_wallControl_Enable = Enable_st;
		run_turn_table_time = 0.0f;
		set_run_mode_state(STRAIGHT_MODE);
		if(machine_->length < (turn_param->param->Lstart+prev_run_fix))
		{
			target_->velo = turn_param->param->velo;
			if(SensingTask::getInstance().Division_Wall_Correction() == True)
			{
				machine_->length = (0.2*machine_->length + 0.8*(6.0));
				Indicate_LED(0xff);
			}
		}
		else
		{
			mt_param->turn_d =  turn_param->param->turn_dir;
		}
		Indicate_LED(0x01);
	}

	//if(mt_param->radian ==  DEG2RAD(turn_param->param->degree) && mt_param->turn_d == Turn_None)
	if(mt_param->turn_d == Post_Turn)
	{
		is_wallControl_Enable = Enable_di;
		run_turn_table_time = 0.0f;
		set_run_mode_state(DIAGONAL_MODE);
		if(machine_->length < (turn_param->param->Lend+post_run_fix))
		{
			target_->velo = turn_param->param->velo;
		}
		else
		{
			is_runTask = False;
		}
		Indicate_LED(0x02);
	}

	if( mt_param->turn_d ==  turn_param->param->turn_dir)
	{
		is_wallControl_Enable = Non_controll;
		set_run_mode_state(TURN_MODE);
		float turn_time_limit = DEG2RAD(turn_param->param->degree)/(accel_Integral*mt_param->rad_max_velo);
		machine_->length = 0.0;
		if(run_turn_table_time <= (turn_time_limit*1000.0f))
		{
			int std_a = (int)(run_turn_table_time/turn_time_limit);
			int std_b = std_a + 1;
			float m = run_turn_table_time/turn_time_limit - (float)(std_a);
			float n = (float)(std_b) - run_turn_table_time/turn_time_limit;
			float set_rad_velo =  mt_param->rad_max_velo*(n*accel_table[std_a] + m*accel_table[std_b]);

			float next_time = (run_turn_table_time + delta_t_ms);
			std_a = (int)(next_time/turn_time_limit);
			std_b = std_a + 1;
			m = next_time/turn_time_limit - (float)(std_a);
			n = (float)(std_b) - next_time/turn_time_limit;

			float next_rad_velo = 0.0;
			if(next_time < (turn_time_limit*1000.0f))
				next_rad_velo = mt_param->rad_max_velo*(n*accel_table[std_a] + m*accel_table[std_b]);

			target_->rad_accel =(next_rad_velo - set_rad_velo)*1000.0f/delta_t_ms;//(set_rad_velo - target_->rad_velo)*1000.0f/delta_t_ms;
			target_->rad_velo = set_rad_velo;
			//target_->accel = (-1.0)*machine_->accel;
			Indicate_LED(0x04+0x08);
		}
		run_turn_table_time = run_turn_table_time + delta_t_ms;
		if(run_turn_table_time > (turn_time_limit*1000.0f))
		{
			mt_param->radian =  DEG2RAD(turn_param->param->degree);
			//mt_param->turn_d = Turn_None;
			mt_param->turn_d = Post_Turn;
			machine_->length = 0.0;
			target_->rad_velo = 0.0f;
			target_->rad_accel = 0.0f;
			target_->radian = 0.0f;
			machine_->radian = 0.0f;
			machine_->x_point = 0.0f;
			Indicate_LED(0x04);

		}

	}
	if(is_wallControl_Enable != Non_controll)
	{
		SensingTask::getInstance().SetWallControll_RadVelo(&(*target_),&(*machine_),1.0);
	}
	target_->radian = target_->radian + target_->rad_velo*delta_t_ms/1000.0f;

}


void RunTask::turn_out(t_motion_param *mt_param,const t_param *turn_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms)
{
	is_runTask = True;
	target_->velo = turn_param->param->velo;
	target_->accel = 0.0;
	//if(mt_param->radian ==  0.0 && mt_param->turn_d == Turn_None)
	if(mt_param->turn_d == Prev_Turn)
	{
		set_run_mode_state(DIAGONAL_MODE);
		is_wallControl_Enable = Enable_di;
		run_turn_table_time = 0.0f;
		if(machine_->length < turn_param->param->Lstart)
		{
			target_->velo = turn_param->param->velo;
			if(SensingTask::getInstance().Division_Wall_Correction() == True)
			{
				if(turn_param->param->turn_dir == Turn_R && SensingTask::getInstance().sen_r.is_wall == False)
				{
					machine_->length = (0.5*machine_->length + 0.5*(0.0));
					Indicate_LED(0xff);
				}else if(turn_param->param->turn_dir == Turn_L && SensingTask::getInstance().sen_l.is_wall == False)
				{
					machine_->length = (0.5*machine_->length + 0.5*(0.0));
					Indicate_LED(0xff);
				}
			}
		}
		else
		{
			mt_param->turn_d =  turn_param->param->turn_dir;
		}
		Indicate_LED(0x01);
	}

	//if(mt_param->radian ==  DEG2RAD(turn_param->param->degree) && mt_param->turn_d == Turn_None)
	if(mt_param->turn_d == Post_Turn)
	{
		is_wallControl_Enable = Enable_st;
		run_turn_table_time = 0.0f;
		set_run_mode_state(STRAIGHT_MODE);
		if(machine_->length < turn_param->param->Lend)
		{
			target_->velo = turn_param->param->velo;
			if(SensingTask::getInstance().Division_Wall_Correction() == True)
			{
				machine_->length = (0.2*machine_->length + 0.8*(turn_param->param->Lend));
				Indicate_LED(0xff);
			}
		}
		else
		{
			is_runTask = False;
		}
		Indicate_LED(0x02);
	}

	if( mt_param->turn_d ==  turn_param->param->turn_dir)
	{
		is_wallControl_Enable = Non_controll;
		float turn_time_limit = DEG2RAD(turn_param->param->degree)/(accel_Integral*mt_param->rad_max_velo);
		set_run_mode_state(TURN_MODE);
		machine_->length = 0.0;

		if(run_turn_table_time <= (turn_time_limit*1000.0f))
		{
			int std_a = (int)(run_turn_table_time/turn_time_limit);
			int std_b = std_a + 1;
			float m = run_turn_table_time/turn_time_limit - (float)(std_a);
			float n = (float)(std_b) - run_turn_table_time/turn_time_limit;
			float set_rad_velo =  mt_param->rad_max_velo*(n*accel_table[std_a] + m*accel_table[std_b]);

			float next_time = (run_turn_table_time + delta_t_ms);
			std_a = (int)(next_time/turn_time_limit);
			std_b = std_a + 1;
			m = next_time/turn_time_limit - (float)(std_a);
			n = (float)(std_b) - next_time/turn_time_limit;

			float next_rad_velo = 0.0;
			if(next_time < (turn_time_limit*1000.0f))
				next_rad_velo = mt_param->rad_max_velo*(n*accel_table[std_a] + m*accel_table[std_b]);

			target_->rad_accel =(next_rad_velo - set_rad_velo)*1000.0f/delta_t_ms;//(set_rad_velo - target_->rad_velo)*1000.0f/delta_t_ms;
			target_->rad_velo = set_rad_velo;
			//target_->accel = (-1.0)*machine_->accel;
			Indicate_LED(0x04+0x08);
		}
		run_turn_table_time = run_turn_table_time + delta_t_ms;
		if(run_turn_table_time > (turn_time_limit*1000.0f))
		{
			mt_param->radian =  DEG2RAD(turn_param->param->degree);
			SensingTask::getInstance().Division_Wall_Correction_Reset();
			motion_task::getInstance().ct.speed_ctrl.I_param_reset();
			//mt_param->turn_d = Turn_None;
			mt_param->turn_d = Post_Turn;
			machine_->length = 0.0;
			machine_->radian = 0.0f;
			target_->rad_velo = 0.0f;
			target_->rad_accel = 0.0f;
			target_->radian = 0.0f;
			machine_->x_point = 0.0f;
			Indicate_LED(0x04);

		}

	}
	if(is_wallControl_Enable != Non_controll)
	{
		SensingTask::getInstance().SetWallControll_RadVelo(&(*target_),&(*machine_),1.0);
	}
	target_->radian = target_->radian + target_->rad_velo*delta_t_ms/1000.0f;
}

void RunTask::long_turn(t_motion_param *mt_param,const t_param *turn_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms)
{
	is_runTask = True;
	target_->velo = turn_param->param->velo;
	target_->accel = 0.0;
	//if(mt_param->radian ==  0.0 && mt_param->turn_d == Turn_None)
	if(mt_param->turn_d == Prev_Turn)
	{
		is_wallControl_Enable = Enable_st;
		run_turn_table_time = 0.0f;
		set_run_mode_state(STRAIGHT_MODE);
		if(machine_->length < (turn_param->param->Lstart+prev_run_fix))
		{
			target_->velo = turn_param->param->velo;

			if(SensingTask::getInstance().Division_Wall_Correction() == True)
			{
				machine_->length = (0.2*machine_->length + 0.8*(5.0));
				Indicate_LED(0xff);
			}
		}
		else
		{
			mt_param->turn_d =  turn_param->param->turn_dir;
			SensingTask::getInstance().Division_Wall_Correction_Reset();
		}
		Indicate_LED(0x01);
	}

	//if(mt_param->radian ==  DEG2RAD(turn_param->param->degree) && mt_param->turn_d == Turn_None)
	if(mt_param->turn_d == Post_Turn)
	{
		is_wallControl_Enable = Enable_st;
		run_turn_table_time = 0.0f;
		set_run_mode_state(STRAIGHT_MODE);
		if(machine_->length < (turn_param->param->Lend+post_run_fix))
		{
			target_->velo = turn_param->param->velo;
			if(SensingTask::getInstance().Division_Wall_Correction() == True)
			{

				machine_->length = (0.2*machine_->length + 0.8*(turn_param->param->Lend+post_run_fix));
				Indicate_LED(0xff);
			}
		}
		else
		{
			is_runTask = False;
		}
		Indicate_LED(0x02);
	}

	if( mt_param->turn_d ==  turn_param->param->turn_dir)
	{
		is_wallControl_Enable = Non_controll;
		float turn_time_limit = DEG2RAD(turn_param->param->degree)/(accel_Integral*mt_param->rad_max_velo);
		machine_->length = 0.0;
		set_run_mode_state(TURN_MODE);
		if(run_turn_table_time <= (turn_time_limit*1000.0f))
		{
			int std_a = (int)(run_turn_table_time/turn_time_limit);
			int std_b = std_a + 1;
			float m = run_turn_table_time/turn_time_limit - (float)(std_a);
			float n = (float)(std_b) - run_turn_table_time/turn_time_limit;
			float set_rad_velo =  mt_param->rad_max_velo*(n*accel_table[std_a] + m*accel_table[std_b]);

			float next_time = (run_turn_table_time + delta_t_ms);
			std_a = (int)(next_time/turn_time_limit);
			std_b = std_a + 1;
			m = next_time/turn_time_limit - (float)(std_a);
			n = (float)(std_b) - next_time/turn_time_limit;

			float next_rad_velo = 0.0;
			if(next_time < (turn_time_limit*1000.0f))
				next_rad_velo = mt_param->rad_max_velo*(n*accel_table[std_a] + m*accel_table[std_b]);

			target_->rad_accel =(next_rad_velo - set_rad_velo)*1000.0f/delta_t_ms;//(set_rad_velo - target_->rad_velo)*1000.0f/delta_t_ms;
			target_->rad_velo = set_rad_velo;
			//target_->accel = (-1.0)*machine_->accel;
			Indicate_LED(0x04+0x08);
		}
		run_turn_table_time = run_turn_table_time + delta_t_ms;
		if(run_turn_table_time > (turn_time_limit*1000.0f))
		{
			mt_param->radian =  DEG2RAD(turn_param->param->degree);
			//mt_param->turn_d = Turn_None;
			SensingTask::getInstance().Division_Wall_Correction_Reset();

			mt_param->turn_d = Post_Turn;
			machine_->length = 0.0;
			machine_->radian = 0.0f;
			target_->rad_velo = 0.0f;
			target_->rad_accel = 0.0f;
			target_->radian = 0.0f;
			machine_->x_point = 0.0f;
			Indicate_LED(0x04);
		}

	}
	if(is_wallControl_Enable != Non_controll)
	{
		SensingTask::getInstance().SetWallControll_RadVelo(&(*target_),&(*machine_),1.0);
	}
	target_->radian = target_->radian + target_->rad_velo*delta_t_ms/1000.0f;

}


void RunTask::turn_v90(t_motion_param *mt_param,const t_param *turn_param,t_machine_param *target_,t_machine_param *machine_,float delta_t_ms)
{
	is_runTask = True;
	target_->velo = turn_param->param->velo;
	target_->accel = 0.0;
	//if(mt_param->radian ==  0.0 && mt_param->turn_d == Turn_None)
	if(mt_param->turn_d == Prev_Turn)
	{
		is_wallControl_Enable = Enable_di;
		run_turn_table_time = 0.0f;
		set_run_mode_state(DIAGONAL_MODE);
		if(machine_->length < turn_param->param->Lstart)
		{
			target_->velo = turn_param->param->velo;
			if(SensingTask::getInstance().Division_Wall_Correction() == True)
			{
				if(turn_param->param->turn_dir == Turn_R && SensingTask::getInstance().sen_r.is_wall == False)
				{
					machine_->length = (0.5*machine_->length + 0.5*(0.0));
					Indicate_LED(0xff);
				}else if(turn_param->param->turn_dir == Turn_L && SensingTask::getInstance().sen_l.is_wall == False)
				{
					machine_->length = (0.5*machine_->length + 0.5*(0.0));
					Indicate_LED(0xff);
				}
			}
		}
		else
		{
			mt_param->turn_d =  turn_param->param->turn_dir;
			motion_task::getInstance().ct.omega_ctrl.I_param_reset();
		}
		Indicate_LED(0x01);
	}

	//if(mt_param->radian ==  DEG2RAD(turn_param->param->degree) && mt_param->turn_d == Turn_None)
	if(mt_param->turn_d == Post_Turn)
	{
		is_wallControl_Enable = Enable_di;
		run_turn_table_time = 0.0f;
		set_run_mode_state(DIAGONAL_MODE);
		if(machine_->length < turn_param->param->Lend)
		{
			target_->velo = turn_param->param->velo;
		}
		else
		{
			is_runTask = False;
		}
		Indicate_LED(0x02);
	}

	if( mt_param->turn_d ==  turn_param->param->turn_dir)
	{
		is_wallControl_Enable = Non_controll;
		set_run_mode_state(TURN_MODE);
		float turn_time_limit = DEG2RAD(turn_param->param->degree)/(accel_Integral*mt_param->rad_max_velo);
		machine_->length = 0.0;
		if(run_turn_table_time <= (turn_time_limit*1000.0f))
		{
			int std_a = (int)(run_turn_table_time/turn_time_limit);
			int std_b = std_a + 1;
			float m = run_turn_table_time/turn_time_limit - (float)(std_a);
			float n = (float)(std_b) - run_turn_table_time/turn_time_limit;
			float set_rad_velo =  mt_param->rad_max_velo*(n*accel_table[std_a] + m*accel_table[std_b]);

			float next_time = (run_turn_table_time + delta_t_ms);
			std_a = (int)(next_time/turn_time_limit);
			std_b = std_a + 1;
			m = next_time/turn_time_limit - (float)(std_a);
			n = (float)(std_b) - next_time/turn_time_limit;

			float next_rad_velo = 0.0;
			if(next_time < (turn_time_limit*1000.0f))
				next_rad_velo = mt_param->rad_max_velo*(n*accel_table[std_a] + m*accel_table[std_b]);

			target_->rad_accel =(next_rad_velo - set_rad_velo)*1000.0f/delta_t_ms;//(set_rad_velo - target_->rad_velo)*1000.0f/delta_t_ms;
			target_->rad_velo = set_rad_velo;
			//target_->accel = (-1.0)*machine_->accel;
			Indicate_LED(0x04+0x08);
		}
		run_turn_table_time = run_turn_table_time + delta_t_ms;
		if(run_turn_table_time > (turn_time_limit*1000.0f))
		{
			mt_param->radian =  DEG2RAD(turn_param->param->degree);
			//mt_param->turn_d = Turn_None;
			mt_param->turn_d = Post_Turn;
			machine_->length = 0.0;
			machine_->radian = DEG2RAD(turn_param->param->degree)-machine_->radian;
			target_->rad_velo = 0.0f;
			target_->rad_accel = 0.0f;
			target_->radian = 0.0f;
			machine_->x_point = 0.0f;
			motion_task::getInstance().ct.omega_ctrl.I_param_reset();
			Indicate_LED(0x04);
			motion_task::getInstance().ct.speed_ctrl.I_param_reset();
		}

	}

	if(is_wallControl_Enable != Non_controll)
	{
		SensingTask::getInstance().SetWallControll_RadVelo(&(*target_),&(*machine_),1.0);
	}
	target_->radian = target_->radian + target_->rad_velo*delta_t_ms/1000.0f;
}

void RunTask::fix_wall(t_machine_param *target_,float *run_time,float run_time_limit,float delta_t_ms)
{
	is_runTask = True;
	is_wallControl_Enable = Non_controll;
	if(SensingTask::getInstance().sen_fr.distance < 70.0 && SensingTask::getInstance().sen_fl.distance < 70.0)
	{
		set_run_mode_state(SPIN_TURN_MODE);
		float sp_err = ((SensingTask::getInstance().sen_fr.distance - 45.0) + (SensingTask::getInstance().sen_fl.distance - 45.0))/2.0f;
		float om_err = ((SensingTask::getInstance().sen_fr.distance - 45.0) - (SensingTask::getInstance().sen_fl.distance - 45.0))/2.0f;

		target_->accel = (1.0 * sp_err - 100.0*target_->velo);
		target_->velo = target_->velo + target_->accel/1000.0f;
			//target.velo = 0.05 * sp_err;//veloだったら0.05
		float max_set_velo = 0.3;
		if(target_->velo >=  max_set_velo)
		{
			target_->accel = 0.0;
			target_->velo = max_set_velo;
		}
		else if(target_->velo <= -max_set_velo){
			target_->accel = 0.0;
			target_->velo = -max_set_velo;
		}

		target_->rad_accel = (10.0*om_err - 20.0*target_->rad_velo);
		target_->rad_velo  = target_->rad_velo + target_->rad_accel*delta_t_ms/1000.0;
			//target->rad_velo = 0.1*om_err;////veloだったら0.5
		float max_set_rad_velo = 10.0;
		if(target_->rad_velo >= max_set_rad_velo)
		{
			target_->rad_accel = 0.0;
			target_->rad_velo = max_set_rad_velo;
		}
		else if(target_->rad_velo <= -max_set_rad_velo)
		{
			target_->rad_accel = 0.0;
			target_->rad_velo = -max_set_rad_velo;
		}

	}
	else
	{
		target_->accel = 0.0f;		target_->velo = 0.0f;
		target_->rad_accel = 0.0f;	target_->rad_velo = 0.0f;
	}


	*run_time = *run_time + delta_t_ms;
	if(*run_time > run_time_limit)
	{
		target_->accel = 0.0f;
		target_->velo = 0.0f;
		target_->rad_velo = 0.0f;
		target_->rad_accel = 0.0f;
		target_->radian = 0.0f;
		if(*run_time > run_time_limit + 100)
			is_runTask = False;
	}
}

t_bool RunTask::is_exe_runTask()
{
	return is_runTask;
}
