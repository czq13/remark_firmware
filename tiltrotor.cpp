/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file tiltrotor.cpp
 *
 * @author Roman Bapst 		<bapstroman@gmail.com>
 * @author Andreas Antener 	<andreas@uaventure.com>
 *
*/

#include "tiltrotor.h"
#include "vtol_att_control_main.h"

using namespace matrix;
using namespace time_literals;

#define ARSP_YAW_CTRL_DISABLE 7.0f	// airspeed at which we stop controlling yaw during a front transition

Tiltrotor::Tiltrotor(VtolAttitudeControl *attc) :
	VtolType(attc)
{
	_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
	_vtol_schedule.transition_start = 0;

	_mc_roll_weight = 1.0f;
	_mc_pitch_weight = 1.0f;
	_mc_yaw_weight = 1.0f;

	_flag_was_in_trans_mode = false;

	_params_handles_tiltrotor.tilt_mc = param_find("VT_TILT_MC");
	_params_handles_tiltrotor.tilt_transition = param_find("VT_TILT_TRANS");
	_params_handles_tiltrotor.tilt_fw = param_find("VT_TILT_FW");
	_params_handles_tiltrotor.tilt_spinup = param_find("VT_TILT_SPINUP");
	_params_handles_tiltrotor.front_trans_dur_p2 = param_find("VT_TRANS_P2_DUR");
}

void
Tiltrotor::parameters_update()
{
	float v;

	/* vtol tilt mechanism position in mc mode */
	param_get(_params_handles_tiltrotor.tilt_mc, &v);
	_params_tiltrotor.tilt_mc = v;

	/* vtol tilt mechanism position in transition mode */
	param_get(_params_handles_tiltrotor.tilt_transition, &v);
	_params_tiltrotor.tilt_transition = v;

	/* vtol tilt mechanism position in fw mode */
	param_get(_params_handles_tiltrotor.tilt_fw, &v);
	_params_tiltrotor.tilt_fw = v;

	/* vtol tilt mechanism position during motor spinup */
	param_get(_params_handles_tiltrotor.tilt_spinup, &v);
	_params_tiltrotor.tilt_spinup = v;

	/* vtol front transition phase 2 duration */
	param_get(_params_handles_tiltrotor.front_trans_dur_p2, &v);
	_params_tiltrotor.front_trans_dur_p2 = v;
}

void Tiltrotor::update_vtol_state()
{
	/* simple logic using a two way switch to perform transitions.
	 * after flipping the switch the vehicle will start tilting rotors, picking up
	 * forward speed. After the vehicle has picked up enough speed the rotors are tilted
	 * forward completely. For the backtransition the motors simply rotate back.
	*/

	if (!_attc->is_fixed_wing_requested()) {

		// plane is in multicopter mode
		switch (_vtol_schedule.flight_mode) {
		case vtol_mode::MC_MODE:
			break;

		case vtol_mode::FW_MODE:
			_vtol_schedule.flight_mode = vtol_mode::TRANSITION_BACK;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case vtol_mode::TRANSITION_FRONT_P1:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_FRONT_P2:
			// failsafe into multicopter mode
			_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			break;

		case vtol_mode::TRANSITION_BACK:
			float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

			if (_tilt_control <= _params_tiltrotor.tilt_mc && time_since_trans_start > _params->back_trans_duration) {
				_vtol_schedule.flight_mode = vtol_mode::MC_MODE;
			}

			break;
		}

	} else {

		switch (_vtol_schedule.flight_mode) {
		case vtol_mode::MC_MODE:
			// initialise a front transition
			_vtol_schedule.flight_mode = vtol_mode::TRANSITION_FRONT_P1;
			_vtol_schedule.transition_start = hrt_absolute_time();
			break;

		case vtol_mode::FW_MODE:
			break;

		case vtol_mode::TRANSITION_FRONT_P1: {

				float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

				const bool airspeed_triggers_transition = PX4_ISFINITE(_airspeed_validated->equivalent_airspeed_m_s)
						&& !_params->airspeed_disabled;

				bool transition_to_p2 = false;

				if (time_since_trans_start > _params->front_trans_time_min) {
					if (airspeed_triggers_transition) {
						transition_to_p2 = _airspeed_validated->equivalent_airspeed_m_s >= _params->transition_airspeed;

					} else {
						transition_to_p2 = _tilt_control >= _params_tiltrotor.tilt_transition &&
								   time_since_trans_start > _params->front_trans_time_openloop;;
					}
				}

				transition_to_p2 |= can_transition_on_ground();

				if (transition_to_p2) {
					_vtol_schedule.flight_mode = vtol_mode::TRANSITION_FRONT_P2;
					_vtol_schedule.transition_start = hrt_absolute_time();
				}

				break;
			}

		case vtol_mode::TRANSITION_FRONT_P2:

			// if the rotors have been tilted completely we switch to fw mode
			if (_tilt_control >= _params_tiltrotor.tilt_fw) {
				_vtol_schedule.flight_mode = vtol_mode::FW_MODE;
				_tilt_control = _params_tiltrotor.tilt_fw;
			}

			break;

		case vtol_mode::TRANSITION_BACK:
			// failsafe into fixed wing mode
			_vtol_schedule.flight_mode = vtol_mode::FW_MODE;
			break;
		}
	}

	// map tiltrotor specific control phases to simple control modes
	switch (_vtol_schedule.flight_mode) {
	case vtol_mode::MC_MODE:
		_vtol_mode = mode::ROTARY_WING;
		break;

	case vtol_mode::FW_MODE:
		_vtol_mode = mode::FIXED_WING;
		break;

	case vtol_mode::TRANSITION_FRONT_P1:
	case vtol_mode::TRANSITION_FRONT_P2:
		_vtol_mode = mode::TRANSITION_TO_FW;
		break;

	case vtol_mode::TRANSITION_BACK:
		_vtol_mode = mode::TRANSITION_TO_MC;
		break;
	}
}
int direct;
void Tiltrotor::update_mc_state()
{
	VtolType::update_mc_state();

	/*Motor spin up: define the first second after arming as motor spin up time, during which
	* the tilt is set to the value of VT_TILT_SPINUP. This allowes the user to set a spin up
	* tilt angle in case the propellers don't spin up smootly in full upright (MC mode) position.
	*/

	const int spin_up_duration_p1 = 1000_ms; // duration of 1st phase of spinup (at fixed tilt)
	const int spin_up_duration_p2 = 700_ms; // duration of 2nd phase of spinup (transition from spinup tilt to mc tilt)

	// reset this timestamp while disarmed
	if (!_v_control_mode->flag_armed) {
		_last_timestamp_disarmed = hrt_absolute_time();
		_tilt_motors_for_startup = _params_tiltrotor.tilt_spinup > 0.01f; // spinup phase only required if spinup tilt > 0

	} else if (_tilt_motors_for_startup) {
		// leave motors tilted forward after arming to allow them to spin up easier
		if (hrt_absolute_time() - _last_timestamp_disarmed > (spin_up_duration_p1 + spin_up_duration_p2)) {
			_tilt_motors_for_startup = false;
		}
	}

	if (_tilt_motors_for_startup) {
		if (hrt_absolute_time() - _last_timestamp_disarmed < spin_up_duration_p1) {
			_tilt_control = _params_tiltrotor.tilt_spinup;

		} else {
			// duration phase 2: begin to adapt tilt to multicopter tilt
			float delta_tilt = (_params_tiltrotor.tilt_mc - _params_tiltrotor.tilt_spinup);
			_tilt_control = _params_tiltrotor.tilt_spinup + delta_tilt / spin_up_duration_p2 * (hrt_absolute_time() -
					(_last_timestamp_disarmed + spin_up_duration_p1));
		}

		_mc_yaw_weight = 0.0f; //disable yaw control during spinup

	} else {
		// normal operation
		//_tilt_control = VtolType::pusher_assist();
		/*
		if (direct != -1 && direct != 1) {
			direct = -1;
			_tilt_control = 1.0f;
		}
		_tilt_control += 0.0001f*direct;
		if (_tilt_control > 0.85f){
			direct = -1;
		}
		if (_tilt_control < -1.01f){
			direct = 1;
		}*/

		_mc_yaw_weight = 1.0f;
		_v_att_sp->thrust_body[2] = Tiltrotor::thrust_compensation_for_tilt();
	}

}

void Tiltrotor::update_fw_state()
{
	VtolType::update_fw_state();

	// make sure motors are tilted forward
	_tilt_control = _params_tiltrotor.tilt_fw;
}

void Tiltrotor::update_transition_state()
{
	VtolType::update_transition_state();

	// copy virtual attitude setpoint to real attitude setpoint (we use multicopter att sp)
	memcpy(_v_att_sp, _mc_virtual_att_sp, sizeof(vehicle_attitude_setpoint_s));

	_v_att_sp->roll_body = _fw_virtual_att_sp->roll_body;

	float time_since_trans_start = (float)(hrt_absolute_time() - _vtol_schedule.transition_start) * 1e-6f;

	if (!_flag_was_in_trans_mode) {
		// save desired heading for transition and last thrust value
		_flag_was_in_trans_mode = true;
	}

	if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_FRONT_P1) {
		// for the first part of the transition all rotors are enabled
		if (_motor_state != motor_state::ENABLED) {
			_motor_state = set_motor_state(_motor_state, motor_state::ENABLED);
		}

		// tilt rotors forward up to certain angle
		if (_tilt_control <= _params_tiltrotor.tilt_transition) {
			_tilt_control = _params_tiltrotor.tilt_mc +
					fabsf(_params_tiltrotor.tilt_transition - _params_tiltrotor.tilt_mc) * time_since_trans_start /
					_params->front_trans_duration;
		}


		// at low speeds give full weight to MC
		_mc_roll_weight = 1.0f;
		_mc_yaw_weight = 1.0f;

		// reduce MC controls once the plane has picked up speed
		if (!_params->airspeed_disabled && PX4_ISFINITE(_airspeed_validated->equivalent_airspeed_m_s) &&
		    _airspeed_validated->equivalent_airspeed_m_s > ARSP_YAW_CTRL_DISABLE) {
			_mc_yaw_weight = 0.0f;
		}

		if (!_params->airspeed_disabled && PX4_ISFINITE(_airspeed_validated->equivalent_airspeed_m_s) &&
		    _airspeed_validated->equivalent_airspeed_m_s >= _params->airspeed_blend) {
			_mc_roll_weight = 1.0f - (_airspeed_validated->equivalent_airspeed_m_s - _params->airspeed_blend) /
					  (_params->transition_airspeed - _params->airspeed_blend);
		}

		// without airspeed do timed weight changes
		if ((_params->airspeed_disabled || !PX4_ISFINITE(_airspeed_validated->equivalent_airspeed_m_s)) &&
		    time_since_trans_start > _params->front_trans_time_min) {
			_mc_roll_weight = 1.0f - (time_since_trans_start - _params->front_trans_time_min) /
					  (_params->front_trans_time_openloop - _params->front_trans_time_min);
			_mc_yaw_weight = _mc_roll_weight;
		}

		_thrust_transition = -_mc_virtual_att_sp->thrust_body[2];

	} else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_FRONT_P2) {
		// the plane is ready to go into fixed wing mode, tilt the rotors forward completely
		_tilt_control = _params_tiltrotor.tilt_transition +
				fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_transition) * time_since_trans_start /
				_params_tiltrotor.front_trans_dur_p2;

		_mc_roll_weight = 0.0f;
		_mc_yaw_weight = 0.0f;

		// ramp down motors not used in fixed-wing flight (setting MAX_PWM down scales the given output into the new range)
		int ramp_down_value = (1.0f - time_since_trans_start / _params_tiltrotor.front_trans_dur_p2) *
				      (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN) + PWM_DEFAULT_MIN;


		_motor_state = set_motor_state(_motor_state, motor_state::VALUE, ramp_down_value);


		_thrust_transition = -_mc_virtual_att_sp->thrust_body[2];

	} else if (_vtol_schedule.flight_mode == vtol_mode::TRANSITION_BACK) {
		// turn on all MC motors
		if (_motor_state != motor_state::ENABLED) {
			_motor_state = set_motor_state(_motor_state, motor_state::ENABLED);
		}


		// set idle speed for rotary wing mode
		if (!flag_idle_mc) {
			flag_idle_mc = set_idle_mc();
		}

		// tilt rotors back
		if (_tilt_control > _params_tiltrotor.tilt_mc) {
			_tilt_control = _params_tiltrotor.tilt_fw -
					fabsf(_params_tiltrotor.tilt_fw - _params_tiltrotor.tilt_mc) * time_since_trans_start / 1.0f;
		}

		_mc_yaw_weight = 1.0f;

		// control backtransition deceleration using pitch.
		_v_att_sp->pitch_body = update_and_get_backtransition_pitch_sp();

		// while we quickly rotate back the motors keep throttle at idle
		if (time_since_trans_start < 1.0f) {
			_mc_throttle_weight = 0.0f;
			_mc_roll_weight = 0.0f;
			_mc_pitch_weight = 0.0f;

		} else {
			_mc_roll_weight = 1.0f;
			_mc_pitch_weight = 1.0f;
			// slowly ramp up throttle to avoid step inputs
			_mc_throttle_weight = (time_since_trans_start - 1.0f) / 1.0f;
		}
	}

	const Quatf q_sp(Eulerf(_v_att_sp->roll_body, _v_att_sp->pitch_body, _v_att_sp->yaw_body));
	q_sp.copyTo(_v_att_sp->q_d);


	_mc_roll_weight = math::constrain(_mc_roll_weight, 0.0f, 1.0f);
	_mc_yaw_weight = math::constrain(_mc_yaw_weight, 0.0f, 1.0f);
	_mc_throttle_weight = math::constrain(_mc_throttle_weight, 0.0f, 1.0f);


}

void Tiltrotor::waiting_on_tecs()
{
	// keep multicopter thrust until we get data from TECS
	_v_att_sp->thrust_body[0] = _thrust_transition;
}

/**
* Write data to actuator output topic.
*/
float _ch_tilt,deltaN,deltaNy;
float a_posc2[4]={0,0,0,0},real1,real2;
float a_pos_pre2; //第二级舵机的位置预测值
void Tiltrotor::fill_actuator_outputs()
{
	// Multirotor output
	_actuators_out_0->timestamp = hrt_absolute_time();
	_actuators_out_0->timestamp_sample = _actuators_mc_in->timestamp_sample;

	_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_ROLL] * _mc_roll_weight;
	_actuators_out_0->control[actuator_controls_s::INDEX_PITCH] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_PITCH] * _mc_pitch_weight;
	_actuators_out_0->control[actuator_controls_s::INDEX_YAW] =
		_actuators_mc_in->control[actuator_controls_s::INDEX_YAW] * _mc_yaw_weight;

	if (_vtol_schedule.flight_mode == vtol_mode::FW_MODE) {
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_THROTTLE];

		/* allow differential thrust if enabled */
		if (_params->diff_thrust == 1) {
			_actuators_out_0->control[actuator_controls_s::INDEX_ROLL] =
				_actuators_fw_in->control[actuator_controls_s::INDEX_YAW] * _params->diff_thrust_scale;
		}

	} else {
		_actuators_out_0->control[actuator_controls_s::INDEX_THROTTLE] =
			_actuators_mc_in->control[actuator_controls_s::INDEX_THROTTLE] * _mc_throttle_weight;
	}

	// Fixed wing output
	_actuators_out_1->timestamp = hrt_absolute_time();
	_actuators_out_1->timestamp_sample = _actuators_fw_in->timestamp_sample;

	struct rc_channels_s rc_channel;
	if (_rc_sub.update(&rc_channel)){
		//printf("thrust=%lf\n",(double)rc_channel.channels[rc_channel.RC_CHANNELS_FUNCTION_THROTTLE]);
		_ch_tilt = rc_channel.channels[rc_channel.RC_CHANNELS_FUNCTION_THROTTLE];
		deltaN = deltaN * 0.95f + 0.05f * rc_channel.channels[rc_channel.RC_CHANNELS_FUNCTION_THROTTLE];
		deltaNy= deltaNy* 0.95f + 0.05f * rc_channel.channels[rc_channel.RC_CHANNELS_FUNCTION_PITCH];
	}
	//CZQ:tilt_wing_debug
	//_tilt_control = (_ch_tilt-0.5f) * 2.9f;
	//CZQ:vector thrust
	struct ch_actuator_state_s _ch_actuator_state;
	if (_sub2.update(&_ch_actuator_state)) {
		if (_ch_actuator_state.num == 1)
			real1 = _ch_actuator_state.value;
		else real2 = _ch_actuator_state.value;
		printf("num=%d,value=%f\n",_ch_actuator_state.num,(double)_ch_actuator_state.value)
;	}

	float a_nozzle = 25.0f/57.3f;
	float a1_nozzle;
	float a2_nozzle;
	float Ky_nozzle;
	float Kn_nozzle;
	float  c1_nozzle;
	float  c2_nozzle;
	a_nozzle = 25.0f / 57.3f;
	Kn_nozzle = deltaN * 105.0f/57.3f;
	Ky_nozzle = (deltaNy-0.5f) * 2.0f * 15.0f / 57.3f + 0.2617f;
	//printf("Kn_nozzle=%f,Ky_nozzle=%f\n",(double)Kn_nozzle,(double)Ky_nozzle);
	//Ky_nozzle = (0.02*(rc_5_i) - 30.0 )/57.3;//遥控器5 1000~2000映射-10~10度 在换成弧度
	//	Kn_nozzle = (200.0 - 0.1*(rc_6_i) )/57.3;//遥控器6 1000~2000映射100~0度 在换成弧度
		//printf("rc_5=%d,rc_6=%d   ",rc_5.radio_out,rc_6.radio_out);
	if(Kn_nozzle>99.9999f/57.3f){
		Kn_nozzle=99.9999f/57.3f;
	}
	if(Kn_nozzle<0.0001f){
		Kn_nozzle=0.0001f;
	}


	a2_nozzle =  acos((cos(Kn_nozzle/2.0f)-cos(a_nozzle)*cos(a_nozzle))/(sin(a_nozzle)*sin(a_nozzle))) ;//二级喷管角度
	c2_nozzle = -(180.0f-a2_nozzle * 57.3f)*2.5f; //二级舵机输入角度值

	/*a_pos_pre2=0.75*chuart.sd[1].pos*0.15+0.25* a_posc2[3];
	a_pos_pre2=0.75*a_pos_pre2+0.25* a_posc2[2];
	a_pos_pre2=0.75*a_pos_pre2+0.25* a_posc2[1];
	a_pos_pre2=0.75*a_pos_pre2+0.25* a_posc2[0];


	if((c2_nozzle-a_pos_pre2) > 3.5){
		c2_nozzle = a_pos_pre2 + 3.5;
	}
	else if((c2_nozzle-a_pos_pre2) < -3.5){
		c2_nozzle = a_pos_pre2 -3.5;
	}*/
	if(c2_nozzle > 0.0f){
	    c2_nozzle = 0.0f;
	}
	else if(c2_nozzle < -450.0f){
	    c2_nozzle = -450.0f;
	}

	a_posc2[3]=a_posc2[2];
	a_posc2[2]=a_posc2[1];
	a_posc2[1]=a_posc2[0];
	a_posc2[0]=c2_nozzle;

	a2_nozzle= (180.0f+c2_nozzle/2.5f)/57.3f;

	float yaw_pwm = 0.0f;
	a1_nozzle =  atan(tan(a2_nozzle/2.0f)*cos(a_nozzle)) + Ky_nozzle  ;  	//一级喷管角度
	c1_nozzle =  -(-a1_nozzle * 57.3f-yaw_pwm*1.5f)*2.55f;//转换成一级舵机输入角度值
	c2_nozzle += 450.0f;
	c1_nozzle *= 1.944f;
	c2_nozzle *= 2.787f;
	ch_actuator_controls_s tmp_status;
	tmp_status.timestamp = hrt_absolute_time();
	tmp_status.ch_1 = c1_nozzle;
	tmp_status.ch_2 = c2_nozzle;
	tmp_status.num = 0;
	_ch_actuator_controls_pub.publish(tmp_status);

	//printf("_tilt_control = %lf",(double)_tilt_control);
	_actuators_out_1->control[4] = _tilt_control;

	if (_params->elevons_mc_lock && _vtol_schedule.flight_mode == vtol_mode::MC_MODE) {
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] = 0.0f;
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] = 0.0f;
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] = 0.0f;

	} else {
		_actuators_out_1->control[actuator_controls_s::INDEX_ROLL] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_ROLL];
		_actuators_out_1->control[actuator_controls_s::INDEX_PITCH] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_PITCH];
		_actuators_out_1->control[actuator_controls_s::INDEX_YAW] =
			_actuators_fw_in->control[actuator_controls_s::INDEX_YAW];
	}
}

/*
 * Increase combined thrust of MC propellers if motors are tilted. Assumes that all MC motors are tilted equally.
 */

float Tiltrotor::thrust_compensation_for_tilt()
{
	// only compensate for tilt angle up to 0.5 * max tilt
	float compensated_tilt = math::constrain(_tilt_control, 0.0f, 0.5f);

	// increase vertical thrust by 1/cos(tilt), limmit to [-1,0]
	return math::constrain(_v_att_sp->thrust_body[2] / cosf(compensated_tilt * M_PI_2_F), -1.0f, 0.0f);

}
