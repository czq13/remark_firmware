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
 * @file tiltrotor_params.c
 * Parameters for vtol attitude controller.
 *
 * @author Roman Bapst <roman@px4.io>
 */

/**
 * Position of tilt servo in mc mode
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_MC, 0.0f);

/**
 * Position of tilt servo in transition mode
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_TRANS, 0.3f);

/**
 * Position of tilt servo in fw mode
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_FW, 1.0f);

/**
 * Duration of front transition phase 2
 *
 * Time in seconds it should take for the rotors to rotate forward completely from the point
 * when the plane has picked up enough airspeed and is ready to go into fixed wind mode.
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_P2_DUR, 0.5f);

/**
 * Tilt_Tail
 *
 * Time in seconds it should take for the rotors to rotate forward completely from the point
 * when the plane has picked up enough airspeed and is ready to go into fixed wind mode.
 *
 * @unit s
 * @min -10.0
 * @max 10.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_TAIL, 0.5f);

/**
 * Tilt_Wing
 *
 * Time in seconds it should take for the rotors to rotate forward completely from the point
 * when the plane has picked up enough airspeed and is ready to go into fixed wind mode.
 *
 * @unit s
 * @min -10.0
 * @max 10.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_WING_L, 0.5f);

PARAM_DEFINE_FLOAT(VT_TILT_WING_R, 0.5f);

PARAM_DEFINE_FLOAT(VT_TILT_WING_TL, 0.5f);

PARAM_DEFINE_FLOAT(VT_TILT_TAIL_TL, 0.5f);

/**
 * ROTOR_THRUST
 *
 * Time in seconds it should take for the rotors to rotate forward completely from the point
 * when the plane has picked up enough airspeed and is ready to go into fixed wind mode.
 *
 * @unit s
 * @min 0
 * @max 2.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_THRUST, 0.5f);

PARAM_DEFINE_FLOAT(VT_TILT_THRUST_T, 0.5f);

/**
 * ROTOR_THRUST
 *
 * Time in seconds it should take for the rotors to rotate forward completely from the point
 * when the plane has picked up enough airspeed and is ready to go into fixed wind mode.
 *
 * @unit s
 * @min 0
 * @max 2.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */

PARAM_DEFINE_FLOAT(VT_TILT_MODE, 0.0f);

PARAM_DEFINE_FLOAT(VT_TILT_TAIL_0, -1.0f);
PARAM_DEFINE_FLOAT(VT_TILT_TAIL_1, 0.0f);
PARAM_DEFINE_FLOAT(VT_TILT_TAIL_2, 0.0f);
PARAM_DEFINE_FLOAT(VT_TILT_TAIL_3, 0.0f);
PARAM_DEFINE_FLOAT(VT_TILT_TAIL_4, 0.0f);
PARAM_DEFINE_FLOAT(VT_TILT_TAIL_5, 0.0f);
PARAM_DEFINE_FLOAT(VT_TILT_TAIL_6, 1.0f);

PARAM_DEFINE_FLOAT(VT_TILT_WING_0, -1.0f);
PARAM_DEFINE_FLOAT(VT_TILT_WING_1, 0.0f);
PARAM_DEFINE_FLOAT(VT_TILT_WING_2, 0.0f);
PARAM_DEFINE_FLOAT(VT_TILT_WING_3, 0.0f);
PARAM_DEFINE_FLOAT(VT_TILT_WING_4, 0.0f);
PARAM_DEFINE_FLOAT(VT_TILT_WING_5, 0.0f);
PARAM_DEFINE_FLOAT(VT_TILT_WING_6, 1.0f);
/**
 * ROTOR_aileron
 *
 * Time in seconds it should take for the rotors to rotate forward completely from the point
 * when the plane has picked up enough airspeed and is ready to go into fixed wind mode.
 *
 * @unit s
 * @min 0
 * @max 2.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TILT_AILERON, 0.5f);
