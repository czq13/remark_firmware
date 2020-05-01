#!/bin/bash
paththisfolder=/home/czq/remark_firmware
pathfirmware=/home/czq/new_version/Firmware
cp $pathfirmware/ROMFS/px4fmu_common/init.d/rc.vtol_defaults $paththisfolder
cp $pathfirmware/ROMFS/px4fmu_common/mixers/firefly6.aux.mix $paththisfolder
cp $pathfirmware/ROMFS/px4fmu_common/mixers/firefly6.main.mix $paththisfolder
cp $pathfirmware/msg/CMakeLists.txt $paththisfolder
cp $pathfirmware/src/drivers/drv_pwm_output.h $paththisfolder
cp $pathfirmware/src/modules/mavlink/mavlink_main.cpp $paththisfolder
cp $pathfirmware/src/modules/mavlink/mavlink_messages.cpp $paththisfolder
cp $pathfirmware/src/modules/mavlink/mavlink_receiver.cpp $paththisfolder
cp $pathfirmware/src/modules/mavlink/mavlink_receiver.h $paththisfolder
cp $pathfirmware/src/modules/vtol_att_control/tiltrotor.cpp $paththisfolder
cp $pathfirmware/src/modules/vtol_att_control/tiltrotor.h $paththisfolder
cp $pathfirmware/src/modules/vtol_att_control/tiltrotor_params.c $paththisfolder
cp $pathfirmware/msg/ch_actuator_controls.msg $paththisfolder
cp $pathfirmware/msg/ch_actuator_state.msg $paththisfolder

