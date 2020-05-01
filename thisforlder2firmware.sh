paththisfolder=/home/czq/remark_firmware
pathfirmware=/home/czq/new_version/Firmware
cp $paththisfolder/rc.vtol_defaults $pathfirmware/ROMFS/px4fmu_common/init.d/
cp $paththisfolder/firefly6.aux.mix $pathfirmware/ROMFS/px4fmu_common/mixers/
cp $paththisfolder/firefly6.main.mix $pathfirmware/ROMFS/px4fmu_common/mixers/
cp $paththisfolder/CMakeLists.txt $pathfirmware/msg/
cp $paththisfolder/drv_pwm_output.h $pathfirmware/src/drivers/
cp $paththisfolder/mavlink_main.cpp $pathfirmware/src/modules/mavlink/
cp $paththisfolder/mavlink_messages.cpp $pathfirmware/src/modules/mavlink/
cp $paththisfolder/mavlink_receiver.cpp $pathfirmware/src/modules/mavlink/
cp $paththisfolder/mavlink_receiver.h $pathfirmware/src/modules/mavlink/
cp $paththisfolder/tiltrotor.cpp $pathfirmware/src/modules/vtol_att_control/
cp $paththisfolder/tiltrotor_params.c $pathfirmware/src/modules/vtol_att_control/
cp $paththisfolder/tiltrotor.h $pathfirmware/src/modules/vtol_att_control/
cp $paththisfolder/ch_actuator_controls.msg $pathfirmware/msg/
cp $paththisfolder/ch_actuator_state.msg $pathfirmware/msg/

