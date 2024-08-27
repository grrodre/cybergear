from enum import Enum, IntEnum


class CanMask(IntEnum):
    communication_type = 0xFF000000
    data_area_two = 0x00FFFF00
    target_address = 0x000000FF
    index_parameter_table = 0x00FF0000


class CommunicationTypeCan(IntEnum):
    """CAN communication type field (bits 28–24 of the extended frame ID).

    Types 0x00–0x07 and 0x11–0x12 are documented in the official datasheet.
    Types 0x13 and 0x14 were reverse-engineered by capturing traffic from the
    official Windows debugger application and are not in the datasheet.
    """

    device_id = 0x00000000
    control_instructions = 0x01000000
    motor_feedback = 0x02000000
    motor_enable = 0x03000000
    motor_stopped = 0x04000000
    zero_position = 0x06000000
    set_can_id = 0x07000000
    parameter_reading = 0x11000000
    parameter_writing = 0x12000000
    parameter_table = 0x13000000  # reverse-engineered
    emergency_stop = 0x14000000  # reverse-engineered
    encoder_calibration = (
        0x05000000  # reverse-engineered: trigger calibration / receive result
    )


class ParameterIndex(IntEnum):
    run_mode = 0x7005
    iq_ref = 0x7006
    spd_ref = 0x700A
    limit_torque = 0x700B
    cur_kp = 0x7010
    cur_ki = 0x7011
    cur_filt_gain = 0x7014
    loc_ref = 0x7016
    limit_spd = 0x7017
    limit_cur = 0x7018
    mech_pos = 0x7019
    iqf = 0x701A
    mech_vel = 0x701B
    v_bus = 0x701C
    rotation = 0x701D  # input shaft rotations (motor side, pre-gearbox)
    loc_kp = 0x701E
    spd_kp = 0x701F
    spd_ki = 0x7020


class RunMode(IntEnum):
    operation = 0
    position = 1
    speed = 2
    current = 3
    zero_position = 4
    quick_move = 7


class ParameterTable(int, Enum):
    parameter_type: str

    def __new__(cls, function_code, parameter_type):
        obj = int.__new__(cls, function_code)
        obj._value_ = function_code
        obj.parameter_type = parameter_type
        return obj

    motor_name = 0x00000, '6s'
    bar_code = 0x0001, '6s'
    boot_code_version = 0x1000, '6s'
    boot_build_date = 0x1001, '6s'
    boot_build_time = 0x1002, '6s'
    app_code_version = 0x1003, '6s'
    app_git_version = 0x1004, '6s'
    app_build_date = 0x1005, '6s'
    app_build_time = 0x1006, '6s'
    app_code_name = 0x1007, '6s'
    echo_para1 = 0x2000, 'H'
    echo_para2 = 0x2001, 'H'
    echo_para3 = 0x2002, 'H'
    echo_para4 = 0x2003, 'H'
    echo_fre_hz = 0x2004, 'I'
    mech_offset = 0x2005, 'f'
    mech_pos_init = 0x2006, 'f'
    limit_torque = 0x2007, 'f'
    i_fw_max = 0x2008, 'f'
    motor_index = 0x2009, 'B'
    can_id = 0x200A, 'B'
    can_master = 0x200B, 'B'
    can_timeout = 0x200C, 'I'
    motor_over_temp = 0x200D, 'h'
    over_temp_time = 0x200E, 'I'
    gear_ratio = 0x200F, 'f'
    tq_cali_type = 0x2010, 'B'
    cur_filt_gain = 0x2011, 'f'
    cur_kp = 0x2012, 'f'
    cur_ki = 0x2013, 'f'
    spd_kp = 0x2014, 'f'
    spd_ki = 0x2015, 'f'
    loc_kp = 0x2016, 'f'
    spd_filt_gain = 0x2017, 'f'
    limit_spd = 0x2018, 'f'
    limit_cur = 0x2019, 'f'
    time_use0 = 0x3000, 'H'
    time_use1 = 0x3001, 'H'
    time_use2 = 0x3002, 'H'
    time_use3 = 0x3003, 'H'
    encoder_raw = 0x3004, 'h'
    mcu_temp = 0x3005, 'h'
    motor_temp = 0x3006, 'h'
    vbus_mv = 0x3007, 'H'
    adc1_offset = 0x3008, 'i'
    adc2_offset = 0x3009, 'i'
    adc1_raw = 0x300A, 'H'
    adc2_raw = 0x300B, 'H'
    vbus = 0x300C, 'f'
    cmd_id = 0x300D, 'f'
    cmd_iq = 0x300E, 'f'
    cmd_loc_ref = 0x300F, 'f'
    cmd_spd_ref = 0x3010, 'f'
    cmd_torque = 0x3011, 'f'
    cmd_pos = 0x3012, 'f'
    cmd_vel = 0x3013, 'f'
    rotation = 0x3014, 'h'
    mod_pos = 0x3015, 'f'
    mech_pos = 0x3016, 'f'
    mech_vel = 0x3017, 'f'
    elec_pos = 0x3018, 'f'
    ia = 0x3019, 'f'
    ib = 0x301A, 'f'
    ic = 0x301B, 'f'
    tick = 0x301C, 'I'
    phase_order = 0x301D, 'B'
    iqf = 0x301E, 'f'
    board_temp = 0x301F, 'h'
    iq = 0x3020, 'f'
    id = 0x3021, 'f'
    fault_sta = 0x3022, 'I'
    warn_sta = 0x3023, 'I'
    drv_fault = 0x3024, 'H'
    drv_temp = 0x3025, 'h'
    uq = 0x3026, 'f'
    ud = 0x3027, 'f'
    dtc_u = 0x3028, 'f'
    dtc_v = 0x3029, 'f'
    dtc_w = 0x302A, 'f'
    v_bus = 0x302B, 'f'
    elec_offset = 0x302C, 'f'
    torque_fdb = 0x302D, 'f'
    rated_i = 0x302E, 'f'
    limit_i = 0x302F, 'f'
    end = 0x404, '6s'
