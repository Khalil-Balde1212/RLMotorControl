from serial_parser import parse_data, SIData


def test_parse_valid_line():
    # Create a valid line: 16 ones for matrix, 4 twos for vector, conv_rate=0.5, error=0.1, converged=1
    nums = ['1.0'] * 16 + ['2.0'] * 4 + ['0.5', '0.1', '1']
    line = ','.join(nums)
    data = parse_data(line)
    assert isinstance(data, SIData)
    assert data.matrix.shape == (4, 4)
    assert len(data.vector) == 4
    assert abs(data.conv_rate - 0.5) < 1e-9
    assert abs(data.curr_error - 0.1) < 1e-9
    assert data.converged is True


def test_parse_si_prefix():
    from serial_parser import parse_message
    nums = ['1.0'] * 16 + ['2.0'] * 4 + ['0.5', '0.1', '1']
    line = 'SI,' + ','.join(nums)
    msg_id, data = parse_message(line)
    assert msg_id == 'SI'
    assert data is not None
    assert data.matrix.shape == (4,4)
    # also accept extra trailing tokens
    line_extra = 'SI,' + ','.join(nums) + ',EXTRA,NOISE'
    msg_id, data = parse_message(line_extra)
    assert msg_id == 'SI'
    assert data is not None


def test_parse_sd_prefix():
    from serial_parser import parse_message
    # Old 6-value SD format: pos, vel, acc, jerk, current, motorCommand
    vals = ['10.0', '0.5', '0.01', '1.2', '50', '2.3']
    line = 'SD,' + ','.join(vals)
    msg_id, data = parse_message(line)
    assert msg_id == 'SD'
    assert data is not None
    assert data.pos == float(vals[0])
    assert data.motor_command == float(vals[-1])


def test_parse_sd_with_setpoint():
    from serial_parser import parse_message
    # New 7-value format: pos, setpoint, vel, acc, jerk, current, motor_cmd
    vals = ['-2.90', '1.57', '-0.01', '0.97', '7.26', '-2.24', '0.00']
    line = 'SD,' + ','.join(vals)
    msg_id, data = parse_message(line)
    assert msg_id == 'SD'
    assert data is not None
    assert data.pos == float(vals[0])
    assert data.setpoint == float(vals[1])
    assert data.motor_command == float(vals[-1])


def test_parse_multiple_prefixes():
    from serial_parser import parse_all_messages
    # line that contains both SD and SI sections
    sd = 'SD,0.1,0.2,0.3,0.4,0.5,0.6'
    si_nums = ['1.0'] * 16 + ['2.0'] * 4 + ['0.5', '0.1', '1']
    si = 'SI,' + ','.join(si_nums)
    line = sd + ',' + si
    results = parse_all_messages(line)
    assert any(m == 'SD' for m, _ in results)
    assert any(m == 'SI' for m, _ in results)


def test_parse_si_in_middle():
    from serial_parser import parse_message
    # 16 ones for matrix, 4 twos for vector, then SI plus conv_rate, curr_err, converged
    nums = ['1.0'] * 16 + ['2.0'] * 4
    line = ','.join(nums) + ',SI,0.180430,0.672270,0'
    msg_id, data = parse_message(line)
    assert msg_id == 'SI'
    assert data is not None
    assert isinstance(data.matrix, type(data.matrix))
    assert data.conv_rate == 0.18043
