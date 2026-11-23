from uwb_imu_sandbox.serial_format import parse_uwb_line


def test_parses_a_simulated_line():
    r = parse_uwb_line('[4,3,6.57826,23.6594,29.3795,8.9004,18.9127]')
    assert r is not None
    assert r.network_count == 4 and r.anchor_id == 3
    assert abs(r.range_m - 6.57826) < 1e-12
    assert abs(r.delta - 18.9127) < 1e-12


def test_parses_a_real_tag_line():
    r = parse_uwb_line(' [2,1,5.39276,-76.3846,-87.5775,186.1304,11.1930]\n')
    assert r is not None and r.anchor_id == 1 and r.rx < 0


def test_rejects_bad_lines():
    for bad in ['', 'SYS_MSG: Tag ready', '[1,2,3]', '[1,2,x,4,5,6,7]', '[1,2,nan,4,5,6,7]',
                '[1,2,-1.0,4,5,6,7]', '1,2,3,4,5,6,7', None, '[1,2,inf,4,5,6,7]']:
        assert parse_uwb_line(bad) is None
