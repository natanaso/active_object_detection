import roslib; roslib.load_manifest('pr2_plan_utils')

def test_arm_nav_error():
    from pr2_plan_utils.exceptions import ArmNavError
    msg = 'some_msg'
    error_code = 32
    trajectory_error_codes = [22, 11]
    try:
        raise ArmNavError(msg, error_code, trajectory_error_codes)
    except ArmNavError as e:
        assert e.msg == msg
        assert e.error_code == error_code
        assert e.trajectory_error_codes == trajectory_error_codes
        
