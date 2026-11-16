"""Explicit velocity-takeover source geometry; independent of position IK options."""
import math

def resolve_velocity_tcp_geometry(model_config, robot_version, explicit_length=None, geometry_id=''):
    if explicit_length is None:
        explicit_length = model_config.get('teleop_tcp_forearm_length')
        geometry_id = geometry_id or model_config.get('teleop_tcp_geometry_id', '')
    if explicit_length is None or not geometry_id:
        raise ValueError('Velocity takeover requires explicit forearm length AND geometry identity; legacy lower_arm_length is not a calibration')
    value = float(explicit_length)
    if not math.isfinite(value) or value <= 0:
        raise ValueError('Velocity TCP forearm length must be finite and positive')
    return {'forearm_retarget_length': value, 'geometry_id': str(geometry_id),
            'robot_version': str(robot_version), 'source_segment': 'ArmLower origin -> HandPalm origin',
            'tcp_position': 'mapped Palm + rotated source_to_tcp translation',
            'calibration_status': 'EXPLICIT_CONFIGURATION_NOT_HARDWARE_CALIBRATION'}
