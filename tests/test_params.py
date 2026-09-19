"""Unit tests for configuration handling and validation."""

import pytest

from beetlebot_risk_nav.core.params import NavConfig, RobotConfig, flatten


def test_default_config_is_valid():
    assert NavConfig().validate() == []


def test_verified_robot_dimensions_are_the_documented_ones():
    """These come from the robot's nav2_params.yaml and VEEROBOT documentation.

    They are not tuning knobs: if someone changes them, the collision checking
    silently stops matching the real chassis, so the values are pinned here.
    """
    robot = RobotConfig()
    assert robot.footprint_length == pytest.approx(0.375)
    assert robot.footprint_width == pytest.approx(0.360)
    assert robot.robot_radius == pytest.approx(0.22)


def test_flat_round_trip_preserves_values():
    cfg = NavConfig()
    flat = cfg.to_flat()
    rebuilt = NavConfig.from_flat(flat)
    assert rebuilt.to_flat() == flat


def test_apply_flat_coerces_types():
    cfg = NavConfig.from_flat({
        'robot.max_linear_speed': 0.5,
        'safety.require_odom': 'false',
        'lidar.median_window': 5.0,
        'logging.directory': 'somewhere',
    })
    assert cfg.robot.max_linear_speed == pytest.approx(0.5)
    assert cfg.safety.require_odom is False
    assert isinstance(cfg.lidar.median_window, int) and cfg.lidar.median_window == 5
    assert cfg.logging.directory == 'somewhere'


def test_unknown_keys_are_reported_not_silently_ignored():
    cfg = NavConfig()
    unknown = cfg.apply_flat({'robot.nonexistent': 1, 'nosuchsection.x': 2, 'bare': 3})
    assert set(unknown) == {'robot.nonexistent', 'nosuchsection.x', 'bare'}


def test_nested_yaml_style_parameters_are_flattened():
    assert flatten({'robot': {'max_linear_speed': 0.3}}) == {'robot.max_linear_speed': 0.3}
    cfg = NavConfig()
    cfg.apply_nested({'robot': {'max_linear_speed': 0.3}})
    assert cfg.robot.max_linear_speed == pytest.approx(0.3)


@pytest.mark.parametrize('overrides,expected_fragment', [
    ({'risk.critical_distance': 2.0}, 'risk distances'),
    ({'risk.critical_ttc': 99.0}, 'risk TTC'),
    ({'lidar.median_window': 4}, 'median_window'),
    ({'robot.footprint_min_inflation': 0.5}, 'min_inflation'),
    ({'safety.max_linear_speed': 0.05}, 'supervisor'),
    ({'velocity.nominal_speed': 5.0}, 'nominal_speed'),
    ({'goal.xy_tolerance': 0.0}, 'xy_tolerance'),
    ({'safety.estop_clear_distance': 0.01}, 'estop_clear_distance'),
])
def test_validate_catches_dangerous_configurations(overrides, expected_fragment):
    problems = NavConfig.from_flat(overrides).validate()
    assert any(expected_fragment in p for p in problems), problems


def test_circular_footprint_uses_nav2_radius():
    cfg = NavConfig.from_flat({'robot.use_circular_footprint': True})
    expected = cfg.robot.robot_radius + cfg.robot.footprint_inflation
    assert cfg.robot.circumscribed_radius == pytest.approx(expected)
    assert cfg.robot.inscribed_radius == pytest.approx(expected)


def test_packaged_yaml_matches_the_dataclass_defaults():
    """The shipped YAML and NavConfig must agree, key for key and value for value.

    The YAML's own header says it is generated from NavConfig "so the two cannot
    drift apart", and the README repeats the claim - but nothing enforced it, and
    they had drifted: four keys were missing from the file, one of them
    ``lidar.mount_offset_x``, the LiDAR mounting offset. A parameter absent from
    the file cannot be found or changed by someone reading it, which for a
    physical-mounting value is a safety-relevant omission rather than a typo.
    """
    import os
    import yaml

    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    path = os.path.join(here, 'src', 'beetlebot_risk_nav', 'config',
                        'beetlebot_nav.yaml')
    with open(path) as handle:
        loaded = yaml.safe_load(handle)['beetlebot_nav']['ros__parameters']

    # The node declares these separately from NavConfig; see the node's
    # __init__. Everything else in the file must be a NavConfig field.
    interface_keys = {
        'scan_topic', 'odom_topic', 'goal_topic', 'cmd_vel_topic',
        'global_frame', 'base_frame', 'odom_frame', 'control_frequency',
        'use_tf', 'publish_status', 'use_sim_time',
    }
    from_file = {key: value for key, value in flatten(loaded).items()
                 if key not in interface_keys}
    defaults = NavConfig().to_flat()

    assert set(from_file) == set(defaults), (
        'missing from the YAML: {}; unknown in the YAML: {}'.format(
            sorted(set(defaults) - set(from_file)),
            sorted(set(from_file) - set(defaults))))
    for key, value in defaults.items():
        assert from_file[key] == pytest.approx(value) if isinstance(value, float) \
            else from_file[key] == value, key
