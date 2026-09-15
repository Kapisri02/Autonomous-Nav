"""Risk-aware autonomous navigation for the BeetleBot Lyra.

Layout
------
``beetlebot_risk_nav.core``
    The complete algorithmic pipeline. Pure Python (stdlib only), no ROS and no
    NumPy, so it can be unit-tested anywhere and runs unchanged on the robot.
``beetlebot_risk_nav.sim``
    A headless 2D simulator used for integration, regression and evaluation runs.
``beetlebot_risk_nav.nodes``
    Thin ROS 2 adapters that connect ``core`` to the robot's topics.
``beetlebot_risk_nav.baseline``
    A faithful re-implementation of the original reactive controller, kept for
    side-by-side comparison. The original ROS script is preserved untouched in
    ``baseline/lyra_control/obstacle_avoidance.py`` at the repository root.
"""

__version__ = '1.0.0'
