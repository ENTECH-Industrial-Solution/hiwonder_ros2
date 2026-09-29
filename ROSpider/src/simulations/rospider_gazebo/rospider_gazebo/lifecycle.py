"""Bring a lifecycle node up to 'active' by polling its state.

nav2's lifecycle_manager sends configure/activate and waits for the reply
with no timeout. On Jazzy the reply is sometimes lost (rmw logs "failed to
send response ... client will not receive response"): slam_toolbox reaches
'inactive', the manager never hears so, and /map never appears. Here every
decision comes from asking the node its state again, so a lost reply costs
one poll, not the run. The same holds for Nav2's servers (smoother_server
lost a change_state reply in ~1 of 12 V-SLAM launches), so bring_up_all
does them in order. No rclpy in this module (scripts/lifecycle_activate.py
wraps it), so it is unit tested.
"""

import time

#: The transition that moves each stable state one step towards 'active'.
NEXT = {'unconfigured': 'configure', 'inactive': 'activate'}


def bring_up(get_state, change_state, timeout=60.0, clock=time.monotonic,
             sleep=time.sleep, period=0.5):
    """Poll until the node is active; True if it got there within `timeout`.

    get_state() returns the state label, or None when the node did not
    answer. change_state(transition) requests one transition; its return
    value is ignored, because the reply is the part that gets lost.
    """
    deadline = clock() + timeout
    while clock() < deadline:
        state = get_state()
        if state == 'active':
            return True
        if state in NEXT:
            change_state(NEXT[state])
        sleep(period)
    return False


def bring_up_all(names, get_state, change_state, timeout=120.0, clock=time.monotonic,
                 sleep=time.sleep, period=0.5):
    """bring_up each node in `names` in order, sharing one deadline, as nav2's
    lifecycle_manager does (a later node may need an earlier one active). Returns the
    names not active at the end: the one that timed out and every one after it."""
    deadline = clock() + timeout
    for index, name in enumerate(names):
        left = deadline - clock()
        if left <= 0 or not bring_up(lambda: get_state(name),
                                     lambda transition: change_state(name, transition),
                                     left, clock, sleep, period):
            return list(names[index:])
    return []
