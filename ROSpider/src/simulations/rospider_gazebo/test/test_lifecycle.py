from rospider_gazebo.lifecycle import bring_up


class FakeNode:
    """A lifecycle node whose replies can be lost: the transition happens,
    the caller just never hears back -- what rmw's 'failed to send response'
    does to nav2's lifecycle_manager."""

    def __init__(self, state='unconfigured', lose_replies=True):
        self.state = state
        self.lose_replies = lose_replies
        self.calls = []

    def get_state(self):
        return self.state

    def change_state(self, transition):
        self.calls.append(transition)
        self.state = {'configure': 'inactive', 'activate': 'active'}[transition]
        return None if self.lose_replies else True


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


def test_lost_replies_still_end_active():
    node, clock = FakeNode(), Clock()
    assert bring_up(node.get_state, node.change_state, 10.0, clock, clock.sleep)
    assert node.state == 'active'
    assert node.calls == ['configure', 'activate']


def test_already_active_changes_nothing():
    node, clock = FakeNode('active'), Clock()
    assert bring_up(node.get_state, node.change_state, 10.0, clock, clock.sleep)
    assert node.calls == []


def test_no_answer_keeps_asking_until_timeout():
    clock = Clock()
    calls = []
    assert not bring_up(lambda: None, calls.append, 5.0, clock, clock.sleep)
    assert calls == []
    assert clock.now >= 5.0


def test_mid_transition_waits_instead_of_calling():
    clock = Clock()
    states = iter(['configuring', 'configuring', 'inactive', 'active'])
    calls = []
    assert bring_up(lambda: next(states), calls.append, 10.0, clock, clock.sleep)
    assert calls == ['activate']


from rospider_gazebo.lifecycle import bring_up_all  # noqa: E402


def _nodes(**states):
    return {name: FakeNode(state) for name, state in states.items()}


def test_all_nodes_come_up_in_order_despite_lost_replies():
    nodes = _nodes(controller_server='unconfigured', planner_server='unconfigured',
                   bt_navigator='unconfigured')
    order = []

    def change(name, transition):
        order.append((name, transition))
        nodes[name].change_state(transition)

    clock = Clock()
    left = bring_up_all(list(nodes), lambda n: nodes[n].get_state(), change,
                        clock=clock, sleep=clock.sleep)
    assert left == []
    assert [n for n, _ in order] == (['controller_server'] * 2 + ['planner_server'] * 2
                                     + ['bt_navigator'] * 2)


def test_a_node_that_never_answers_is_reported_and_blocks_the_rest():
    nodes = _nodes(controller_server='unconfigured', planner_server='unconfigured')
    clock = Clock()
    left = bring_up_all(['controller_server', 'ghost', 'planner_server'],
                        lambda n: nodes[n].get_state() if n in nodes else None,
                        lambda n, t: nodes[n].change_state(t), timeout=30.0,
                        clock=clock, sleep=clock.sleep)
    assert left == ['ghost', 'planner_server']
    assert nodes['planner_server'].state == 'unconfigured'   # order kept: never started early
