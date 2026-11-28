"""Mission-owned B1 marker detection gate (pure Python, no ROS dependencies).

Placement and retrieval may enable detection only on their respective D_1 leg.
The caller must supply fresh, run-owned navigation evidence for retrieval;
the phase name alone does not prove that a goal was accepted or is moving.
"""


def placement_leg(session, operation):
    """The pallet node's PICK operation remains current until DROP starts."""
    return f'placement:{session}:{operation}' if session and operation >= 0 else ''


class ArucoRouteGate:
    @staticmethod
    def combine(supervisor_state, placement_grant, placement_fresh,
                retrieval_status, retrieval_grant, retrieval_fresh):
        """Resolve two grants without allowing simultaneous mission ownership."""
        retrieval_active = (isinstance(retrieval_status, dict)
                            and retrieval_status.get('state') not in
                            (None, 'IDLE', 'DONE', 'FAILED', 'STOPPED', 'CANCELED')
                            and not retrieval_status.get('discarded'))
        placement_active = supervisor_state not in (
            'IDLE', 'DONE', 'FAILED', 'STOPPED', 'RECOVERY', 'STOPPING')
        if retrieval_active and placement_active:
            return False
        return bool((supervisor_state == 'GOTO_D' and placement_fresh and placement_grant)
                    or (supervisor_state in ('IDLE', 'DONE', 'FAILED', 'STOPPED')
                        and retrieval_fresh and retrieval_grant))

    def __init__(self):
        self.leg_armed = False
        self.enabled = False
        self.owner = None
        self.retrieval_run_id = None
        self.retrieval_arrived_d1 = False

    def transition(self, previous, current, cargo):
        if self.owner != 'placement':
            self.leg_armed = False
        self.owner = 'placement'
        self.retrieval_run_id = None
        self.retrieval_arrived_d1 = False
        if previous == 'PICKING' and current == 'GOTO_D' and cargo is True:
            self.leg_armed = True
        elif current in (
            'IDLE', 'GOTO_PICK', 'PICKING', 'WAITING_SLOT', 'DROPPING',
            'RETURN_HOME', 'DONE', 'FAILED', 'STOPPED', 'CANCELING',
        ):
            self.leg_armed = False
        elif current not in ('GOTO_D', 'RECOVERY', 'STOPPING'):
            self.leg_armed = False
        # RECOVERY/STOPPING temporarily disables detection. If the operator
        # resumes GOTO_D for the same pick, the leg can be re-enabled.
        self.enabled = bool(self.leg_armed and current == 'GOTO_D' and cargo is True)
        return self.enabled

    def retrieval_transition(self, status, *, nav_goal_active=False, fresh=True,
                             arrived_d1=False):
        """Apply one retrieval snapshot and owned Nav2 goal observation.

        ``nav_goal_active`` must describe this retrieval's D_1 goal, not any
        unrelated Nav2 goal. A ``nav_leg='slot'`` snapshot is an arrival event:
        the retrieval worker emits it only after its centered D_1 helper ends.
        Explicit ``arrived_d1`` may be supplied from independently confirmed
        arrival. Missing/stale data always switches detection off.
        """
        self.enabled = False
        self.leg_armed = False
        self.owner = 'retrieval'
        if not fresh or not isinstance(status, dict):
            return False
        run_id = status.get('run_id')
        state = status.get('state')
        leg = status.get('nav_leg')
        if not isinstance(run_id, str) or not run_id or not isinstance(state, str):
            return False
        if run_id != self.retrieval_run_id:
            self.retrieval_run_id = run_id
            self.retrieval_arrived_d1 = False
        if arrived_d1 is True or state == 'NAVIGATING_TO_D1' and leg == 'slot':
            self.retrieval_arrived_d1 = True
        if state != 'NAVIGATING_TO_D1' or leg != 'd1':
            return False
        if self.retrieval_arrived_d1 or nav_goal_active is not True:
            return False
        self.enabled = True
        return True

    def arrived_d1(self):
        self.leg_armed = False
        self.enabled = False
        if self.owner == 'retrieval':
            self.retrieval_arrived_d1 = True

    def reset(self):
        self.arrived_d1()
        self.owner = None
        self.retrieval_run_id = None
        self.retrieval_arrived_d1 = False
