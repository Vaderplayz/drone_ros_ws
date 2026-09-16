from mini_ground_control.models.landing_state import LandingPhase, LandingStateMachine
import pytest


def test_nominal_landing_transitions() -> None:
    machine = LandingStateMachine()
    for phase in (
        LandingPhase.ACTIVATING,
        LandingPhase.SEARCHING,
        LandingPhase.TARGET_DETECTED,
        LandingPhase.ALIGNING,
        LandingPhase.CENTERED,
        LandingPhase.DESCENDING,
        LandingPhase.LANDING,
        LandingPhase.LANDED,
        LandingPhase.IDLE,
    ):
        machine.transition(phase)
    assert machine.state.phase == LandingPhase.IDLE


def test_invalid_landing_transition_is_rejected() -> None:
    machine = LandingStateMachine()
    with pytest.raises(ValueError):
        machine.transition(LandingPhase.DESCENDING)


@pytest.mark.parametrize(
    ("text", "expected"),
    [
        ("WAIT_INPUT mode=OFFBOARD", LandingPhase.SEARCHING),
        ("APPROACH dx=0.2", LandingPhase.ALIGNING),
        ("DESCEND alt=1.0", LandingPhase.DESCENDING),
        ("AUTO_LAND", LandingPhase.LANDING),
    ],
)
def test_existing_controller_state_mapping(text: str, expected: LandingPhase) -> None:
    assert LandingStateMachine.from_status_text(text) == expected
