"""Score the same physical flags used by all three entry points."""
from types import SimpleNamespace
import pytest
from utils.task_scorer import Task1Scorer, Task3Scorer
from utils.episode_success import _task1_complete, _task3_complete

@pytest.mark.parametrize("cls,names,base,rate,cap", [
    (Task1Scorer, Task1Scorer.CYLINDERS, 50, .5, 10),
    (Task3Scorer, Task3Scorer.RINGS, 45, 1, 20)])
def test_physical_scores_and_time(tmp_path, cls, names, base, rate, cap):
    tap = SimpleNamespace(task2_state={name:8 for name in names})
    scorer = cls(tap, str(tmp_path/"score.txt"), metadata={})
    assert scorer._evaluate().get("grasped", []) == []
    scorer.start_time = 100
    for index,name in enumerate(names,1):
        tap.task2_state[name] = 1
        state = scorer._evaluate()
        assert scorer.final_score(state,100)[1]["grasp"] == index*(10 if cls is Task1Scorer else 5)
        tap.task2_state[name] = 3  # above != placed
        state = scorer._evaluate()
        assert scorer.final_score(state,100)[1]["place"] == (index-1)*(15 if cls is Task1Scorer else 10)
        tap.task2_state[name] = 7
        state = scorer._evaluate()
    if cls is Task1Scorer:
        assert not _task1_complete(scorer,state)
        assert scorer.final_score(state,100)[0] == 75
        tap.task2_state["source_bin"] = 16
        state = scorer._evaluate()
        assert _task1_complete(scorer,state)
    else:
        assert _task3_complete(scorer,state)
        assert state["base_visits"] == 3
    assert scorer.final_score(state,99.5+base)[0] == 100
    assert scorer.final_score(state,100+base)[0] == 100
    assert scorer.final_score(state,100.5+base)[0] == 100-.5*rate
    assert scorer.final_score(state,102+base)[0] == 100-2*rate
    assert scorer.final_score(state,1000)[0] == 100-cap
    tap.task2_state.clear()
    assert scorer.final_score(scorer._evaluate(),100)[0] == 100

def test_no_score_without_observed_source_support(tmp_path):
    tap=SimpleNamespace(task2_state={name:1 for name in Task3Scorer.RINGS})
    scorer=Task3Scorer(tap,str(tmp_path/"score.txt"),metadata={})
    scorer.start_time=100
    assert scorer.final_score(scorer._evaluate(),100)[1]["grasp"]==0
