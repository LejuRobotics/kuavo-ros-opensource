from types import SimpleNamespace
from utils.task_scorer import Task1Scorer
from utils.episode_success import _task1_complete

def test_blue_bin_only_scores_after_settling(tmp_path):
    tap=SimpleNamespace(task2_state={})
    scorer=Task1Scorer(tap,str(tmp_path/"score.txt"),metadata={})
    scorer.start_time=1
    scorer.ever_placed.update(scorer.CYLINDERS)
    state=scorer._evaluate()
    assert scorer.final_score(state,1)[1]["lever"]==0
    assert not _task1_complete(scorer,state)
    tap.task2_state["source_bin"]=16
    state=scorer._evaluate()
    assert scorer.final_score(state,1)[1]["lever"]==25
    assert _task1_complete(scorer,state)
