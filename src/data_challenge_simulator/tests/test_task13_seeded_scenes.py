"""Generated XML and action plans agree without a saved layout catalogue."""
from dataclasses import asdict
import math
import xml.etree.ElementTree as ET
import pytest
from utils.task1_v2_randomization import Task1V2RandomizationPlanner
from utils.task3_randomization import Task3RandomizationPlanner
from utils.task2_randomization import task_scene, lighting_for_seed, seed_for_round

@pytest.mark.parametrize("task,planner", [(1,Task1V2RandomizationPlanner),(3,Task3RandomizationPlanner)])
def test_seeded_scene_and_geometry(task, planner, monkeypatch):
    # Legacy environment variables must no longer alter either entry's scene.
    monkeypatch.setenv("TASK1_V2_LAYOUT_FILE","/does/not/exist")
    monkeypatch.setenv("TASK3_LAYOUT_FILE","/does/not/exist")
    assert seed_for_round(1)==28
    seen=set()
    for seed in range(28,33):
        plan=planner().plan(seed)
        assert asdict(plan)==asdict(planner().plan(seed))
        with task_scene(task,seed) as filename:
            root=ET.parse(filename).getroot()
            if task==1:
                assert plan.initial_base==(.015,0,0)==plan.task_base
                name="target_bin"; target=plan.target_position
                assert abs(target[1]+.528)<=.1
                objects=[("cylinder_"+str(i),s.position) for i,s in enumerate(plan.cylinders,1)]
            else:
                assert plan.initial_base==(0,0,0)
                name="task3_destination_table"; target=plan.destination_position
                assert target[0] == .62
                assert abs(target[1]-.85)<=.1
                objects=[(s.name,s.position) for s in plan.rings]
                for i,a in enumerate(plan.rings):
                    assert abs(a.destination_xy[1] - ((.90,.76,.62)[i] + target[1]-.85))<1e-9
                    assert abs(a.destination_xy[0]-(target[0]-.12))<1e-9
                    for b in plan.rings[i+1:]:
                        assert math.dist(a.position[:2],b.position[:2])>=.16
            assert tuple(map(float,root.find(".//body[@name='%s']"%name).get("pos").split()))==target
            for name,position in objects:
                assert tuple(map(float,root.find(".//body[@name='%s']"%name).get("pos").split()))==position
            light=tuple(map(float,root.find("visual/headlight").get("diffuse").split()))
            assert light==lighting_for_seed(seed)["headlight_diffuse"]
            if task==1:
                import mujoco
                from utils.scene1_v2_right_arm_ik import Scene1V2RightArmIK
                ik=Scene1V2RightArmIK(filename)
                ik.set_base_translation_world(plan.initial_base)
                for point in ik.preparation_trajectory([0.0]*14,plan.cylinders[0].position):
                    ik._forward(point[7:14])
                    mujoco.mj_forward(ik.model,ik.data)
                    for contact in ik.data.contact:
                        right=[ik.model.body(int(ik.model.geom_bodyid[int(g)])).name
                               .startswith(("r_","zarm_r")) for g in (contact.geom1,contact.geom2)]
                        assert sum(right)!=1

            seen.add(tuple(p for _,p in objects))
    assert len(seen)==5
