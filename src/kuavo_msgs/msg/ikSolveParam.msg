# snopt params
float64 major_optimality_tol
float64 major_feasibility_tol
float64 minor_feasibility_tol
float64 major_iterations_limit
# constraint and cost params
float64 oritation_constraint_tol
float64 pos_constraint_tol # work when pos_cost_weight > 0.0
float64 pos_cost_weight
uint8 constraint_mode   # External value (after internal swap of 0<->1): 0=pos_soft+ori_hard(default), 1=pos_soft+ori_soft, 2=pos_hard+ori_soft, 3=pos_hard+ori_hard, 4=three_point_soft, 6=three_point_mixed
# elbow cost scale (relative to pos_cost_weight): ==0 use default(0.1), <0 disable elbow constraint, >0 use directly
float64 elbow_cost_scale