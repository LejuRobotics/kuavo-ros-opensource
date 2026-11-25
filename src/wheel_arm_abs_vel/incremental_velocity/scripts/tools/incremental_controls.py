"""Beta wheel controller semantics, in mapper axes unless marked as base coordinates."""
import math
import numpy as np
from .lightweight_transforms import quaternion_to_matrix, matrix_to_rpy, rpy_to_matrix
from .pose_velocity_kalman import exp_so3, log_so3, _NATIVE


def xyz(v):
    return np.array((v.x, v.y, v.z))


def rotation(q):
    return quaternion_to_matrix((q.x, q.y, q.z, q.w))


def pose(p):
    return rotation(p.orientation), xyz(p.position)


class ArmModes:
    def __init__(self):
        self.down = [False, False]
        self.first = [None, None]
        self.active = [True, True]
        self.transition = [False, False]
        self.last_change = -math.inf

    def reset(self):
        self.first[:] = (None, None)
        self.active[:] = (True, True)
        self.transition[:] = (False, False)

    def update(self, msg, now, enabled):
        keys = [msg.left_first_button_pressed, msg.right_first_button_pressed]
        changed = []
        if not enabled:
            self.first[:] = (None, None)
        else:
            for arm, pressed in enumerate(keys):
                if not pressed or self.down[arm]:
                    continue
                first = self.first[arm]
                if first is None or now - first > .5:
                    self.first[arm] = now
                elif now >= first:
                    self.first[arm] = now + .5
                    if now - self.last_change >= 2.:
                        self.last_change = now
                        self.active[arm] = not self.active[arm]
                        self.transition[arm] = True
                        changed.append(arm)
        self.down = keys
        return changed


class Workspace:
    def __init__(self, model, side, fallback_box):
        bounds = model.get('drake_chest_elbow_hand_bounds', {})
        self.lower = np.asarray(bounds.get(side+'_hand_lower_bound', fallback_box[0]))
        self.upper = np.asarray(bounds.get(side+'_hand_upper_bound', fallback_box[1]))
        self.elbow_clearance = float(bounds.get('min_p1_xy_norm', .2))
        weights = model.get('drake_chest_elbow_hand_weights', {})
        self.elbow_weight = float(weights.get('q1', .8))
        self.hand_weight = float(weights.get('q2', 1.))
        if min(self.elbow_weight,self.hand_weight) < 0 or not np.isfinite([self.elbow_weight,self.hand_weight]).all():
            raise ValueError('invalid legacy hand/elbow weights')
        self.sign = 1. if side == 'left' else -1.
        if (not np.isfinite([self.lower, self.upper]).all() or np.any(self.lower > self.upper)
                or not math.isfinite(self.elbow_clearance)):
            raise ValueError('invalid legacy workspace bounds')

    def project(self, p, v, elbow, base_from_mapper, base_from_chest, shoulder, reach, lengths=None):
        if _NATIVE is not None:
            return _NATIVE.workspace_project(p, v, elbow, *base_from_mapper, *base_from_chest,
                shoulder, reach, None if lengths is None else np.asarray(lengths),
                self.lower, self.upper, self.elbow_clearance, self.elbow_weight, self.hand_weight, self.sign)
        return self.project_python(p, v, elbow, base_from_mapper, base_from_chest, shoulder, reach, lengths)

    def project_python(self, p, v, elbow, base_from_mapper, base_from_chest, shoulder, reach, lengths=None):
        R, t = base_from_mapper
        chest_R, chest = base_from_chest
        yaw = math.atan2(chest_R[1,0], chest_R[0,0])
        front = np.array([math.cos(yaw), math.sin(yaw), 0.])
        side = self.sign * np.array([-front[1], front[0], 0.])
        hand, velocity = R @ p + t, R @ v
        center = R @ shoulder + t
        # Beta remapUpperBodyRefPoints clips shoulder reach before applying
        # its base-frame hand box and chest-relative front/height bounds.
        initial = hand - center
        initial_length = np.linalg.norm(initial)
        if initial_length > reach:
            hand = center + initial*(reach/initial_length)
        request = hand.copy()
        hi = self.upper.copy()
        hi[2] = min(hi[2], chest[2] + .25)
        # The wrist must satisfy the base-frame box, yaw-relative half-space
        # and shoulder reach simultaneously. No cyclic solver is used for joints.
        for _ in range(16):
            before = hand.copy()
            hand = np.clip(hand, self.lower, hi)
            hand += max(0., -.03 - front @ (hand-chest)) * front
            radial = hand - center
            length = np.linalg.norm(radial)
            if length > reach:
                hand = center + radial * (reach / length)
            if lengths is not None:
                upper, lower = lengths
                # The elbow is constrained to the lateral spherical cap about
                # the shoulder. Its closest point gives the wrist reach limit.
                distance = np.linalg.norm(hand-center)
                axis = (hand-center)/distance if distance>1.e-9 else front
                if distance < abs(upper-lower)+1.e-8:
                    hand = center + (abs(upper-lower)+1.e-8)*axis
                nearest = center + upper*axis
                lateral = side @ (nearest-chest)
                if lateral < self.elbow_clearance:
                    height = self.elbow_clearance - side @ (center-chest)
                    if height > upper:
                        raise ValueError('infeasible elbow clearance')
                    tangent = axis-(side@axis)*side
                    norm = np.linalg.norm(tangent)
                    tangent = tangent/norm if norm>1.e-9 else front
                    nearest = center + height*side + math.sqrt(max(0.,upper**2-height**2))*tangent
                away = hand-nearest
                if np.linalg.norm(away)>lower:
                    hand=nearest+lower*away/np.linalg.norm(away)
            if np.linalg.norm(hand-before) < 1.e-10:
                break
        if (np.any(hand < self.lower-1.e-6) or np.any(hand > hi+1.e-6)
                or front @ (hand-chest) < -.030001):
            raise ValueError('wrist has no admitted projection')
        velocity[(hand <= self.lower+1.e-7) & (velocity < 0)] = 0.
        velocity[(hand >= hi-1.e-7) & (velocity > 0)] = 0.
        if front @ (hand-chest) <= -.03+1.e-7:
            velocity -= min(0., front @ velocity) * front
        radial = hand-center
        if np.linalg.norm(radial) >= reach-1.e-7:
            n = radial/max(1.e-12, np.linalg.norm(radial))
            velocity -= max(0., n @ velocity) * n
        if elbow is not None:
            e = R @ elbow + t
            if lengths is None:
                e += max(0., self.elbow_clearance - side @ (e-chest)) * side
            else:
                # Retain the original bend preference when the request first hits
                # its shoulder reach. A wall can then conflict with that preference.
                preferred = self._elbow_on_circle(request, center, e, lengths, front, side, None)
                offset = self.elbow_clearance + side @ chest
                e = self._elbow_on_circle(hand, center, preferred, lengths, front, side, offset)
                if np.linalg.norm(hand-request) > 1.e-8 and self.elbow_weight > 0.:
                    axes = np.eye(3)
                    normals = [axes[i] for i in range(3) if hand[i] <= self.lower[i]+1.e-7 or hand[i] >= hi[i]-1.e-7]
                    if front @ (hand-chest) <= -.03+1.e-7:
                        normals.append(front)
                    if normals:
                        # On a clipped plane there are two geometric endpoints:
                        # nearest wrist, or nearest wrist retaining the desired
                        # elbow. Interpolate using the beta task error weights.
                        tangent_space = np.eye(3)
                        for normal in normals:
                            n = tangent_space @ normal
                            if n@n > 1.e-12:
                                tangent_space -= np.outer(n,n)/(n@n)
                        fixed = (np.eye(3)-tangent_space) @ (hand-preferred)
                        free = tangent_space @ (request-preferred)
                        rho2 = lengths[1]**2-fixed@fixed
                        if rho2 > 0. and free@free > 1.e-12:
                            alternate = preferred+fixed+math.sqrt(rho2/(free@free))*free
                            correction = alternate-hand
                            elbow_cost = self.elbow_weight*((e-preferred)@(e-preferred))
                            hand_cost = self.hand_weight*(correction@correction)
                            if elbow_cost+hand_cost > 1.e-14:
                                blended = hand + correction*(elbow_cost/(elbow_cost+hand_cost))
                                if (np.all(blended >= self.lower-1.e-8) and np.all(blended <= hi+1.e-8)
                                        and front@(blended-chest) >= -.03000001
                                        and np.linalg.norm(blended-center) <= reach+1.e-8):
                                    trial = self._elbow_on_circle(blended, center, preferred, lengths, front, side, offset)
                                    hand, e = blended, trial
            elbow = R.T @ (e-t)
        return R.T @ (hand-t), R.T @ velocity, elbow


    @staticmethod
    def _elbow_on_circle(hand, shoulder, preferred, lengths, front, side, offset):
        upper, lower = lengths
        length = np.linalg.norm(hand-shoulder)
        if length < 1.e-9:
            raise ValueError('undefined shoulder-to-wrist axis')
        axis = (hand-shoulder)/length
        along = np.clip((upper**2-lower**2+length**2)/(2.*length), -upper, upper)
        mid = shoulder+along*axis
        radius = math.sqrt(max(0.,upper**2-along**2))
        tangent = preferred-mid-axis*(axis@(preferred-mid))
        lateral = side-axis*(side@axis)
        if tangent@tangent < 1.e-16:
            tangent = lateral if lateral@lateral > 1.e-16 else front-axis*(front@axis)
        elbow = mid+radius*tangent/max(1.e-12,np.linalg.norm(tangent))
        if offset is not None and side@elbow < offset-1.e-8:
            norm = np.linalg.norm(lateral)
            if norm < 1.e-9:
                raise ValueError('no elbow on admitted side')
            unit = lateral/norm
            height = (offset-side@mid)/norm
            if height > radius+1.e-6:
                raise ValueError('no elbow on admitted circle')
            perpendicular = np.cross(axis,unit)
            if perpendicular@tangent < 0.:
                perpendicular = -perpendicular
            elbow = mid+height*unit+math.sqrt(max(0.,radius**2-height**2))*perpendicular
        return elbow


def bound_rotation(goal, measured, link4, limits, incremental, engage, with_branch=False):
    # Re-engagement inherits the exact command pose. Applying EE/link4 boxes
    # only after an entry delay changes the admissible frame/conditions again
    # on the next sample (the recorded double jump).
    # Final admission is identical on every sample, after chest remapping.
    final, _, _ = limits
    delta = measured.T @ goal
    rv = log_so3(delta)
    def result(value, branch):
        return (value, branch) if with_branch else value
    if incremental:
        radius = min(float(np.min(final)), math.pi/2 - 1.e-6)
        length = np.linalg.norm(rv)
        if length <= radius:
            return result(goal, ('raw',))
        axis = int(np.argmax(np.abs(rv)))
        return result(measured @ exp_so3(rv * (radius/length)), ('so3', axis, int(np.sign(rv[axis]))))
    # !4081: do not reconstruct a tiny rotation, or Euler yaw/roll near
    # the ZYX gimbal. Quaternion signs represent the same physical rotation.
    if np.linalg.norm(rv) < 1.e-3 or abs(delta[2, 0]) >= .94:
        return result(goal, ('raw',))
    rpy = matrix_to_rpy(delta)
    clipped = np.clip(rpy, -final[::-1], final[::-1])
    branch = tuple(np.where(np.abs(rpy)>final[::-1], np.sign(rpy), 0.).astype(int))
    return result(measured @ rpy_to_matrix(clipped), ('euler',)+branch)


class RotationTransfer:
    """Continuous release of an entry/frame-change offset, not another hand filter.

    Away from a handoff the goal and observer velocity pass through exactly.
    The offset decays to identity. Large offsets use a slower decay so the
    absolute pose at regrip cannot demand a fast round of catch-up.
    A branch-cut change of the bounded candidate uses the same transfer, without
    waiting for confirmations, changing clutch state or discarding human input.
    """
    def __init__(self, frequency=4., max_offset_speed=6.0):
        self.frequency = float(frequency)
        if not math.isfinite(self.frequency) or self.frequency <= 0:
            raise ValueError('invalid orientation reference frequency')
        self.max_offset_speed = float(max_offset_speed)
        if not math.isfinite(self.max_offset_speed) or self.max_offset_speed <= 0:
            raise ValueError('invalid regrip orientation speed')
        self.output = self.previous_goal = None
        self.offset = np.zeros(3)
        self.offset_duration = 0.
        self.start = 0.
        self.pending = False
        self.previous_source = self.previous_branch = None

    def _capture_offset(self, held, goal, stamp):
        self.offset = log_so3(held @ goal.T)
        self.start = stamp
        angle = np.linalg.norm(self.offset)
        natural_rate = 2. * math.pi * self.frequency
        # Keep the nominal blend for small offsets. A quintic has zero
        # endpoint velocity/acceleration and peak speed 1.875 * angle / T.
        self.offset_duration = (1.875 * angle / self.max_offset_speed
                                if angle * natural_rate / math.e > self.max_offset_speed else 0.)

    def engage(self, held, stamp, goal=None, source=None, branch=None):
        self.output = held.copy()
        self.previous_goal = None if goal is None else goal.copy()
        self.previous_source = None if source is None else source.copy()
        self.previous_branch = branch
        self.start = stamp
        self.pending = goal is None
        if goal is not None:
            self._capture_offset(held, goal, stamp)

    def step(self, goal, omega, stamp, source=None, branch=None):
        if self.output is None:
            self.output = goal.copy()
        if self.pending:
            self._capture_offset(self.output, goal, self.start)
            self.pending = False
        elif (branch is not None and self.previous_branch is not None and
              branch != self.previous_branch):
            # Coordinate/limit branch handoff: retain the real human rotation,
            # release only the admission discontinuity. No motion threshold,
            # input gate, or additional HOLD state is introduced.
            expected = source @ self.previous_source.T @ self.output
            self._capture_offset(expected, goal, stamp)
        self.previous_source = None if source is None else source.copy()
        self.previous_branch = branch
        self.previous_goal = goal.copy()
        t = max(0., stamp-self.start)
        if self.offset_duration > 0.:
            u = min(1., t / self.offset_duration)
            blend = 1. - u**3 * (10. - 15.*u + 6.*u*u)
        else:
            u = 2. * math.pi * self.frequency * t
            blend = (1. + u) * math.exp(-u)
        correction = exp_so3(blend*self.offset)
        self.output = correction @ goal
        # Feedforward remains the hand observer's state, including exact zero
        # at rest. The shared IK shaper derives the velocity of this commanded
        # handoff; injecting the offset-decay velocity here would double it.
        return self.output.copy(), correction @ omega


class ModeReturn:
    """Generate the beta mode-change target at its 100 Hz tick. The shared IK executes it."""
    def __init__(self, point, stamp):
        self.p = point.copy()
        self.v = np.zeros(3)
        self.stamp = stamp - .01

    def step(self, goal, stamp, activating):
        if activating:
            return goal.copy()
        # No catch-up after interrupted tracking; the clutch is rearmed by caller.
        steps = min(10, max(0, int((stamp-self.stamp+1.e-9)/.01)))
        r, h0 = .3, .04
        d = r * h0**2
        def sign(x):
            return np.where(np.abs(x) < 1.e-6, 0., np.sign(x))
        for _ in range(steps):
            a0 = h0*self.v
            y = self.p-goal+a0
            a2 = a0 + sign(y)*(np.sqrt(d*(d+8*np.abs(y)))-d)/2
            sy = (sign(y+d)-sign(y-d))/2
            a = (a0+y)*sy + a2*(1-sy)
            sa = (sign(a+d)-sign(a-d))/2
            acceleration = -r*(a/d*sa + sign(a)*(1-sa))
            self.p += .01*self.v
            self.v += .01*acceleration
        self.stamp += steps*.01
        return self.p.copy()


class ReachStop:
    """Discard outward clutch travel only after measured FK stalls at an IK boundary."""
    def __init__(self):
        self.start = None
        self.origin = None
        self.normal = None

    def apply(self, prior, goal, velocity, measured, at_boundary, stamp):
        delta = goal-prior
        if self.normal is not None:
            n = self.normal
            advance = float(n @ delta)
            if advance < -1.e-8:
                # First inward delta starts at the attainable wrist on the
                # blocked axis. Tangential components keep their anchors.
                goal = goal - max(0., n @ (prior-measured)) * n
                self.start = self.normal = None
                return goal, velocity
            if at_boundary and n @ (prior-measured) > .002:
                return goal-max(0.,advance)*n, velocity-max(0.,n@velocity)*n
            self.normal = None
        residual = goal-measured
        distance = np.linalg.norm(residual)
        if not at_boundary or distance < .004:
            self.start = None
            return goal, velocity
        if self.start is None or stamp < self.start or np.linalg.norm(measured-self.origin) > .002:
            self.origin = measured.copy()
            self.start = stamp
        elif stamp-self.start >= .1:
            self.normal = residual/distance
            n = self.normal
            return goal-max(0.,n@delta)*n, velocity-max(0.,n@velocity)*n
        return goal, velocity
