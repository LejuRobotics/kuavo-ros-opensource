"""Causal, fixed-noise CV observers. No ROS, deadband, FIFO or motion gate.

Units: m, s, rad. q is continuous white-acceleration spectral density,
not an acceleration limit. Angular velocity and errors are world-expressed.
"""
import math
import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def exp_so3(v):
    # These fixed-size expressions are Rodrigues' formula with V@V expanded.
    # Avoid allocating identity/skew/product temporaries for each observer.
    x, y, z = v
    xx, yy, zz = x*x, y*y, z*z
    a2 = xx+yy+zz
    a = math.sqrt(a2)
    if a < 1e-6:
        s, c = 1-a2/6, .5-a2/24
    else:
        s, c = math.sin(a)/a, (1-math.cos(a))/a2
    return np.array([[1-c*(yy+zz), c*x*y-s*z, c*x*z+s*y],
                     [c*x*y+s*z, 1-c*(xx+zz), c*y*z-s*x],
                     [c*x*z-s*y, c*y*z+s*x, 1-c*(xx+yy)]])


def left_jacobian(v):
    x, y, z = v
    xx, yy, zz = x*x, y*y, z*z
    a2 = xx+yy+zz
    a = math.sqrt(a2)
    if a < 1e-6:
        s, c = .5-a2/24, 1/6-a2/120
    else:
        s, c = (1-math.cos(a))/a2, (a-math.sin(a))/(a2*a)
    return np.array([[1-c*(yy+zz), c*x*y-s*z, c*x*z+s*y],
                     [c*x*y+s*z, 1-c*(xx+zz), c*y*z-s*x],
                     [c*x*z-s*y, c*y*z+s*x, 1-c*(xx+yy)]])


def log_so3(R):
    a = math.acos(float(np.clip((np.trace(R)-1)*.5, -1., 1.)))
    vee = np.array([R[2, 1]-R[1, 2], R[0, 2]-R[2, 0], R[1, 0]-R[0, 1]])
    if a < 1e-6:
        return (.5+a*a/12)*vee
    if math.pi-a < 1e-5:
        values, vectors = np.linalg.eigh((R+R.T)*.5)
        axis = vectors[:, np.argmax(values)]
        if axis@vee < 0: axis = -axis
        return a*axis
    return a/(2*math.sin(a))*vee


def positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(name + " must be finite and positive")
    return value


def velocity_gain(value):
    value = positive(value, 'velocity correction gain')
    if value > 1.:
        raise ValueError('velocity correction gain must be in (0,1]')
    return value


class PositionCV:
    """Identical independent axes: share one 2x2 covariance, not state."""
    def __init__(self, sigma=.001, q=.2, velocity_correction_gain=1.):
        self.r = positive(sigma, 'position sigma')**2
        self.q = positive(q, 'position q')
        self.velocity_correction_gain = velocity_gain(velocity_correction_gain)
        self.x = None

    def update(self, z, dt=None):
        z = np.asarray(z, dtype=float)
        if z.shape != (3,) or not np.isfinite(z).all():
            raise ValueError('invalid position')
        if self.x is None or dt is None:
            self.x = np.array([z, np.zeros(3)])
            self.P = np.diag([self.r, .25])
        else:
            # Scalar expansion of F P F' + Q, followed by the same Joseph
            # update. Preserve the actual regularized velocity gain and the
            # public covariance; no steady-state gain or noise approximation.
            p00, p01, p10, p11 = self.P.ravel()
            dt2 = dt*dt
            a = p00+dt*(p01+p10)+dt2*p11+self.q*dt2*dt/3
            b = p01+dt*p11+self.q*dt2/2
            c = p10+dt*p11+self.q*dt2/2
            d = p11+self.q*dt
            k0 = a/(a+self.r)
            k1 = c/(a+self.r)*self.velocity_correction_gain
            predicted = self.x[0]+dt*self.x[1]
            residual = z-predicted
            self.x = np.array([predicted+k0*residual,
                               self.x[1]+k1*residual])
            u = 1-k0
            off_diagonal = u*(.5*(b+c)-k1*a)+self.r*k0*k1
            self.P = np.array([[u*u*a+self.r*k0*k0, off_diagonal],
                               [off_diagonal, d-k1*(b+c)+k1*k1*(a+self.r)]])
        return self.x[0].copy(), self.x[1].copy()


class RotationCV:
    def __init__(self, sigma=.003, q=1.8, velocity_correction_gain=1.):
        self.r = positive(sigma, 'rotation sigma')**2
        self.q = positive(q, 'rotation q')
        self.velocity_correction_gain = velocity_gain(velocity_correction_gain)
        self.R = None

    def update(self, measured, dt=None):
        if not np.isfinite(measured).all(): raise ValueError('invalid rotation')
        if self.R is None or dt is None:
            self.R = measured.copy(); self.w = np.zeros(3)
            self.P = np.diag([self.r]*3+[1.]*3)
        else:
            phi = self.w*dt
            E = exp_so3(phi)
            F = np.eye(6); F[:3, :3] = E
            F[:3, 3:] = left_jacobian(phi)*dt
            # Integrate exp(Fc*s) G Qc G' exp(Fc*s)' with Gauss quadrature.
            # Fc = [[skew(w), I], [0, 0]] for spatial/left errors.
            Q = np.zeros((6, 6))
            for node, weight in ((.1127016653792583, 5/18), (.5, 4/9),
                                 (.8872983346207417, 5/18)):
                s = dt*node
                B = np.vstack((left_jacobian(self.w*s)*s, np.eye(3)))
                Q += self.q*dt*weight*(B@B.T)
            self.R = E@self.R
            self.P = F@self.P@F.T+Q
            residual = log_so3(measured@self.R.T)
            K = np.linalg.solve(self.P[:3, :3]+self.r*np.eye(3), self.P[:3, :]).T
            K[3:, :] *= self.velocity_correction_gain
            correction = K@residual
            self.R = exp_so3(correction[:3])@self.R
            self.w += correction[3:]
            A = np.eye(6); A[:, :3] -= K
            self.P = A@self.P@A.T+self.r*(K@K.T)
            reset = np.eye(6); reset[:3, :3] = left_jacobian(correction[:3])
            self.P = reset@self.P@reset.T
            self.P = .5*(self.P+self.P.T)
        return self.R.copy(), self.w.copy()


class SkeletonCV:
    """One observation epoch for the geometry dependency group.

    Equal numerical values at a NEW stamp are still observations. Repeated
    stamps do not update covariance. No prediction across reset/gap/source.
    """
    indices = (0, 1, 2, 3, 4, 5, 22, 23)
    rotating = (4, 5, 23)

    def __init__(self, sigma_p=.001, q_p=.2, sigma_r=.003, q_r=1.8, max_gap=.1,
                 hand_q_r=None, position_velocity_gain=1., rotation_velocity_gain=1.,
                 hand_velocity_gain=1.):
        self.parameters = (positive(sigma_p, 'sigma_p'), positive(q_p, 'q_p'),
                           positive(sigma_r, 'sigma_r'), positive(q_r, 'q_r'))
        # None preserves the legacy API; the ROS mapper explicitly selects
        # the palm noise. Chest and both palms share the same timestamp.
        self.hand_q_r = None if hand_q_r is None else positive(hand_q_r, 'hand_q_r')
        self.position_velocity_gain = velocity_gain(position_velocity_gain)
        self.rotation_velocity_gain = velocity_gain(rotation_velocity_gain)
        self.hand_velocity_gain = velocity_gain(hand_velocity_gain)
        self.max_gap = positive(max_gap, 'max_gap')
        self.reset()

    def reset(self):
        self.stamp = None; self.key = None; self.states = {}
        self.positions = {}; self.rotations = {}; self.updates = 0

    def update(self, transforms, stamp, key):
        if not math.isfinite(stamp): raise ValueError('invalid source stamp')
        if self.key == key and self.stamp is not None and stamp == self.stamp:
            return self.states
        dt = None if self.stamp is None else stamp-self.stamp
        if self.key != key or dt is None or dt <= 0 or dt > self.max_gap:
            self.reset(); dt = None
        sp, qp, sr, qr = self.parameters
        # Validate all required geometry before committing any estimator.
        for i in self.indices:
            if not np.isfinite(transforms[i]).all(): raise ValueError('invalid skeleton')
        for i in self.indices:
            if i not in self.positions: self.positions[i] = PositionCV(sp, qp, self.position_velocity_gain)
            p, v = self.positions[i].update(transforms[i][:3, 3], dt)
            R = transforms[i][:3, :3]; w = np.zeros(3)
            if i in self.rotating:
                rotation_q = self.hand_q_r if i in (4, 5) and self.hand_q_r is not None else qr
                if i not in self.rotations: self.rotations[i] = RotationCV(sr, rotation_q, self.hand_velocity_gain if i in (4,5) else self.rotation_velocity_gain)
                R, w = self.rotations[i].update(R, dt)
            T = np.eye(4); T[:3, :3] = R; T[:3, 3] = p
            self.states[i] = (T, v, w)
        self.stamp = stamp; self.key = key; self.updates += 1
        return self.states


def direction_rate(d, dv):
    length = np.linalg.norm(d)
    if length < 1e-6: raise ValueError('unresolved bone direction')
    n = d/length
    return (dv-n*(n@dv))/length


def _cross3(a, b):
    """Cross product for the fixed three-vector mapping contract."""
    ax, ay, az = a
    bx, by, bz = b
    return np.array([ay*bz-az*by, az*bx-ax*bz, ax*by-ay*bx])


def mapped_rates(chest, shoulder, elbow, palm, axes, upper_length,
                 lower_length, source_rotation, tool_offset,
                 source_wrist_offset=None, wrist_rotation=None):
    """Derivative of the existing normalized-bone mapping, no differencing.

    Optional offsets support WRIST_CONSISTENT_6D as well as legacy TCP.
    Fixed right-multiplied palm calibration leaves spatial omega unchanged.
    """
    C, _, wc = chest; S, vs, _ = shoulder; E, ve, _ = elbow; H, vh, wh = palm
    remove = axes@C[:3, :3].T
    removed_wc = remove@wc

    def mapped_direction_rate(d, dv):
        # Position normalization and its derivative must use the same length.
        length = math.sqrt(float(d@d))
        if length < 1e-6:
            raise ValueError('unresolved bone direction')
        n = d/length
        return (remove@((dv-n*(n@dv))/length)
                - _cross3(removed_wc, remove@n))

    edot = upper_length*mapped_direction_rate(E[:3, 3]-S[:3, 3], ve-vs)
    end = H[:3, 3]; vend = vh
    if source_wrist_offset is not None:
        offset = source_rotation@source_wrist_offset
        end = end+offset; vend = vend+_cross3(wh, offset)
    d = end-E[:3, 3]
    v = edot+lower_length*mapped_direction_rate(d, vend-ve)
    omega = remove@(wh-wc)
    mapped_rotation = remove@source_rotation
    if wrist_rotation is not None: mapped_rotation = mapped_rotation@wrist_rotation
    v += _cross3(omega, mapped_rotation@tool_offset)
    return v, omega, edot
