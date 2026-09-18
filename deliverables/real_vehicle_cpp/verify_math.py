"""Independent formula checks, NOT compilation/execution of the C++ sources."""
import math
import json
from pathlib import Path
import numpy as np
from scipy.linalg import solve_discrete_are


def model(v, yaw, delta, dt=1/30, wheelbase=2.5):
    a = np.eye(3)
    a[0, 2] = -v * dt * math.sin(yaw)
    a[1, 2] = v * dt * math.cos(yaw)
    b = np.array([[dt * math.cos(yaw), 0], [dt * math.sin(yaw), 0],
                  [dt * math.tan(delta)/wheelbase,
                   v * dt/(wheelbase * math.cos(delta)**2)]])
    return a, b


def step(z, u, dt=1/30, wheelbase=2.5):
    x, y, yaw = z
    v, delta = u
    return np.array([x+dt*v*math.cos(yaw), y+dt*v*math.sin(yaw),
                     yaw+dt*v*math.tan(delta)/wheelbase])


def riccati(a, b, q, r, count):
    p = q.copy()
    for _ in range(count):
        gain = np.linalg.solve(r+b.T@p@b, b.T@p@a)
        new = q+a.T@p@a-a.T@p@b@gain
        new = (new+new.T)/2
        difference = np.max(np.abs(new-p))
        p = new
        if difference < 1e-4:
            break
    return np.linalg.solve(r+b.T@p@b, b.T@p@a)


def pp(points, nearest, pose, lookahead=5, wheelbase=2.5):
    target = len(points)-1
    for i in range(nearest+1, len(points)):
        target = i
        if math.dist(points[i], pose[:2]) >= lookahead:
            break
    dx, dy = np.array(points[target])-pose[:2]
    distance = math.hypot(dx, dy)
    sine = math.sin(math.atan2(dy, dx)-pose[2])
    angle = 0 if distance <= 1e-6 or abs(sine) <= 1e-6 else math.atan(2*wheelbase*sine/distance)
    return target, math.degrees(angle)


def main():
    q, r = np.diag([3., 3., 3.]), np.diag([2., 2.])
    maximum_radius = 0
    for v in (0.5, 5/3.6, 10/3.6):
        for yaw in (0, 1.2, -3.13):
            for delta in (0, .2, -.2):
                a, b = model(v, yaw, delta)
                z, u, eps = np.array([2., 3., yaw]), np.array([v, delta]), 1e-6
                numeric_a = np.column_stack([(step(z+eps*e, u)-step(z-eps*e, u))/(2*eps) for e in np.eye(3)])
                numeric_b = np.column_stack([(step(z, u+eps*e)-step(z, u-eps*e))/(2*eps) for e in np.eye(2)])
                np.testing.assert_allclose(a, numeric_a, atol=1e-8)
                np.testing.assert_allclose(b, numeric_b, atol=1e-8)
                gain = riccati(a, b, q, r, 100)
                radius = max(abs(np.linalg.eigvals(a-b@gain)))
                assert radius < 1
                maximum_radius = max(maximum_radius, float(radius))
                p = solve_discrete_are(a, b, q, r)
                expected = np.linalg.solve(r+b.T@p@b, b.T@p@a)
                np.testing.assert_allclose(riccati(a, b, q, r, 10000), expected, atol=1e-4)
    a, b = model(1.5, 0, 0)
    gain = riccati(a, b, q, r, 100)
    assert (-gain @ np.array([0., 1., 0.]))[1] < 0
    assert (-gain @ np.array([0., -1., 0.]))[1] > 0
    assert pp([(0, 0), (10, 0)], 0, (0, 1, 0))[1] < 0
    assert pp([(0, 0), (10, 0)], 0, (0, -1, 0))[1] > 0
    assert pp([(0, 0)], 0, (0, 0, 0)) == (0, 0)
    assert pp([(0, 0), (1, 0)], 1, (0, 0, 0)) == (1, 0)
    wrapped = math.atan2(math.sin(-math.pi+.01-(math.pi-.01)), math.cos(-math.pi+.01-(math.pi-.01)))
    assert abs(wrapped-.02) < 1e-12
    here = Path(__file__).parent
    for name in ('pp', 'lqr'):
        source = (here / f'{name}_tracking_controller.cpp').read_text(encoding='utf-8')
        assert 'TODO(' not in source
    result = {'status': 'passed', 'model_cases': 27,
              'max_closed_loop_spectral_radius_100_iterations': maximum_radius,
              'checks': ['analytic model vs finite differences', 'LQR vs scipy DARE',
                         'LQR and PP corrective direction', 'PP endpoint and coincident point',
                         'yaw wrap', 'no remaining template TODO'],
              'limitation': 'Independent Python numerical checks only; ROS C++ build not performed.'}
    (here / 'verification.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
