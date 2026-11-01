// The twist's exponential against its definition and against integrating the
// motion, the old coordinates' angle against the quaternion they stand for, and
// the two solves' behaviour near a half turn.
const near = (a, b, e, assert, m = "") => assert.ok(Math.abs(a - b) < e, `${m} ${a} vs ${b}`);

export function a_half_turn_turns_x_into_minus_x(algo, assert) {
  const m = algo.exp([0, 0, Math.PI], [0, 0, 0]);
  const p = algo.qrotate(m.rotation, [1, 0, 0]);
  near(p[0], -1, 1e-14, assert); near(p[1], 0, 1e-14, assert);
  const n = Math.hypot(...m.rotation);
  near(n, 1, 1e-14, assert, "unit");
}

export function exp_agrees_with_integrating_the_twist(algo, assert) {
  // A body twisting at (omega, v) in its own frame: dq/dt = q (0, omega)/2, dp/dt = q v q*.
  const cases = [[[0.3, -0.5, 0.9], [1, 2, -0.5]], [[0, 0, 2.5], [1, 0, 0.7]], [[1.2, 0.1, 0.2], [0, 1, 0]]];
  for (const [omega, v] of cases) {
    const N = 20000, h = 1 / N;
    let q = [1, 0, 0, 0], p = [0, 0, 0];
    const dq = [Math.cos(h * Math.hypot(...omega) / 2), ...omega.map((w) => Math.sin(h * Math.hypot(...omega) / 2) * w / Math.hypot(...omega))];
    for (let k = 0; k < N; k++) {
      // Midpoint rule for the position.
      const qm = algo.qnorm(algo.qmul(q, [Math.cos(h * Math.hypot(...omega) / 4), ...omega.map((w) => Math.sin(h * Math.hypot(...omega) / 4) * w / Math.hypot(...omega))]));
      const u = algo.qrotate(qm, v);
      p = [p[0] + h * u[0], p[1] + h * u[1], p[2] + h * u[2]];
      q = algo.qmul(q, dq);
    }
    const m = algo.exp(omega, v);
    for (let i = 0; i < 3; i++) near(m.position[i], p[i], 1e-7, assert, `position ${i}`);
    for (let i = 0; i < 4; i++) near(m.rotation[i], q[i], 1e-9, assert, `rotation ${i}`);
  }
}

export function small_twists_are_first_order_identity(algo, assert) {
  const m = algo.exp([0, 0, 0], [1, 2, 3]);
  near(m.position[0], 1, 1e-12, assert); near(m.position[1], 2, 1e-12, assert); near(m.position[2], 3, 1e-12, assert);
  near(m.rotation[0], 1, 1e-12, assert);
}

export function the_old_variable_stands_for_twice_its_arctangent(algo, assert) {
  for (const w of [0, 0.5, 2, 10, 300]) {
    const q = algo.qnorm([1, 0, 0, w / 2]);
    const p = algo.qrotate(q, [1, 0, 0]);
    near(Math.atan2(p[1], p[0]), algo.angleOfW(w), 1e-12, assert, `w=${w}`);
    const h = 1e-6;
    near(algo.dAngleDw(w), (algo.angleOfW(w + h) - algo.angleOfW(w - h)) / (2 * h), 1e-8, assert, `slope at ${w}`);
  }
  // Half a turn is reached only as w goes to infinity.
  assert.ok(algo.angleOfW(1e6) < Math.PI);
}

export function twist_steps_arrive_quickly_and_the_old_variable_stalls(algo, assert) {
  const target = 179.5 * Math.PI / 180;
  const a = algo.run(target, 1e-4, "quat", 200), b = algo.run(target, 1e-4, "exp", 200);
  assert.ok(b.residual < 1e-8 && b.steps < 25, `exp: ${b.steps} steps, ${b.residual}`);
  assert.ok(a.steps > 3 * b.steps || a.residual > 1e-8, `old: ${a.steps} steps, ${a.residual}`);
  assert.ok(Math.abs(a.final.w) > 100, `w ${a.final.w}`);
}
