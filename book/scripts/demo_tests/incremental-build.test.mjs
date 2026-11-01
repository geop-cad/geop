// The model of the runner against its specification: reuse must never change
// what is built (the same check as assert_incremental), and exactly what read
// what changed is made again.
const same = (a, b) => JSON.stringify(a) === JSON.stringify(b);

export function incremental_equals_from_scratch_under_random_edits(algo, assert) {
  let seed = 12345;
  const rand = (n) => { seed = (seed * 1103515245 + 12345) % 2147483648; return seed % n; };
  const inputs = { width: 2, depth: 1, radius: 0.4, title: 1 };
  const order = ["outline", "hole_sketch", "box", "hole", "label"];
  const part = algo.emptyPart();
  for (let k = 0; k < 500; k++) {
    const what = rand(5);
    if (what === 0) inputs.width = 1 + rand(30) / 10;
    if (what === 1) inputs.depth = 0.5 + rand(15) / 10;
    if (what === 2) inputs.radius = 0.1 + rand(5) / 10;
    if (what === 3) inputs.title = rand(5);
    if (what === 4) { const a = rand(5), b = rand(5); [order[a], order[b]] = [order[b], order[a]]; }
    const program = algo.plateProgram(order);
    algo.run(part, program, inputs);
    const scratch = algo.emptyPart();
    algo.run(scratch, program, inputs);
    assert.ok(same(algo.values(part), algo.values(scratch)), `edit ${k}: differs from a build from scratch`);
  }
}
export function nothing_changed_makes_nothing(algo, assert) {
  const inputs = { width: 2, depth: 1, radius: 0.4, title: 1 };
  const part = algo.emptyPart();
  const program = algo.plateProgram();
  algo.run(part, program, inputs);
  assert.equal(part.makes, 5);
  const st = algo.run(part, program, inputs);
  assert.equal(part.makes, 5);
  assert.ok(Object.values(st).every((s) => s === "reused"));
}
export function an_equal_value_stops_the_wave(algo, assert) {
  const inputs = { width: 2, depth: 1, radius: 0.4, title: 1 };
  const part = algo.emptyPart();
  const program = algo.plateProgram();
  algo.run(part, program, inputs);
  // 2.2 rounds to the same width: the sketch is made again, into an equal value.
  const st = algo.run(part, program, { ...inputs, width: 2.2 });
  assert.equal(st.outline, "kept");
  assert.equal(st.box, "reused");
  assert.equal(st.hole, "reused");
}
export function a_change_reruns_exactly_its_readers(algo, assert) {
  const inputs = { width: 2, depth: 1, radius: 0.4, title: 1 };
  const part = algo.emptyPart();
  const program = algo.plateProgram();
  algo.run(part, program, inputs);
  const st = algo.run(part, program, { ...inputs, radius: 0.3 });
  assert.equal(JSON.stringify(st), JSON.stringify({ outline: "reused", hole_sketch: "made", box: "reused", hole: "made", label: "reused" }));
  const st2 = algo.run(part, program, { ...inputs, radius: 0.3, title: 2 });
  assert.equal(st2.label, "made");
  assert.equal(st2.hole, "reused");
}
export function swapping_independent_steps_reuses_both(algo, assert) {
  const inputs = { width: 2, depth: 1, radius: 0.4, title: 1 };
  const part = algo.emptyPart();
  algo.run(part, algo.plateProgram(), inputs);
  const st = algo.run(part, algo.plateProgram(["hole_sketch", "outline", "box", "hole", "label"]), inputs);
  assert.ok(Object.values(st).every((s) => s === "reused"), JSON.stringify(st));
}
