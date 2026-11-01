// The three steps of hidden-line removal on scenes whose answer is known.
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

export function view_frame_is_orthonormal(algo, assert) {
  const f = algo.viewFrame(0.7, 0.4);
  for (const [a, b, want] of [[f.right, f.right, 1], [f.up, f.up, 1], [f.eye, f.eye, 1], [f.right, f.up, 0], [f.right, f.eye, 0], [f.up, f.eye, 0]]) {
    assert.ok(Math.abs(dot(a, b) - want) < 1e-12);
  }
}
export function a_box_seen_from_a_corner_shows_nine_edges_and_hides_three(algo, assert) {
  const r = algo.hiddenLines([algo.box([-1, -1, -1], [1, 1, 1])], algo.viewFrame(0.65, 0.55));
  // No projected edges of one box cross but where the hidden corner sits over the
  // silhouette: count whole edges by their pieces' visibility at their middles.
  let visible = 0, hidden = 0;
  for (const line of r.lines) {
    const mid = line.a.map((x, k) => (x + line.b[k]) / 2);
    if (algo.hidden([algo.box([-1, -1, -1], [1, 1, 1])], mid, algo.viewFrame(0.65, 0.55).eye)) hidden++; else visible++;
  }
  assert.equal(visible, 9);
  assert.equal(hidden, 3);
}
export function a_block_in_front_hides_what_is_behind_it(algo, assert) {
  const view = algo.viewFrame(0, 0); // eye along -y
  assert.ok(Math.abs(view.eye[1] + 1) < 1e-12);
  const front = algo.box([-1, -2, -1], [1, -1, 1]), back = algo.box([-0.5, 1, -0.5], [0.5, 2, 0.5]);
  const r = algo.hiddenLines([front, back], view);
  // Every piece of the block behind lies over the front one on the paper: all hidden.
  for (const piece of r.pieces) {
    if (piece.mid[1] > 0.5) assert.equal(piece.visible, false);
  }
}
export function an_edge_passing_behind_is_cut_at_the_outline(algo, assert) {
  const view = algo.viewFrame(0, 0);
  // A thin long bar behind a post: its long edges pass behind the post.
  const post = algo.box([-0.5, -2, -2], [0.5, -1, 2]), bar = algo.box([-3, 1, -0.5], [3, 2, 0.5]);
  const r = algo.hiddenLines([post, bar], view);
  // Pieces of the bar's top front edge, y = 1, z = 0.5.
  const pieces = r.pieces.filter((p) => Math.abs(p.mid[2] - 0.5) < 1e-9 && Math.abs(p.mid[1] - 1) < 1e-9 && Math.abs(p.paper[0][1] - p.paper[1][1]) < 1e-9);
  const sorted = pieces.sort((a, b) => Math.min(a.paper[0][0], a.paper[1][0]) - Math.min(b.paper[0][0], b.paper[1][0]));
  assert.equal(sorted.map((p) => p.visible).join(), "true,false,true");
  const middle = sorted[1].paper.map((q) => q[0]).sort((a, b) => a - b);
  assert.ok(Math.abs(middle[0] + 0.5) < 1e-9 && Math.abs(middle[1] - 0.5) < 1e-9);
}
export function a_silhouette_has_the_normal_square_to_the_view(algo, assert) {
  const view = algo.viewFrame(0.9, 0.3);
  for (const a of algo.silhouetteAngles(view.eye)) {
    assert.ok(Math.abs(dot([Math.cos(a), Math.sin(a), 0], view.eye)) < 1e-12);
  }
  // And it is the outline: the two generators are the extremes across the cylinder on the paper.
  const xs = [];
  for (let i = 0; i < 720; i++) {
    const a = (i * Math.PI) / 360;
    xs.push(dot([Math.cos(a), Math.sin(a), 0], view.right));
  }
  const reach = Math.max(...xs);
  for (const a of algo.silhouetteAngles(view.eye)) {
    assert.ok(Math.abs(Math.abs(dot([Math.cos(a), Math.sin(a), 0], view.right)) - 1) < 1e-9);
  }
  assert.ok(Math.abs(reach - 1) < 1e-4);
}
export function a_ray_from_a_face_towards_the_eye_is_not_hidden_by_its_own_solid(algo, assert) {
  const b = algo.box([-1, -1, -1], [1, 1, 1]);
  assert.equal(algo.rayInside(b, [0, -1, 0], [0, -1, 0]), null);
  assert.ok(algo.rayInside(b, [0, 1, 0], [0, -1, 0]) !== null);
  // A ray running in a face of the solid passes through no interior.
  assert.equal(algo.rayInside(b, [0, -1, -1], [0, -1, 0]), null);
  assert.equal(algo.rayInside(b, [1, -1, 0], [0, 1, 0]), null);
}
