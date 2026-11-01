// The spacing of floats against what IEEE 754 says it is.
export function ulp_of_one_is_two_to_the_precision(a, assert) {
  assert.equal(a.ulp64(1), Math.pow(2, -52));
  assert.equal(a.ulp32(1), Math.pow(2, -23));
}
export function spacing_doubles_at_each_power_of_two(a, assert) {
  for (let e = -40; e < 40; e++) {
    const x = Math.pow(2, e);
    assert.equal(a.ulp64(x), Math.pow(2, e - 52));
    assert.equal(a.ulp64(1.5 * x), Math.pow(2, e - 52));
    assert.equal(a.ulp32(x), Math.pow(2, e - 23));
  }
}
export function next_up_steps_through_zero_and_subnormals(a, assert) {
  assert.equal(a.nextUp64(0), 5e-324);
  assert.ok(a.nextUp64(-5e-324) === 0);
  assert.equal(a.nextUp64(1), 1 + Math.pow(2, -52));
  assert.equal(a.nextUp64(-1), -1 + Math.pow(2, -53));
}
export function fields_put_the_value_back_together(a, assert) {
  for (const x of [1, 0.1, 1000, 1e16, 2.5e-5, 123456.789]) {
    const f = a.fields64(x);
    const v = (1 + Number(f.fraction) / Math.pow(2, 52)) * Math.pow(2, f.unbiased);
    assert.equal(v, x);
  }
}
