import { FEMBracket } from './physics.js';
import assert from 'node:assert/strict';

function run(peak, strength=160) {
  const params={peak,rise:.15,hold:.10,gap:.10,cycles:1};
  const model=new FEMBracket({strengthMPa:strength,fractureEnergy:30000});
  let maxDisp=0,previousDamage=0;
  const dt=.002;
  for(let i=0;i<250;i++) {
    model.step(dt,params);
    assert.equal(model.terminated,false,`solver failed at step ${i}: ${model.error}`);
    assert.ok(Number.isFinite(model.tipDisplacement()));
    assert.ok(model.maxDamage+1e-10>=previousDamage,'damage must not heal');
    previousDamage=model.maxDamage;
    maxDisp=Math.max(maxDisp,Math.abs(model.tipDisplacement()));
  }
  return {model,maxDisp};
}
const zero=run(0);
assert.ok(zero.maxDisp<1e-10,'zero force should not deform bracket');
const low=run(150), high=run(300);
assert.ok(low.maxDisp>0.0001,'should deform under applied force');
const ratio=high.maxDisp/low.maxDisp;
assert.ok(ratio>1.95&&ratio<2.05,`linear elastic response should scale with force, got ${ratio}`);
assert.equal(high.model.maxDamage,0);
console.log('PASS: zero load, finite solution, elastic linearity, irreversible damage invariant');
