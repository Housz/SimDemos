/**
 * Small-strain, plane-stress constant-strain triangle (CST) finite elements.
 * Linear Newmark-beta dynamics + matrix-free, diagonally preconditioned CG.
 * Irreversible element-level tensile damage / erosion (educational model).
 * All computation is in SI units: metres, seconds, newtons, kilograms, pascals.
 */
export class FEMBracket {
  static length = 0.12;
  static halfHeight = 0.018;
  static thickness = 0.012;

  constructor(options = {}) {
    this.options = {
      nx: 20, ny: 6, young: 3.0e9, poisson: 0.33,
      density: 1250, strengthMPa: 95, fractureEnergy: 20000,
      pedalMass: 1.2, damping: 24, ...options
    };
    this.time = 0;
    this.terminated = false;
    this.error = '';
    this.makeMesh();
    this.resetState();
  }

  makeMesh() {
    const {nx,ny,young,poisson,density,pedalMass} = this.options;
    const L = FEMBracket.length;
    const H = FEMBracket.halfHeight;
    const t = FEMBracket.thickness;
    const nodes = [];
    for (let i = 0; i <= nx; i++) {
      const x = L * i / nx;
      // Local narrowing near the root; no real vehicle geometry is claimed.
      const half = H - 0.0038 * Math.exp(-Math.pow((x - 0.029) / 0.010, 2));
      for (let j = 0; j <= ny; j++) {
        nodes.push({ x, y: half * (2 * j / ny - 1) });
      }
    }
    this.nodes = nodes;
    this.n = nodes.length * 2;
    this.mass = new Float64Array(this.n);
    this.fixed = new Uint8Array(this.n);
    this.loadNodes = [];
    for (let j = 0; j <= ny; j++) {
      const fixed = j * 2;
      this.fixed[fixed] = this.fixed[fixed + 1] = 1;
      const tip = nx * (ny + 1) + j;
      this.loadNodes.push(tip);
      this.mass[2 * tip] += pedalMass / (ny + 1);
      this.mass[2 * tip + 1] += pedalMass / (ny + 1);
    }
    const e0 = young / (1 - poisson * poisson);
    const D = [e0, e0 * poisson, 0, e0 * poisson, e0, 0,
      0, 0, e0 * (1 - poisson) * 0.5];
    this.elements = [];
    const addTriangle = (i0,i1,i2) => {
      const a = nodes[i0], b = nodes[i1], c = nodes[i2];
      const doubleA = (b.x-a.x)*(c.y-a.y) - (c.x-a.x)*(b.y-a.y);
      if (doubleA <= 0) throw new Error('Invalid FEM element orientation');
      const area = doubleA * 0.5;
      const bx = [b.y-c.y, c.y-a.y, a.y-b.y];
      const cy = [c.x-b.x, a.x-c.x, b.x-a.x];
      const B = new Float64Array(18);
      for (let j=0; j<3; j++) {
        const k = 2*j;
        B[k] = bx[j]/doubleA;
        B[6 + k + 1] = cy[j]/doubleA;
        B[12 + k] = cy[j]/doubleA;
        B[12 + k + 1] = bx[j]/doubleA;
      }
      const kLocal = new Float64Array(36);
      for (let r=0;r<6;r++) for (let s=0;s<6;s++) {
        let v=0;
        for (let p=0;p<3;p++) for (let q=0;q<3;q++) {
          v += B[p*6+r]*D[p*3+q]*B[q*6+s];
        }
        kLocal[r*6+s] = v * area * t;
      }
      const nodeIds = [i0,i1,i2];
      const dofs = [2*i0,2*i0+1,2*i1,2*i1+1,2*i2,2*i2+1];
      const m=area*t*density/3;
      for (const node of nodeIds) {
        this.mass[node*2] += m;
        this.mass[node*2+1] += m;
      }
      this.elements.push({nodeIds,dofs,B,kLocal,area,
        length:Math.sqrt(2*area), damage:0, history:0,failed:false,
        stress:0, strain:0});
    };
    for (let i=0;i<nx;i++) for(let j=0;j<ny;j++) {
      const a=i*(ny+1)+j, b=(i+1)*(ny+1)+j,
        c=b+1, d=a+1;
      if ((i+j)%2===0) { addTriangle(a,b,d);addTriangle(b,c,d); }
      else {addTriangle(a,b,c);addTriangle(a,c,d);}
    }
    this.fixedNodes = Array.from({length:ny+1},(_,j)=>j);
  }

  resetState() {
    this.time=0;this.terminated=false;this.error='';
    this.u=new Float64Array(this.n);
    this.v=new Float64Array(this.n);
    this.a=new Float64Array(this.n);
    this.maxDamage=0;this.maxStress=0;this.failedCount=0;
    this.maxDisplacement=0;this.load=0;
    this.connected=true;
    this.lastIterations=0;
    for(const e of this.elements) {
      e.damage=0;e.history=0;e.failed=false;e.stress=0;e.strain=0;
    }
    // Reused work vectors: avoid per-step allocation / garbage collection.
    this.pred=new Float64Array(this.n);
    this.vpred=new Float64Array(this.n);
    this.rhs=new Float64Array(this.n);
    this.x=new Float64Array(this.n);
    this.r=new Float64Array(this.n);
    this.z=new Float64Array(this.n);
    this.p=new Float64Array(this.n);
    this.Ap=new Float64Array(this.n);
    this.invDiag=new Float64Array(this.n);
  }

  /** A*x = (K_damage + a0*M + a1*C)*x, with C = damping*M. */
  applyEffective(x,out,massFactor) {
    const n=this.n, masses=this.mass, fixed=this.fixed;
    for (let i=0;i<n;i++) out[i]=fixed[i]?0:masses[i]*massFactor*x[i];
    for(const e of this.elements) {
      if(e.failed) continue;
      const d=e.dofs, k=e.kLocal, fact=1-e.damage;
      const x0=x[d[0]],x1=x[d[1]],x2=x[d[2]],
        x3=x[d[3]],x4=x[d[4]],x5=x[d[5]];
      for(let i=0;i<6;i++) {
        const idx=d[i]; if(fixed[idx]) continue;
        const z=6*i;
        out[idx] += fact*(k[z]*x0+k[z+1]*x1+k[z+2]*x2+
          k[z+3]*x3+k[z+4]*x4+k[z+5]*x5);
      }
    }
  }

  buildPreconditioner(massFactor) {
    const fixed=this.fixed;
    for(let i=0;i<this.n;i++) this.invDiag[i]=fixed[i]?0:this.mass[i]*massFactor;
    for(const e of this.elements) {
      if(e.failed) continue;
      const factor=1-e.damage, d=e.dofs, k=e.kLocal;
      for(let j=0;j<6;j++) {
        const idx=d[j]; if(!fixed[idx]) this.invDiag[idx] += factor*k[j*6+j];
      }
    }
    for(let i=0;i<this.n;i++) {
      if(!fixed[i]) this.invDiag[i]=1/Math.max(1e-12,this.invDiag[i]);
    }
  }

  solve(massFactor) {
    const {n,x,r,p,z,Ap,rhs,invDiag}=this;
    this.applyEffective(x,Ap,massFactor);
    let rz=0, rNorm2=0, bNorm2=0;
    for(let i=0;i<n;i++) {
      r[i]=rhs[i]-Ap[i];
      z[i]=invDiag[i]*r[i];p[i]=z[i];
      rz+=r[i]*z[i];rNorm2+=r[i]*r[i];bNorm2+=rhs[i]*rhs[i];
    }
    const tolerance2=1e-12*Math.max(1,bNorm2);
    if(rNorm2<=tolerance2) { this.lastIterations=0; return true; }
    for(let iteration=0;iteration<180;iteration++) {
      this.applyEffective(p,Ap,massFactor);
      let pAp=0;
      for(let i=0;i<n;i++) pAp+=p[i]*Ap[i];
      if(!(pAp>0)||!Number.isFinite(pAp)) return false;
      const alpha=rz/pAp;
      let norm2=0;
      for(let i=0;i<n;i++) {
        x[i]+=alpha*p[i];r[i]-=alpha*Ap[i];norm2+=r[i]*r[i];
      }
      if(norm2<=tolerance2) {this.lastIterations=iteration+1;return true;}
      let newRz=0;
      for(let i=0;i<n;i++) {z[i]=invDiag[i]*r[i];newRz+=r[i]*z[i];}
      const beta=newRz/rz;
      for(let i=0;i<n;i++) p[i]=z[i]+beta*p[i];
      rz=newRz;
    }
    this.lastIterations=180;
    return false;
  }

  /** Force history: trapezoidal single/multi pulse, negative y = down. */
  static forceAt(time, settings) {
    const {peak,rise,hold,gap,cycles} = settings;
    const fall=Math.max(0.025,0.7*rise);
    const duration=rise+hold+fall+gap;
    const cycle=Math.floor(time/duration);
    if(cycle>=cycles) return 0;
    const p=time-cycle*duration;
    if(p<rise) return peak*(p/rise);
    if(p<rise+hold) return peak;
    if(p<rise+hold+fall) return peak*(1-(p-rise-hold)/fall);
    return 0;
  }

  static duration(settings) {
    return settings.cycles * (settings.rise+settings.hold+Math.max(0.025,0.7*settings.rise)+settings.gap);
  }

  step(dt, settings) {
    if(this.terminated) return;
    const n=this.n, damping=this.options.damping;
    const beta=0.25, gamma=0.5;
    const a0=1/(beta*dt*dt), a1=gamma/(beta*dt);
    const massFactor=a0+damping*a1;
    const time=this.time+dt;
    const applied=FEMBracket.forceAt(time,settings);
    this.load=applied;
    this.buildPreconditioner(massFactor);
    for(let i=0;i<n;i++) {
      if(this.fixed[i]) {
        this.pred[i]=this.vpred[i]=this.x[i]=this.rhs[i]=0;
        continue;
      }
      this.pred[i]=this.u[i]+dt*this.v[i]+dt*dt*(0.5-beta)*this.a[i];
      this.vpred[i]=this.v[i]+dt*(1-gamma)*this.a[i];
      this.x[i]=this.u[i];
      this.rhs[i]=this.mass[i]*(massFactor*this.pred[i]-damping*this.vpred[i]);
    }
    const nodalForce=applied/this.loadNodes.length;
    for(const node of this.loadNodes) this.rhs[node*2+1]-=nodalForce;
    if(!this.solve(massFactor)) {
      this.error='线性求解器未收敛；请降低载荷或调整参数';
      this.terminated=true;
      return;
    }
    for(let i=0;i<n;i++) {
      if(this.fixed[i]) continue;
      const acc=a0*(this.x[i]-this.pred[i]);
      this.v[i]=this.vpred[i]+gamma*dt*acc;
      this.a[i]=acc;
      this.u[i]=this.x[i];
    }
    this.time=time;
    this.updateDamage();
    this.maxDisplacement=this.tipDisplacement();
    if(!Number.isFinite(this.maxDisplacement) || Math.abs(this.maxDisplacement)>0.025) {
      this.terminated=true;
      this.error='位移超出小应变模型有效范围；已在断裂过程处停止计算';
    }
  }

  tipDisplacement() {
    let total=0;
    for(const node of this.loadNodes) total += this.u[2*node+1];
    return total/this.loadNodes.length;
  }

  updateDamage() {
    const E=this.options.young,nu=this.options.poisson;
    const strength=this.options.strengthMPa*1e6;
    const eps0=strength/E;
    const gc=this.options.fractureEnergy;
    const mult=E/(1-nu*nu);
    let stressMax=0,damageMax=0,fail=0,changed=false;
    for(const e of this.elements) {
      const dofs=e.dofs,B=e.B,u=this.u;
      const local=[u[dofs[0]],u[dofs[1]],u[dofs[2]],u[dofs[3]],u[dofs[4]],u[dofs[5]]];
      let ex=0,ey=0,gxy=0;
      for(let i=0;i<6;i++) {
        ex+=B[i]*local[i]; ey+=B[6+i]*local[i];gxy+=B[12+i]*local[i];
      }
      const eps1=(ex+ey)/2 + Math.sqrt(((ex-ey)/2)**2+(gxy/2)**2);
      e.strain=Math.max(0,eps1);
      e.history=Math.max(e.history,e.strain);
      // Crack-band regularization: 0.5*sigma_c*(eps_f-eps_0)*h = Gc.
      const epsF=eps0+2*gc/(strength*e.length);
      if(e.history>eps0) {
        const candidate=e.history>=epsF?1:
          1-(eps0/e.history)*(epsF-e.history)/(epsF-eps0);
        const prev=e.damage;
        e.damage=Math.min(1,Math.max(prev,candidate));
        if(e.damage>prev+1e-8) changed=true;
      }
      if(e.damage>=0.999) e.failed=true;
      const sx=mult*(ex+nu*ey),sy=mult*(nu*ex+ey),txy=mult*(1-nu)*gxy/2;
      e.stress=e.failed?0:(1-e.damage)*Math.max(0,(sx+sy)/2+Math.hypot((sx-sy)/2,txy));
      stressMax=Math.max(stressMax,e.stress);
      damageMax=Math.max(damageMax,e.damage);
      if(e.failed) fail++;
    }
    this.maxStress=stressMax;
    this.maxDamage=damageMax;
    this.failedCount=fail;
    if(changed && fail>0) this.connected=this.checkConnection();
  }

  /** Node connectivity through surviving triangles: topological diagnostic only. */
  checkConnection() {
    const parents=Array.from({length:this.nodes.length},(_,i)=>i);
    const find=(i)=>{ while(parents[i]!==i){parents[i]=parents[parents[i]];i=parents[i];} return i; };
    const join=(a,b)=>{a=find(a);b=find(b);if(a!==b)parents[b]=a;};
    for(const e of this.elements) if(!e.failed) {
      join(e.nodeIds[0],e.nodeIds[1]);join(e.nodeIds[0],e.nodeIds[2]);
    }
    const roots=new Set(this.fixedNodes.map(find));
    return this.loadNodes.some(n=>roots.has(find(n)));
  }
}
