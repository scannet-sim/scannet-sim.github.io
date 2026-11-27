// Unitree G1 locomotion policy (unitree_rl_gym, BSD-3-Clause): LSTM(47 -> 64) + MLP 64 -> 32 (ELU) -> 12, run in plain JS.
// Same maths as torch.nn.LSTM (gate order i, f, g, o) and the deploy_mujoco loop of unitree_rl_gym.

export class G1Policy {
  constructor(cfg, w) {
    this.cfg = cfg;
    const get = (k) => {
      const s = cfg.weights[k];
      const n = s.shape.reduce((a, b) => a * b, 1);
      return w.subarray(s.offset, s.offset + n);
    };
    this.Wih = get('memory.weight_ih_l0'); this.Whh = get('memory.weight_hh_l0');
    this.bih = get('memory.bias_ih_l0'); this.bhh = get('memory.bias_hh_l0');
    this.W1 = get('actor.0.weight'); this.b1 = get('actor.0.bias');
    this.W2 = get('actor.2.weight'); this.b2 = get('actor.2.bias');
    this.H = cfg.hidden; this.nIn = cfg.num_obs; this.nOut = cfg.num_actions;
    this.h = new Float32Array(this.H); this.c = new Float32Array(this.H);
    this.g = new Float32Array(4 * this.H); this.z = new Float32Array(32); this.a = new Float32Array(this.nOut);
  }

  static async load(base, fetcher = fetch) {
    const cfg = await (await fetcher(base + 'policy.json')).json();
    const buf = await (await fetcher(base + 'policy.bin')).arrayBuffer();
    return new G1Policy(cfg, new Float32Array(buf));
  }

  reset() { this.h.fill(0); this.c.fill(0); }

  act(x) {
    const { H, nIn, Wih, Whh, bih, bhh, h, c, g } = this;
    for (let r = 0; r < 4 * H; r++) {
      let s = bih[r] + bhh[r];
      const oi = r * nIn, oh = r * H;
      for (let k = 0; k < nIn; k++) s += Wih[oi + k] * x[k];
      for (let k = 0; k < H; k++) s += Whh[oh + k] * h[k];
      g[r] = s;
    }
    const sig = (v) => 1 / (1 + Math.exp(-v));
    for (let k = 0; k < H; k++) {
      const i = sig(g[k]), f = sig(g[H + k]), gg = Math.tanh(g[2 * H + k]), o = sig(g[3 * H + k]);
      c[k] = f * c[k] + i * gg;
      h[k] = o * Math.tanh(c[k]);
    }
    const { W1, b1, W2, b2, z, a } = this;
    for (let r = 0; r < 32; r++) {
      let s = b1[r];
      for (let k = 0; k < H; k++) s += W1[r * H + k] * h[k];
      z[r] = s > 0 ? s : Math.exp(s) - 1;
    }
    for (let r = 0; r < this.nOut; r++) {
      let s = b2[r];
      for (let k = 0; k < 32; k++) s += W2[r * 32 + k] * z[k];
      a[r] = s;
    }
    return a;
  }
}
