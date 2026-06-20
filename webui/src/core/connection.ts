import { loadRig, type Rig } from "./wasm";
import { snapshot, type Snapshot, type BoxView } from "../stores";

export interface SystemConnection {
  setSwitch(box: number, on: boolean): void;
  heartbeat(): void;
  arm(): void;
  disarm(): void;
  estop(): void;
  clearEstop(): void;
  fire(box: number, ch: number): void;
  loadSequence(triples: number[]): number;
  startSequence(): void;
  stopSequence(): void;
  start(): void;        // begin the sim-clock loop
  stop(): void;         // pause the loop
  setConnected(on: boolean): void; // when false, stop sending heartbeats (simulate phone drop)
}

const CHANNELS = 16;

export class SimConnection implements SystemConnection {
  private rig!: Rig;
  private now = 0;
  private last = 0;
  private nonce = 1;
  private timer: ReturnType<typeof setInterval> | null = null;
  private connected = true;

  static async create(): Promise<SimConnection> {
    const c = new SimConnection();
    c.rig = await loadRig();
    c.rig.reset();
    c.publish();
    return c;
  }

  setSwitch(box: number, on: boolean) { this.rig.setSwitch(box, on, this.now); this.publish(); }
  heartbeat() { this.rig.heartbeat(this.now); }
  arm() { this.rig.arm(this.nonce++, this.now); this.publish(); }
  disarm() { this.rig.disarm(this.now); this.publish(); }
  estop() { this.rig.estop(this.now); this.publish(); }
  clearEstop() { this.rig.clearEstop(this.now); this.publish(); }
  fire(box: number, ch: number) { this.rig.fire(box, ch, this.now); this.publish(); }
  loadSequence(triples: number[]) { return this.rig.loadSequence(triples); }
  startSequence() { this.rig.startSequence(this.now); this.publish(); }
  stopSequence() { this.rig.stopSequence(this.now); this.publish(); }
  setConnected(on: boolean) { this.connected = on; }

  start() {
    if (this.timer) return;
    this.last = Date.now();
    this.timer = setInterval(() => {
      const t = Date.now();
      this.now += t - this.last;
      this.last = t;
      if (this.connected) this.rig.heartbeat(this.now);
      this.rig.tick(this.now);
      this.publish();
    }, 50);
  }
  stop() { if (this.timer) { clearInterval(this.timer); this.timer = null; } }

  private publish() {
    const boxes: BoxView[] = [];
    for (let b = 0; b < 2; b++) {
      const channels = [];
      for (let c = 0; c < CHANNELS; c++)
        channels.push({ firing: this.rig.channelFiring(b, c), msLeft: this.rig.channelMsLeft(b, c, this.now) });
      boxes.push({
        id: b, switchOn: this.rig.boxSwitch(b), armed: this.rig.boxState(b) === 1,
        estopped: this.rig.boxEstopped(b), canFire: this.rig.boxCanFire(b, this.now), channels,
      });
    }
    const snap: Snapshot = { now: this.now, seqRunning: this.rig.seqRunning(), boxes };
    snapshot.set(snap);
  }
}
