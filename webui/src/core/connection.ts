import { loadRig, type Rig } from "./wasm";
import { snapshot, diag, fault, events as eventsStore, type Snapshot, type BoxView } from "../stores";
import { mergeEvents } from "../lib/events";
import { LiveConnection } from "./live_connection";

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
  private evSeq = 0;

  static async create(): Promise<SimConnection> {
    const c = new SimConnection();
    c.rig = await loadRig();
    c.rig.reset();
    c.publish();
    return c;
  }

  setSwitch(box: number, on: boolean) { this.rig.setSwitch(box, on, this.now); this.publish(); }
  heartbeat() { this.rig.heartbeat(this.now); }
  arm() { this.rig.arm(this.nonce++, this.now); this.emit(1, "ARM -> box0"); this.publish(); }
  disarm() { this.rig.disarm(this.now); this.emit(0, "DISARM"); this.publish(); }
  estop() { this.rig.estop(this.now); this.emit(2, "ESTOP"); this.publish(); }
  clearEstop() { this.rig.clearEstop(this.now); this.publish(); }
  fire(box: number, ch: number) { this.rig.fire(box, ch, this.now); this.emit(0, `FIRE ch${ch} -> box${box}`); this.publish(); }
  loadSequence(triples: number[]) { return this.rig.loadSequence(triples); }
  startSequence() { this.rig.startSequence(this.now); this.emit(0, "SEQ start"); this.publish(); }
  stopSequence() { this.rig.stopSequence(this.now); this.publish(); }
  setConnected(on: boolean) { this.connected = on; }

  private emit(sev: 0 | 1 | 2, msg: string) {
    const ev = { seq: ++this.evSeq, t: this.now, sev, msg };
    eventsStore.update((prev) => mergeEvents(prev, [ev]));
  }

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
        channels.push({ firing: this.rig.channelFiring(b, c), msLeft: this.rig.channelMsLeft(b, c, this.now), fired: this.rig.channelFiring(b, c) });
      boxes.push({
        id: b, switchOn: this.rig.boxSwitch(b), armed: this.rig.boxState(b) === 1,
        estopped: this.rig.boxEstopped(b), canFire: this.rig.boxCanFire(b, this.now),
        linkAlive: this.connected, rssi: null, channels,
      });
    }
    const snap: Snapshot = { now: this.now, seqRunning: this.rig.seqRunning(), boxes };
    snapshot.set(snap);
    const anyEstop = this.rig.boxEstopped(0) || this.rig.boxEstopped(1);
    diag.set({
      uptimeMs: this.now, freeHeap: 140000, apClients: this.connected ? 1 : 0,
      fired: 0, acked: 0, failed: 0, retries: 0, lastAckMs: 0,
    });
    fault.set(anyEstop ? { active: true, msg: "ESTOP" } : { active: false, msg: "" });
  }
}

// ── Factory ────────────────────────────────────────────────────────────────
// Returns a SimConnection when running on the Vite dev server (import.meta.env.DEV),
// and a LiveConnection when served from the ESP32 controller (production build).
export async function createConnection(): Promise<SystemConnection> {
  if (import.meta.env.DEV) {
    return SimConnection.create();
  }
  const live = new LiveConnection();
  return live;
}
