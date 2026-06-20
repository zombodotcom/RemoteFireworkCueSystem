// @ts-ignore - generated single-file ES module, no types
import Module from "./fireworkcore.js";

export interface Rig {
  reset(): void;
  setSwitch(box: number, on: boolean, now: number): void;
  heartbeat(now: number): void;
  arm(nonce: number, now: number): number;
  disarm(now: number): void;
  estop(now: number): void;
  clearEstop(now: number): void;
  fire(box: number, ch: number, now: number): number;
  loadSequence(triples: number[]): number;
  startSequence(now: number): void;
  stopSequence(now: number): void;
  seqRunning(): boolean;
  tick(now: number): void;
  boxState(box: number): number;     // 0 SAFE, 1 ARMED
  boxSwitch(box: number): boolean;
  boxEstopped(box: number): boolean;
  boxCanFire(box: number, now: number): boolean;
  channelFiring(box: number, ch: number): boolean;
  channelMsLeft(box: number, ch: number, now: number): number;
}

export async function loadRig(): Promise<Rig> {
  const m = await Module();
  const N = "number";
  const call = (fn: string, ret: string | null, types: string[], args: any[]) =>
    m.ccall(fn, ret as any, types as any, args);

  return {
    reset: () => call("rig_reset", null, [], []),
    setSwitch: (box, on, now) => call("rig_set_switch", null, [N, N, N], [box, on ? 1 : 0, now]),
    heartbeat: (now) => call("rig_heartbeat", null, [N], [now]),
    arm: (nonce, now) => call("rig_arm", N, [N, N], [nonce, now]),
    disarm: (now) => call("rig_disarm", null, [N], [now]),
    estop: (now) => call("rig_estop", null, [N], [now]),
    clearEstop: (now) => call("rig_clear_estop", null, [N], [now]),
    fire: (box, ch, now) => call("rig_fire", N, [N, N, N], [box, ch, now]),
    loadSequence: (triples) => {
      const count = Math.floor(triples.length / 3);
      const ptr = m._malloc(triples.length * 4);
      for (let i = 0; i < triples.length; i++) m.setValue(ptr + i * 4, triples[i], "i32");
      const loaded = call("rig_load_sequence", N, [N, N], [ptr, count]);
      m._free(ptr);
      return loaded;
    },
    startSequence: (now) => call("rig_start_sequence", null, [N], [now]),
    stopSequence: (now) => call("rig_stop_sequence", null, [N], [now]),
    seqRunning: () => call("rig_seq_running", N, [], []) === 1,
    tick: (now) => call("rig_tick", null, [N], [now]),
    boxState: (box) => call("rig_box_state", N, [N], [box]),
    boxSwitch: (box) => call("rig_box_switch", N, [N], [box]) === 1,
    boxEstopped: (box) => call("rig_box_estopped", N, [N], [box]) === 1,
    boxCanFire: (box, now) => call("rig_box_can_fire", N, [N, N], [box, now]) === 1,
    channelFiring: (box, ch) => call("rig_channel_firing", N, [N, N], [box, ch]) === 1,
    channelMsLeft: (box, ch, now) => call("rig_channel_msleft", N, [N, N, N], [box, ch, now]),
  };
}
