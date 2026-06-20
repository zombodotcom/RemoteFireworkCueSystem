import { describe, it, expect } from "vitest";
import { loadRig } from "../src/core/wasm";

describe("wasm rig", () => {
  it("arms and fires a channel", async () => {
    const rig = await loadRig();
    rig.reset();
    expect(rig.boxState(0)).toBe(0);          // SAFE
    rig.setSwitch(0, true, 0);
    rig.heartbeat(0);
    expect(rig.arm(1, 0)).toBe(1);            // only box 0 armed
    expect(rig.boxState(0)).toBe(1);          // ARMED
    expect(rig.fire(0, 3, 0)).toBe(1);
    expect(rig.channelFiring(0, 3)).toBe(true);
    rig.tick(401);                            // pulse expires after FIRE_MS
    expect(rig.channelFiring(0, 3)).toBe(false);
  });

  it("rejects fire when disarmed", async () => {
    const rig = await loadRig();
    rig.reset();
    expect(rig.fire(0, 0, 0)).toBe(0);
    expect(rig.channelFiring(0, 0)).toBe(false);
  });
});
