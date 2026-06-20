import { describe, it, expect } from "vitest";
import {
  defaultConfig, validateConfig, expandSequence, exportConfig, importConfig,
  saveConfig, loadConfig, type ShowConfig,
} from "../src/lib/config";

function fakeStorage(): Storage {
  const m = new Map<string, string>();
  return {
    get length() { return m.size; },
    clear: () => m.clear(),
    getItem: (k) => (m.has(k) ? m.get(k)! : null),
    key: (i) => [...m.keys()][i] ?? null,
    removeItem: (k) => void m.delete(k),
    setItem: (k, v) => void m.set(k, v),
  };
}

describe("config", () => {
  it("default has 32 channels mapped to two boxes", () => {
    const c = defaultConfig();
    expect(c.channels).toHaveLength(32);
    expect(c.channels[0]).toMatchObject({ id: "c0", boxId: 0, channel: 0 });
    expect(c.channels[16]).toMatchObject({ id: "c16", boxId: 1, channel: 0 });
    expect(c.channels[31]).toMatchObject({ id: "c31", boxId: 1, channel: 15 });
    expect(validateConfig(c)).toEqual([]);
  });

  it("validate flags a group member that does not exist", () => {
    const c = defaultConfig();
    c.groups.push({ id: "g1", label: "G", members: ["c0", "nope"] });
    expect(validateConfig(c).join(" ")).toContain("nope");
  });

  it("validate flags a sequence target that does not exist", () => {
    const c = defaultConfig();
    c.sequences.push({ id: "s1", label: "S", steps: [{ timeMs: 0, targetType: "channel", targetId: "zzz" }] });
    expect(validateConfig(c).join(" ")).toContain("zzz");
  });

  it("expands a channel step to one triple", () => {
    const c = defaultConfig();
    c.sequences.push({ id: "s1", label: "S", steps: [{ timeMs: 500, targetType: "channel", targetId: "c17" }] });
    // c17 = box 1, channel 1
    expect(expandSequence(c, "s1")).toEqual([500, 1, 1]);
  });

  it("expands a group step to one triple per member, sorted by time", () => {
    const c = defaultConfig();
    c.groups.push({ id: "g1", label: "Finale", members: ["c0", "c16"] }); // box0 ch0, box1 ch0
    c.sequences.push({ id: "s1", label: "S", steps: [
      { timeMs: 1000, targetType: "group", targetId: "g1" },
      { timeMs: 0, targetType: "channel", targetId: "c1" },     // box0 ch1, earlier
    ]});
    expect(expandSequence(c, "s1")).toEqual([
      0, 0, 1,        // c1 first (t=0)
      1000, 0, 0,     // g1 member c0
      1000, 1, 0,     // g1 member c16
    ]);
  });

  it("round-trips through export/import", () => {
    const c = defaultConfig();
    c.channels[0].label = "Left mortar";
    const json = exportConfig(c);
    const back = importConfig(json);
    expect(back.channels[0].label).toBe("Left mortar");
  });

  it("import throws on invalid config", () => {
    expect(() => importConfig('{"version":1,"channels":[],"groups":[],"sequences":[{"id":"s","label":"x","steps":[{"timeMs":0,"targetType":"channel","targetId":"missing"}]}]}'))
      .toThrow(Error);
  });

  it("save then load returns the same config via injected storage", () => {
    const s = fakeStorage();
    const c = defaultConfig();
    c.channels[5].label = "Roman candle";
    saveConfig(c, s);
    expect(loadConfig(s).channels[5].label).toBe("Roman candle");
  });

  it("load returns default when storage is empty", () => {
    const s = fakeStorage();
    expect(loadConfig(s).channels).toHaveLength(32);
  });

  it("loadConfig returns default on corrupt JSON", () => {
    const s = fakeStorage();
    s.setItem("fw.show.config", "{not valid json");
    expect(loadConfig(s).channels).toHaveLength(32);
  });

  it("validate flags a negative step time", () => {
    const c = defaultConfig();
    c.sequences.push({ id: "s1", label: "S", steps: [{ timeMs: -5, targetType: "channel", targetId: "c0" }] });
    expect(validateConfig(c).join(" ").toLowerCase()).toContain("negative");
  });

  it("validate flags duplicate sequence ids", () => {
    const c = defaultConfig();
    c.sequences.push({ id: "dup", label: "A", steps: [] });
    c.sequences.push({ id: "dup", label: "B", steps: [] });
    expect(validateConfig(c).join(" ")).toContain("Duplicate sequence id");
  });
});
