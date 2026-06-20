# Simulator Plan B — Show Authoring Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add show authoring to the firework simulator — label the 32 channels, define groups/macros, and build/save/run timed sequences — all persisted as the same JSON the controller will eventually consume, and runnable on the existing WASM rig.

**Architecture:** A pure, host-testable `config.ts` module owns the show-config schema, validation, persistence, import/export, and the expansion of a saved sequence (with group targets) into the flat `[timeMs, boxId, channel]` triples the rig runs. Svelte editor components mutate a config store; an authoring view sits alongside the existing simulator view. The core sequence cap is raised so realistic shows fit.

**Tech Stack:** TypeScript, Svelte 4, Vite, Vitest (config logic), the existing `fireworkcore` WASM rig + `SimConnection`.

## Global Constraints

- Config is the shape from the simulator spec §5: `{ version, channels:[{id,label,boxId,channel}], groups:[{id,label,members:[channelId]}], sequences:[{id,label,steps:[{timeMs,targetType,targetId}]}] }`.
- 32 channels total: box 0 channels 0–15 = ids `c0`..`c15`; box 1 channels 0–15 = ids `c16`..`c31`.
- A saved sequence expands to flat `[timeMs, boxId, channel]` triples (group target → one triple per member channel) — the format `SimConnection.loadSequence(number[])` already accepts.
- Expansion must be sorted by `timeMs` ascending and preserve group member order within the same time.
- Persistence uses `localStorage` key `fw.show.config`, but every persistence function takes an injectable `Storage` (default `globalThis.localStorage`) so logic is host-testable in the Vitest `node` env.
- Do NOT edit `fireworkcore` core logic except the single `MAX_SEQ_STEPS` constant in Task 1. New web code lives under `webui/src`.
- Generated `webui/src/core/fireworkcore.js`, `node_modules/`, `dist/` stay git-ignored.
- Vitest env is `node`; config tests must not require a DOM.
- `verbatimModuleSyntax` is on: type-only imports MUST use `import type` / inline `type`.

---

## File Structure

```
components/fireworkcore/include/sequence.h     MODIFY: MAX_SEQ_STEPS 64 -> 256
components/fireworkcore/wasm/sim_bindings.{h,cpp}  MODIFY: rig_load_sequence returns int (count loaded)
components/fireworkcore/host_test/test_rig.cpp     MODIFY: test the load count + >64 capacity
webui/src/core/wasm.ts                          MODIFY: loadSequence returns number
webui/src/lib/config.ts                         NEW: schema, validate, persist, import/export, expand
webui/test/config.test.ts                       NEW: Vitest for config.ts
webui/src/stores.ts                             MODIFY: add `config` writable store
webui/src/components/ChannelEditor.svelte       NEW: edit 32 channel labels
webui/src/components/GroupEditor.svelte         NEW: create/edit/delete groups
webui/src/components/SequenceEditor.svelte      NEW: build/save/run sequences
webui/src/components/AuthoringPanel.svelte      NEW: hosts the three editors + import/export
webui/src/components/ChannelGrid.svelte         MODIFY: show channel labels (tooltip)
webui/src/components/BoxPanel.svelte            MODIFY: pass labels through
webui/src/components/ControllerPanel.svelte     MODIFY: derive labels from config; run a chosen saved sequence
webui/src/App.svelte                            MODIFY: tab between Simulator and Authoring
```

---

## Task 1: Raise the sequence cap and report load count

**Files:**
- Modify: `components/fireworkcore/include/sequence.h`
- Modify: `components/fireworkcore/wasm/sim_bindings.h`
- Modify: `components/fireworkcore/wasm/sim_bindings.cpp`
- Modify: `components/fireworkcore/host_test/test_rig.cpp`
- Modify: `webui/src/core/wasm.ts`

**Interfaces:**
- Produces: `int rig_load_sequence(const uint32_t* triples, int count)` now RETURNS the number of steps actually loaded (capped at `MAX_SEQ_STEPS`). `wasm.ts` `loadSequence(triples): number` returns that count.

- [ ] **Step 1: Add a failing host test for capacity + return count**

In `components/fireworkcore/host_test/test_rig.cpp`, add before `main()`:

```cpp
void test_load_sequence_returns_count_and_capacity() {
    rig_reset();
    // 70 cues all at t=0 on box 0, channels cycling 0..15
    uint32_t steps[70 * 3];
    for (int i = 0; i < 70; i++) { steps[i*3+0] = 0; steps[i*3+1] = 0; steps[i*3+2] = (uint32_t)(i % 16); }
    int loaded = rig_load_sequence(steps, 70);   // MAX_SEQ_STEPS is 256, so all 70 load
    CHECK_EQ(loaded, 70);
}
```
And add `RUN(test_load_sequence_returns_count_and_capacity);` in `main()`.

- [ ] **Step 2: Build — verify it FAILS to compile**

Run: `cmake --build build/host_test`
Expected: FAIL — `rig_load_sequence` returns `void`, cannot assign to `int loaded`.

- [ ] **Step 3: Raise the cap**

In `components/fireworkcore/include/sequence.h`, change:
```cpp
static const size_t MAX_SEQ_STEPS = 64;
```
to:
```cpp
static const size_t MAX_SEQ_STEPS = 256;
```

- [ ] **Step 4: Change the rig signature to return the count**

In `components/fireworkcore/wasm/sim_bindings.h`, change the declaration:
```cpp
void rig_load_sequence(const uint32_t* triples, int count);
```
to:
```cpp
int rig_load_sequence(const uint32_t* triples, int count); // returns number of steps loaded
```

In `components/fireworkcore/wasm/sim_bindings.cpp`, change the definition to return the clamped count:
```cpp
int rig_load_sequence(const uint32_t* triples, int count) {
    SeqStep steps[MAX_SEQ_STEPS];
    if (count < 0) count = 0;
    if ((size_t)count > MAX_SEQ_STEPS) count = (int)MAX_SEQ_STEPS;
    for (int i = 0; i < count; i++) {
        steps[i].timeMs  = triples[i * 3 + 0];
        steps[i].boxId   = (uint8_t)triples[i * 3 + 1];
        steps[i].channel = (uint8_t)triples[i * 3 + 2];
    }
    g.scheduler.load(steps, (size_t)count);
    return count;
}
```

- [ ] **Step 5: Build + run — verify host tests PASS**

Run:
```
cmake --build build/host_test
ctest --test-dir build/host_test --output-on-failure
```
Expected: all suites pass, `test_rig` now includes the capacity test.

- [ ] **Step 6: Update the JS wrapper return type and rebuild WASM**

In `webui/src/core/wasm.ts`, change the `Rig` interface method and its implementation:
- Interface: `loadSequence(triples: number[]): number;`
- Implementation:
```ts
    loadSequence: (triples) => {
      const count = Math.floor(triples.length / 3);
      const ptr = m._malloc(triples.length * 4);
      for (let i = 0; i < triples.length; i++) m.setValue(ptr + i * 4, triples[i], "i32");
      const loaded = call("rig_load_sequence", N, [N, N], [ptr, count]);
      m._free(ptr);
      return loaded;
    },
```
Then rebuild the module and run the web smoke tests:
```
bash webui/scripts/build-wasm.sh
cd webui && npm run test
```
Expected: build prints `built ...`; 2 Vitest smoke tests still pass.

- [ ] **Step 7: Commit**

```bash
git add components/fireworkcore/include/sequence.h components/fireworkcore/wasm/sim_bindings.h components/fireworkcore/wasm/sim_bindings.cpp components/fireworkcore/host_test/test_rig.cpp webui/src/core/wasm.ts
git commit -m "feat(sim): raise MAX_SEQ_STEPS to 256 and report load count"
```

---

## Task 2: Show-config module (schema, validate, persist, import/export, expand)

**Files:**
- Create: `webui/src/lib/config.ts`
- Create: `webui/test/config.test.ts`

**Interfaces:**
- Produces:
  - Types: `ChannelCfg { id:string; label:string; boxId:number; channel:number }`, `GroupCfg { id:string; label:string; members:string[] }`, `SeqStepCfg { timeMs:number; targetType:"channel"|"group"; targetId:string }`, `SequenceCfg { id:string; label:string; steps:SeqStepCfg[] }`, `ShowConfig { version:number; channels:ChannelCfg[]; groups:GroupCfg[]; sequences:SequenceCfg[] }`.
  - `defaultConfig(): ShowConfig` — 32 channels (c0..c31), no groups, no sequences.
  - `validateConfig(cfg: ShowConfig): string[]` — list of human-readable problems (empty = valid).
  - `saveConfig(cfg, storage?): void` / `loadConfig(storage?): ShowConfig`.
  - `exportConfig(cfg): string` / `importConfig(json: string): ShowConfig` (throws `Error` on invalid).
  - `expandSequence(cfg: ShowConfig, seqId: string): number[]` — flat `[timeMs,boxId,channel]` triples, sorted by time.

- [ ] **Step 1: Write the failing tests**

`webui/test/config.test.ts`:
```ts
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
      .toThrow();
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
});
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd webui && npx vitest run test/config.test.ts`
Expected: FAIL — cannot import from `../src/lib/config`.

- [ ] **Step 3: Implement `config.ts`**

`webui/src/lib/config.ts`:
```ts
export interface ChannelCfg { id: string; label: string; boxId: number; channel: number; }
export interface GroupCfg { id: string; label: string; members: string[]; }
export interface SeqStepCfg { timeMs: number; targetType: "channel" | "group"; targetId: string; }
export interface SequenceCfg { id: string; label: string; steps: SeqStepCfg[]; }
export interface ShowConfig {
  version: number;
  channels: ChannelCfg[];
  groups: GroupCfg[];
  sequences: SequenceCfg[];
}

const STORAGE_KEY = "fw.show.config";
const CHANNELS_PER_BOX = 16;
const BOXES = 2;

export function defaultConfig(): ShowConfig {
  const channels: ChannelCfg[] = [];
  for (let boxId = 0; boxId < BOXES; boxId++) {
    for (let channel = 0; channel < CHANNELS_PER_BOX; channel++) {
      const idx = boxId * CHANNELS_PER_BOX + channel;
      channels.push({ id: `c${idx}`, label: `Box ${boxId} · Ch ${channel}`, boxId, channel });
    }
  }
  return { version: 1, channels, groups: [], sequences: [] };
}

export function validateConfig(cfg: ShowConfig): string[] {
  const errors: string[] = [];
  const ids = new Set<string>();
  for (const ch of cfg.channels) {
    if (ids.has(ch.id)) errors.push(`Duplicate channel id "${ch.id}"`);
    ids.add(ch.id);
    if (ch.boxId < 0 || ch.boxId >= BOXES) errors.push(`Channel "${ch.id}" has invalid box ${ch.boxId}`);
    if (ch.channel < 0 || ch.channel >= CHANNELS_PER_BOX) errors.push(`Channel "${ch.id}" has invalid channel ${ch.channel}`);
  }
  const groupIds = new Set<string>();
  for (const g of cfg.groups) {
    if (groupIds.has(g.id)) errors.push(`Duplicate group id "${g.id}"`);
    groupIds.add(g.id);
    for (const m of g.members) if (!ids.has(m)) errors.push(`Group "${g.label}" references unknown channel "${m}"`);
  }
  for (const seq of cfg.sequences) {
    for (const step of seq.steps) {
      if (step.timeMs < 0) errors.push(`Sequence "${seq.label}" has a negative time`);
      if (step.targetType === "channel" && !ids.has(step.targetId))
        errors.push(`Sequence "${seq.label}" references unknown channel "${step.targetId}"`);
      if (step.targetType === "group" && !groupIds.has(step.targetId))
        errors.push(`Sequence "${seq.label}" references unknown group "${step.targetId}"`);
    }
  }
  return errors;
}

export function expandSequence(cfg: ShowConfig, seqId: string): number[] {
  const seq = cfg.sequences.find((s) => s.id === seqId);
  if (!seq) return [];
  const chById = new Map(cfg.channels.map((c) => [c.id, c]));
  const grpById = new Map(cfg.groups.map((g) => [g.id, g]));
  // Stable sort by time; preserve original order for equal times.
  const ordered = seq.steps.map((s, i) => ({ s, i }))
    .sort((a, b) => a.s.timeMs - b.s.timeMs || a.i - b.i)
    .map((x) => x.s);
  const out: number[] = [];
  const pushChannel = (timeMs: number, channelId: string) => {
    const ch = chById.get(channelId);
    if (ch) out.push(timeMs, ch.boxId, ch.channel);
  };
  for (const step of ordered) {
    if (step.targetType === "channel") pushChannel(step.timeMs, step.targetId);
    else {
      const g = grpById.get(step.targetId);
      if (g) for (const m of g.members) pushChannel(step.timeMs, m);
    }
  }
  return out;
}

export function exportConfig(cfg: ShowConfig): string {
  return JSON.stringify(cfg, null, 2);
}

export function importConfig(json: string): ShowConfig {
  const cfg = JSON.parse(json) as ShowConfig;
  const errors = validateConfig(cfg);
  if (errors.length) throw new Error("Invalid config: " + errors.join("; "));
  return cfg;
}

export function saveConfig(cfg: ShowConfig, storage: Storage = globalThis.localStorage): void {
  storage.setItem(STORAGE_KEY, JSON.stringify(cfg));
}

export function loadConfig(storage: Storage = globalThis.localStorage): ShowConfig {
  const raw = storage.getItem(STORAGE_KEY);
  if (!raw) return defaultConfig();
  try {
    const cfg = JSON.parse(raw) as ShowConfig;
    if (validateConfig(cfg).length) return defaultConfig();
    return cfg;
  } catch {
    return defaultConfig();
  }
}
```

- [ ] **Step 4: Run to verify PASS**

Run: `cd webui && npx vitest run test/config.test.ts`
Expected: all 8 tests pass.

- [ ] **Step 5: Commit**

```bash
git add webui/src/lib/config.ts webui/test/config.test.ts
git commit -m "feat(sim): show-config schema, validation, persistence, expand (tested)"
```

---

## Task 3: Config store + channel-label editing shown in the rig

**Files:**
- Modify: `webui/src/stores.ts`
- Create: `webui/src/components/ChannelEditor.svelte`
- Modify: `webui/src/components/ChannelGrid.svelte`
- Modify: `webui/src/components/BoxPanel.svelte`
- Modify: `webui/src/components/ControllerPanel.svelte`

**Interfaces:**
- Consumes: `config.ts` (`loadConfig`, `saveConfig`, `ShowConfig`, `ChannelCfg`).
- Produces: `config` writable store in `stores.ts`; `ChannelGrid`/`BoxPanel` accept a `labels: string[]` prop (16 labels for that box).

- [ ] **Step 1: Add the config store and persist on change**

In `webui/src/stores.ts`, append:
```ts
import { loadConfig, saveConfig, type ShowConfig } from "./lib/config";

export const config = writable<ShowConfig>(loadConfig());
config.subscribe((c) => { try { saveConfig(c); } catch { /* storage unavailable */ } });
```
(Keep the existing `snapshot` store and imports.)

- [ ] **Step 2: ChannelEditor — edit all 32 labels**

`webui/src/components/ChannelEditor.svelte`:
```svelte
<script lang="ts">
  import { config } from "../stores";
  import type { ChannelCfg } from "../lib/config";
  function setLabel(id: string, label: string) {
    config.update((c) => ({ ...c, channels: c.channels.map((ch) => ch.id === id ? { ...ch, label } : ch) }));
  }
  $: byBox = (b: number): ChannelCfg[] => $config.channels.filter((c) => c.boxId === b);
</script>

<div class="cols">
  {#each [0, 1] as b}
    <div>
      <h3>Box {b}</h3>
      {#each byBox(b) as ch (ch.id)}
        <label class="row">
          <span class="num">Ch {ch.channel}</span>
          <input value={ch.label} on:input={(e) => setLabel(ch.id, (e.target as HTMLInputElement).value)} />
        </label>
      {/each}
    </div>
  {/each}
</div>

<style>
  .cols { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
  .row { display: flex; align-items: center; gap: 8px; margin: 2px 0; }
  .num { color: #888; width: 48px; font-family: monospace; font-size: 12px; }
  input { flex: 1; background: #1a1a1a; color: #ddd; border: 1px solid #444; padding: 4px; }
</style>
```

- [ ] **Step 3: ChannelGrid shows labels as tooltips**

Replace `webui/src/components/ChannelGrid.svelte` with:
```svelte
<script lang="ts">
  import type { BoxView } from "../stores";
  export let box: BoxView;
  export let labels: string[] = [];
  export let onFire: (ch: number) => void;
</script>

<div class="grid">
  {#each box.channels as ch, i}
    <button class="cell" class:firing={ch.firing} on:click={() => onFire(i)} title={labels[i] ?? `ch ${i}`}>
      {i}
    </button>
  {/each}
</div>

<style>
  .grid { display: grid; grid-template-columns: repeat(8, 1fr); gap: 4px; }
  .cell { aspect-ratio: 1; border: 1px solid #444; background: #222; color: #888; cursor: pointer; }
  .cell.firing { background: gold; color: #000; box-shadow: 0 0 10px gold; }
</style>
```

- [ ] **Step 4: BoxPanel passes labels through**

In `webui/src/components/BoxPanel.svelte`, add a `labels` prop and forward it. Change the `<script>` to add `export let labels: string[] = [];` and the `<ChannelGrid {box} {onFire} />` usage to `<ChannelGrid {box} {labels} {onFire} />`.

- [ ] **Step 5: ControllerPanel derives per-box labels from config**

In `webui/src/components/ControllerPanel.svelte`, add to the `<script>`:
```ts
  import { config } from "../stores";
  function labelsFor(boxId: number): string[] {
    const arr = new Array(16).fill("");
    for (const ch of $config.channels) if (ch.boxId === boxId) arr[ch.channel] = ch.label;
    return arr;
  }
```
And change the `<BoxPanel ... />` usage to pass labels:
```svelte
      <BoxPanel
        {box}
        labels={labelsFor(box.id)}
        onSwitch={(on) => conn?.setSwitch(box.id, on)}
        onFire={(ch) => conn?.fire(box.id, ch)}
      />
```

- [ ] **Step 6: Verify build + the existing tests**

Run:
```
cd webui && npx vite build 2>&1 | tail -3
npm run test
```
Expected: build succeeds; the 2 smoke tests + 8 config tests pass (10 total).

- [ ] **Step 7: Commit**

```bash
git add webui/src/stores.ts webui/src/components/ChannelEditor.svelte webui/src/components/ChannelGrid.svelte webui/src/components/BoxPanel.svelte webui/src/components/ControllerPanel.svelte
git commit -m "feat(sim): config store + channel-label editor, labels shown on rig"
```

---

## Task 4: Group editor

**Files:**
- Create: `webui/src/components/GroupEditor.svelte`

**Interfaces:**
- Consumes: `config` store, `GroupCfg`.

- [ ] **Step 1: GroupEditor — create, rename, delete groups; toggle members**

`webui/src/components/GroupEditor.svelte`:
```svelte
<script lang="ts">
  import { config } from "../stores";
  let newName = "";
  let nextId = 1;
  function addGroup() {
    const name = newName.trim();
    if (!name) return;
    config.update((c) => ({ ...c, groups: [...c.groups, { id: `g${Date.now()}_${nextId++}`, label: name, members: [] }] }));
    newName = "";
  }
  function rename(id: string, label: string) {
    config.update((c) => ({ ...c, groups: c.groups.map((g) => g.id === id ? { ...g, label } : g) }));
  }
  function remove(id: string) {
    config.update((c) => ({ ...c, groups: c.groups.filter((g) => g.id !== id) }));
  }
  function toggleMember(gid: string, chId: string) {
    config.update((c) => ({
      ...c,
      groups: c.groups.map((g) => g.id !== gid ? g
        : { ...g, members: g.members.includes(chId) ? g.members.filter((m) => m !== chId) : [...g.members, chId] }),
    }));
  }
</script>

<div class="add">
  <input placeholder="New group name" bind:value={newName} on:keydown={(e) => e.key === "Enter" && addGroup()} />
  <button on:click={addGroup}>Add group</button>
</div>

{#each $config.groups as g (g.id)}
  <div class="group">
    <header>
      <input value={g.label} on:input={(e) => rename(g.id, (e.target as HTMLInputElement).value)} />
      <span class="count">{g.members.length} ch</span>
      <button class="del" on:click={() => remove(g.id)}>✕</button>
    </header>
    <div class="members">
      {#each $config.channels as ch (ch.id)}
        <button class="chip" class:on={g.members.includes(ch.id)} on:click={() => toggleMember(g.id, ch.id)} title={ch.label}>
          {ch.boxId}:{ch.channel}
        </button>
      {/each}
    </div>
  </div>
{/each}

<style>
  .add { display: flex; gap: 8px; margin-bottom: 12px; }
  input { background: #1a1a1a; color: #ddd; border: 1px solid #444; padding: 4px; }
  .group { border: 1px solid #444; border-radius: 6px; padding: 8px; margin-bottom: 10px; }
  header { display: flex; gap: 8px; align-items: center; margin-bottom: 6px; }
  .count { color: #888; font-size: 12px; }
  .del { background: #722; color: #fff; border: none; cursor: pointer; }
  .members { display: grid; grid-template-columns: repeat(8, 1fr); gap: 3px; }
  .chip { font-size: 11px; background: #222; color: #888; border: 1px solid #444; cursor: pointer; padding: 2px; }
  .chip.on { background: #161; color: #fff; }
</style>
```

- [ ] **Step 2: Verify build**

Run: `cd webui && npx vite build 2>&1 | tail -3`
Expected: build succeeds.

- [ ] **Step 3: Commit**

```bash
git add webui/src/components/GroupEditor.svelte
git commit -m "feat(sim): group/macro editor"
```

---

## Task 5: Sequence editor (build, save, run)

**Files:**
- Create: `webui/src/components/SequenceEditor.svelte`

**Interfaces:**
- Consumes: `config` store, `expandSequence`, and a `run`/`stop` callback prop wired to `SimConnection`.

- [ ] **Step 1: SequenceEditor — manage a sequence's steps and run it**

`webui/src/components/SequenceEditor.svelte`:
```svelte
<script lang="ts">
  import { config } from "../stores";
  import { expandSequence } from "../lib/config";
  export let onRun: (triples: number[]) => void;
  export let onStop: () => void;

  let selId = "";
  let nextId = 1;
  $: sequences = $config.sequences;
  $: selected = sequences.find((s) => s.id === selId) ?? null;

  // new-step form
  let stepTime = 0;
  let stepKind: "channel" | "group" = "channel";
  let stepTarget = "";

  function addSequence() {
    const id = `s${Date.now()}_${nextId++}`;
    config.update((c) => ({ ...c, sequences: [...c.sequences, { id, label: `Show ${c.sequences.length + 1}`, steps: [] }] }));
    selId = id;
  }
  function rename(label: string) {
    config.update((c) => ({ ...c, sequences: c.sequences.map((s) => s.id === selId ? { ...s, label } : s) }));
  }
  function removeSeq(id: string) {
    config.update((c) => ({ ...c, sequences: c.sequences.filter((s) => s.id !== id) }));
    if (selId === id) selId = "";
  }
  function addStep() {
    if (!selected || !stepTarget) return;
    const step = { timeMs: Math.max(0, Math.floor(stepTime)), targetType: stepKind, targetId: stepTarget };
    config.update((c) => ({ ...c, sequences: c.sequences.map((s) => s.id === selId ? { ...s, steps: [...s.steps, step] } : s) }));
  }
  function removeStep(idx: number) {
    config.update((c) => ({ ...c, sequences: c.sequences.map((s) => s.id === selId ? { ...s, steps: s.steps.filter((_, i) => i !== idx) } : s) }));
  }
  function labelFor(kind: "channel" | "group", id: string): string {
    if (kind === "channel") return $config.channels.find((c) => c.id === id)?.label ?? id;
    return $config.groups.find((g) => g.id === id)?.label ?? id;
  }
  function run() { if (selected) onRun(expandSequence($config, selected.id)); }
</script>

<div class="bar">
  <select bind:value={selId}>
    <option value="">— pick a show —</option>
    {#each sequences as s (s.id)}<option value={s.id}>{s.label}</option>{/each}
  </select>
  <button on:click={addSequence}>New show</button>
  {#if selected}
    <button on:click={() => removeSeq(selected.id)}>Delete</button>
    <button on:click={run}>▶ Run</button>
    <button on:click={onStop}>■ Stop</button>
  {/if}
</div>

{#if selected}
  <label class="name">Name <input value={selected.label} on:input={(e) => rename((e.target as HTMLInputElement).value)} /></label>

  <ol class="steps">
    {#each selected.steps as st, i}
      <li><span class="t">{st.timeMs}ms</span> {st.targetType}: {labelFor(st.targetType, st.targetId)} <button class="del" on:click={() => removeStep(i)}>✕</button></li>
    {/each}
  </ol>

  <div class="addstep">
    <input type="number" min="0" step="50" bind:value={stepTime} /> ms
    <select bind:value={stepKind} on:change={() => (stepTarget = "")}>
      <option value="channel">channel</option>
      <option value="group">group</option>
    </select>
    <select bind:value={stepTarget}>
      <option value="">— target —</option>
      {#if stepKind === "channel"}
        {#each $config.channels as c (c.id)}<option value={c.id}>{c.label}</option>{/each}
      {:else}
        {#each $config.groups as g (g.id)}<option value={g.id}>{g.label}</option>{/each}
      {/if}
    </select>
    <button on:click={addStep}>Add cue</button>
  </div>
{/if}

<style>
  .bar, .addstep { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; margin: 8px 0; }
  select, input { background: #1a1a1a; color: #ddd; border: 1px solid #444; padding: 4px; }
  .name input { width: 200px; }
  .steps { font-family: monospace; font-size: 13px; }
  .steps li { margin: 2px 0; }
  .t { color: #6cf; display: inline-block; width: 70px; }
  .del { background: #722; color: #fff; border: none; cursor: pointer; }
</style>
```

- [ ] **Step 2: Verify build**

Run: `cd webui && npx vite build 2>&1 | tail -3`
Expected: build succeeds.

- [ ] **Step 3: Commit**

```bash
git add webui/src/components/SequenceEditor.svelte
git commit -m "feat(sim): sequence builder with run/stop via expansion"
```

---

## Task 6: Authoring panel, view tabs, import/export wiring

**Files:**
- Create: `webui/src/components/AuthoringPanel.svelte`
- Modify: `webui/src/components/ControllerPanel.svelte`
- Modify: `webui/src/App.svelte`

**Interfaces:**
- Consumes: ChannelEditor, GroupEditor, SequenceEditor, `config`, `exportConfig`/`importConfig`; a run/stop wired to a `SimConnection`.

- [ ] **Step 1: Expose run/stop sequence on ControllerPanel via a shared SimConnection**

The simplest wiring: `SimConnection` is created in `ControllerPanel`. To let the AuthoringPanel run sequences on the same rig, lift the connection into a store. In `webui/src/stores.ts` append:
```ts
import type { SimConnection } from "./core/connection";
export const connection = writable<SimConnection | null>(null);
```
In `ControllerPanel.svelte`, after `conn = await SimConnection.create()` and `conn.start()`, add `connection.set(conn)` (import `connection` from `../stores`), and in `onDestroy` add `connection.set(null)`.

- [ ] **Step 2: AuthoringPanel — hosts editors + import/export, runs on the shared connection**

`webui/src/components/AuthoringPanel.svelte`:
```svelte
<script lang="ts">
  import { config, connection } from "../stores";
  import { exportConfig, importConfig } from "../lib/config";
  import ChannelEditor from "./ChannelEditor.svelte";
  import GroupEditor from "./GroupEditor.svelte";
  import SequenceEditor from "./SequenceEditor.svelte";

  let tab: "channels" | "groups" | "sequences" = "channels";
  let importErr = "";

  function runTriples(triples: number[]) {
    const c = $connection;
    if (!c) return;
    c.setSwitch(0, true); c.setSwitch(1, true); c.arm();
    c.loadSequence(triples); c.startSequence();
  }
  function stop() { $connection?.stopSequence(); }

  function doExport() {
    const blob = new Blob([exportConfig($config)], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url; a.download = "firework-show.json"; a.click();
    URL.revokeObjectURL(url);
  }
  async function doImport(e: Event) {
    importErr = "";
    const file = (e.target as HTMLInputElement).files?.[0];
    if (!file) return;
    try { config.set(importConfig(await file.text())); }
    catch (err) { importErr = (err as Error).message; }
  }
</script>

<div class="tabs">
  <button class:on={tab === "channels"} on:click={() => (tab = "channels")}>Channels</button>
  <button class:on={tab === "groups"} on:click={() => (tab = "groups")}>Groups</button>
  <button class:on={tab === "sequences"} on:click={() => (tab = "sequences")}>Sequences</button>
  <span class="spacer"></span>
  <button on:click={doExport}>Export JSON</button>
  <label class="import">Import JSON <input type="file" accept="application/json" on:change={doImport} /></label>
</div>
{#if importErr}<p class="err">{importErr}</p>{/if}

{#if tab === "channels"}<ChannelEditor />{/if}
{#if tab === "groups"}<GroupEditor />{/if}
{#if tab === "sequences"}<SequenceEditor onRun={runTriples} onStop={stop} />{/if}

<style>
  .tabs { display: flex; gap: 6px; align-items: center; margin-bottom: 12px; }
  .tabs .spacer { flex: 1; }
  .tabs button { background: #222; color: #aaa; border: 1px solid #444; padding: 4px 10px; cursor: pointer; }
  .tabs button.on { background: #2a3; color: #000; }
  .import input { display: none; }
  .import { background: #234; color: #cde; border: 1px solid #456; padding: 4px 10px; cursor: pointer; }
  .err { color: #f88; }
</style>
```

- [ ] **Step 3: App — tab between Simulator and Authoring**

Replace `webui/src/App.svelte`:
```svelte
<script lang="ts">
  import ControllerPanel from "./components/ControllerPanel.svelte";
  import AuthoringPanel from "./components/AuthoringPanel.svelte";
  let view: "sim" | "author" = "sim";
</script>

<main>
  <h1>🎆 Firework Simulator</h1>
  <nav>
    <button class:on={view === "sim"} on:click={() => (view = "sim")}>Simulator</button>
    <button class:on={view === "author"} on:click={() => (view = "author")}>Authoring</button>
  </nav>
  <!-- ControllerPanel stays mounted (owns the WASM rig + sim clock); hide it when authoring -->
  <div class:hidden={view !== "sim"}><ControllerPanel /></div>
  {#if view === "author"}<AuthoringPanel />{/if}
</main>

<style>
  :global(body) { background: #111; color: #ddd; font-family: system-ui, sans-serif; margin: 0; }
  main { max-width: 1000px; margin: 0 auto; padding: 24px; }
  nav { display: flex; gap: 6px; margin-bottom: 16px; }
  nav button { background: #222; color: #aaa; border: 1px solid #444; padding: 6px 14px; cursor: pointer; }
  nav button.on { background: #36c; color: #fff; }
  .hidden { display: none; }
</style>
```

> ControllerPanel stays mounted (just hidden) when authoring so the WASM rig and sim-clock keep running and a sequence launched from the authoring tab plays live — switch back to Simulator to watch it.

- [ ] **Step 4: Build + full test run**

Run:
```
cd webui && npx vite build 2>&1 | tail -3
npm run test
```
Expected: build succeeds; all Vitest tests pass (2 smoke + 8 config).

- [ ] **Step 5: Manual verification (browser)**

Run `bash webui/scripts/build-wasm.sh && cd webui && npm run dev`, open the URL:
1. **Authoring → Channels:** rename a channel; switch to **Simulator** and hover that channel cell → tooltip shows the new label.
2. **Authoring → Groups:** add a group, toggle a few channels into it.
3. **Authoring → Sequences:** New show → add cues (mix channel + group targets at different times) → **Run** → switch to Simulator and watch the cues fire on schedule.
4. **Export JSON** downloads the show; reload the page (config persisted via localStorage) and confirm your channels/groups/sequences are still there; **Import JSON** restores an exported file.

- [ ] **Step 6: Commit**

```bash
git add webui/src/stores.ts webui/src/components/AuthoringPanel.svelte webui/src/components/ControllerPanel.svelte webui/src/App.svelte
git commit -m "feat(sim): authoring panel, view tabs, JSON import/export"
```

---

## Done criteria

- Host tests + Vitest all green (`ctest` 6 suites; `npm run test` 10 tests).
- You can label channels, define groups, build a timed sequence mixing channel + group cues, and run it on the rig — watching the real interlocks.
- Config persists across reloads (localStorage) and round-trips through JSON export/import.
- The show JSON is exactly the schema the controller firmware will consume (spec §5).
- No `fireworkcore` logic changed except `MAX_SEQ_STEPS`.

## Notes for later (not in this plan)

- The exported show JSON is the handoff artifact to the future controller firmware (Plan 3) — it will load the same schema from NVS and run the same expansion.
- A future "live" transport (`LiveConnection` implementing `SystemConnection`) can drive a real ESP32 controller with no UI changes.
