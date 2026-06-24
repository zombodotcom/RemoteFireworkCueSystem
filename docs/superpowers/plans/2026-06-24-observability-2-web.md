# Observability Plan 2 — Website (Log + Diagnostics + Fault)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Show the controller's event log, diagnostics, and faults on the web UI — a scrolling color-coded log, a diagnostics stat panel, and a fault banner — fed by Plan 1's `/api/events` + extended `/api/status`.

**Architecture:** Three new Svelte stores (`events`, `diag`, `fault`) populated by the connection layer. `LiveConnection` parses `diag`/`fault` from `/api/status` and incrementally polls `/api/events?since=<maxSeq>`. `SimConnection` emits synthetic events/diag so the simulator demonstrates the panels. A pure `mergeEvents` helper (vitest-tested) does the incremental dedup. Three new components render the stores.

**Tech Stack:** Vite + Svelte + TypeScript, vitest. Built to a single file embedded in the controller.

## Global Constraints

- Web build/test in **Git Bash** from `webui`: `npm run build` (vite + tsc type-check), `npm test` (vitest). Both must pass.
- Re-embed the built UI for the controller: `gzip -9 -c dist/index.html > ../controller/main/www/index.html.gz`, then rebuild the controller (PowerShell `idf.py build`). The `.gz` is git-ignored — commit only `webui/src` sources.
- Event shape (from Plan 1 `/api/events`): `{seq:number, t:number, sev:0|1|2, msg:string}`; severities info=0/warn=1/err=2. `/api/status` adds `diag:{uptimeMs,freeHeap,apClients,fired,acked,failed,retries,lastAckMs}` and `fault:{active,msg}` (all existing keys unchanged).
- Changing shared store types means every producer (`LiveConnection`, `SimConnection`) must compile — `npm run build` is the gate.
- Read-only display; no new control actions.

---

## File Structure
- `webui/src/lib/events.ts` — pure `LogEvent` type + `mergeEvents` + `maxSeq` (Task 1).
- `webui/test/events.test.ts` — vitest for the merge logic (Task 1).
- `webui/src/stores.ts` — add `events`/`diag`/`fault` stores + `Diag`/`Fault` types (Task 1).
- `webui/src/core/live_connection.ts` — parse diag/fault, poll `/api/events` (Task 2).
- `webui/src/core/connection.ts` — `SimConnection` synthetic events/diag/fault (Task 3).
- `webui/src/components/{FaultBanner,LogPanel,DiagPanel}.svelte` — render (Task 4).
- `webui/src/App.svelte`, `webui/src/components/ControllerPanel.svelte` — wire in (Task 4).

---

### Task 1: Stores + pure `mergeEvents` (vitest TDD)

**Files:**
- Create: `webui/src/lib/events.ts`, `webui/test/events.test.ts`
- Modify: `webui/src/stores.ts`

**Interfaces:**
- Produces: `interface LogEvent { seq:number; t:number; sev:0|1|2; msg:string }`; `function mergeEvents(existing:LogEvent[], incoming:LogEvent[], cap?:number):LogEvent[]` (dedup by seq, sorted ascending, cap keeps newest, default cap 200); `function maxSeq(events:LogEvent[]):number` (0 if empty). Stores: `events` (`LogEvent[]`), `diag` (`Diag|null`), `fault` (`Fault`). `interface Diag {uptimeMs;freeHeap;apClients;fired;acked;failed;retries;lastAckMs:number}`; `interface Fault {active:boolean; msg:string}`.

- [ ] **Step 1: Write the failing test**

`webui/test/events.test.ts`:
```ts
import { describe, it, expect } from "vitest";
import { mergeEvents, maxSeq, type LogEvent } from "../src/lib/events";

const e = (seq: number, sev: 0 | 1 | 2 = 0): LogEvent => ({ seq, t: seq * 10, sev, msg: `m${seq}` });

describe("mergeEvents", () => {
  it("appends new events sorted by seq", () => {
    const r = mergeEvents([e(1), e(2)], [e(3), e(4)]);
    expect(r.map((x) => x.seq)).toEqual([1, 2, 3, 4]);
  });
  it("dedups by seq (incoming wins, no duplicates)", () => {
    const r = mergeEvents([e(1), e(2)], [e(2), e(3)]);
    expect(r.map((x) => x.seq)).toEqual([1, 2, 3]);
  });
  it("caps to the newest N", () => {
    const existing = [e(1), e(2), e(3)];
    const r = mergeEvents(existing, [e(4), e(5)], 3);
    expect(r.map((x) => x.seq)).toEqual([3, 4, 5]);
  });
  it("maxSeq returns the highest seq, 0 when empty", () => {
    expect(maxSeq([e(2), e(5), e(3)])).toBe(5);
    expect(maxSeq([])).toBe(0);
  });
});
```

- [ ] **Step 2: Run to verify it fails**

Run (Git Bash): `cd webui && npm test`
Expected: FAIL — `../src/lib/events` cannot be resolved.

- [ ] **Step 3: Implement `events.ts`**

`webui/src/lib/events.ts`:
```ts
export type Severity = 0 | 1 | 2;
export interface LogEvent { seq: number; t: number; sev: Severity; msg: string; }

/** Merge incoming events into existing: dedup by seq, sort ascending, keep newest `cap`. */
export function mergeEvents(existing: LogEvent[], incoming: LogEvent[], cap = 200): LogEvent[] {
  const bySeq = new Map<number, LogEvent>();
  for (const ev of existing) bySeq.set(ev.seq, ev);
  for (const ev of incoming) bySeq.set(ev.seq, ev);   // incoming wins on collision
  const merged = [...bySeq.values()].sort((a, b) => a.seq - b.seq);
  return merged.length > cap ? merged.slice(merged.length - cap) : merged;
}

export function maxSeq(events: LogEvent[]): number {
  let m = 0;
  for (const ev of events) if (ev.seq > m) m = ev.seq;
  return m;
}
```

- [ ] **Step 4: Add the stores**

In `webui/src/stores.ts`, add the import and stores:
```ts
import type { LogEvent } from "./lib/events";

export interface Diag {
  uptimeMs: number; freeHeap: number; apClients: number;
  fired: number; acked: number; failed: number; retries: number; lastAckMs: number;
}
export interface Fault { active: boolean; msg: string; }

export const events = writable<LogEvent[]>([]);
export const diag = writable<Diag | null>(null);
export const fault = writable<Fault>({ active: false, msg: "" });
```

- [ ] **Step 5: Run tests to verify pass**

Run (Git Bash): `cd webui && npm test`
Expected: all suites pass (config + wasm.smoke + new events: 4 new cases green).

- [ ] **Step 6: Commit**

```bash
git add webui/src/lib/events.ts webui/test/events.test.ts webui/src/stores.ts
git commit -m "feat(web): event/diag/fault stores + pure mergeEvents (vitest)"
```

---

### Task 2: LiveConnection — parse diag/fault + poll /api/events

**Files:**
- Modify: `webui/src/core/live_connection.ts`

**Interfaces:**
- Consumes: `events`/`diag`/`fault` stores, `mergeEvents`/`maxSeq`/`LogEvent` (Task 1).
- Produces: live `/api/events` incremental polling → `events` store; `diag`/`fault` from `/api/status`.

- [ ] **Step 1: Extend imports + StatusResponse**

At the top of `webui/src/core/live_connection.ts`, extend the stores import and add types:
```ts
import { snapshot, diag, fault, events as eventsStore, type Diag, type Fault } from "../stores";
import { mergeEvents, maxSeq, type LogEvent } from "../lib/events";
```
Extend `StatusResponse`:
```ts
interface StatusResponse {
  armed: boolean;
  seqRunning: boolean;
  lastFailedBox: number | null;
  boxes?: BoxStatus[];
  diag?: Diag;
  fault?: Fault;
}
interface EventsResponse { lastSeq: number; events: LogEvent[]; }
```

- [ ] **Step 2: Track maxSeq + poll events**

Add a field next to the other private fields:
```ts
  private lastSeq = 0;
```
Add an events poll method (after `pollStatus`):
```ts
  private async pollEvents(): Promise<void> {
    try {
      const res = await fetch(`/api/events?since=${this.lastSeq}`);
      if (!res.ok) return;
      const data = (await res.json()) as EventsResponse;
      if (data.events && data.events.length) {
        let merged: LogEvent[] = [];
        eventsStore.update((prev) => (merged = mergeEvents(prev, data.events)));
        this.lastSeq = maxSeq(merged);
      } else if (typeof data.lastSeq === "number") {
        this.lastSeq = Math.max(this.lastSeq, data.lastSeq);
      }
    } catch { /* leave events stale on network error */ }
  }
```

- [ ] **Step 3: Set diag/fault in applyStatus; poll events on the timer**

In `applyStatus`, after the `snapshot.set(...)` call, add:
```ts
    if (s.diag) diag.set(s.diag);
    if (s.fault) fault.set(s.fault);
```
In `start()`, change the status interval to poll events too, and add an immediate events poll:
```ts
    this.statusTimer = setInterval(() => {
      void this.pollStatus();
      void this.pollEvents();
    }, STATUS_POLL_INTERVAL_MS);

    void this.pollStatus();
    void this.pollEvents();
```

- [ ] **Step 4: Type-check / build**

Run (Git Bash): `cd webui && npm run build`
Expected: clean build (no TS errors). Then `npm test` — existing + events tests still green.

- [ ] **Step 5: Commit**

```bash
git add webui/src/core/live_connection.ts
git commit -m "feat(web): LiveConnection parses diag/fault + polls /api/events incrementally"
```

---

### Task 3: SimConnection — synthetic events/diag/fault

**Files:**
- Modify: `webui/src/core/connection.ts`

**Interfaces:**
- Consumes: `events`/`diag`/`fault` stores, `mergeEvents` (Task 1).
- Produces: the simulator pushes synthetic log events on arm/disarm/estop/fire/startSequence and publishes a synthetic `diag` + `fault` each tick, so the web panels are populated in DEV.

- [ ] **Step 1: Imports + a tiny event emitter**

At the top of `webui/src/core/connection.ts`, extend the stores import:
```ts
import { snapshot, diag, fault, events as eventsStore, type Snapshot, type BoxView } from "../stores";
import { mergeEvents } from "../lib/events";
```
In `class SimConnection`, add fields:
```ts
  private evSeq = 0;
```
Add a private helper:
```ts
  private emit(sev: 0 | 1 | 2, msg: string) {
    const ev = { seq: ++this.evSeq, t: this.now, sev, msg };
    eventsStore.update((prev) => mergeEvents(prev, [ev]));
  }
```

- [ ] **Step 2: Emit on actions**

Update the action methods to emit (keep the existing rig calls + publish):
```ts
  arm() { this.rig.arm(this.nonce++, this.now); this.emit(1, "ARM -> box0"); this.publish(); }
  disarm() { this.rig.disarm(this.now); this.emit(0, "DISARM"); this.publish(); }
  estop() { this.rig.estop(this.now); this.emit(2, "ESTOP"); this.publish(); }
  fire(box: number, ch: number) { this.rig.fire(box, ch, this.now); this.emit(0, `FIRE ch${ch} -> box${box}`); this.publish(); }
  startSequence() { this.rig.startSequence(this.now); this.emit(0, "SEQ start"); this.publish(); }
```
(Leave `clearEstop`, `loadSequence`, `stopSequence`, `setSwitch`, `heartbeat` as they are.)

- [ ] **Step 3: Publish synthetic diag/fault**

At the end of `publish()`, after `snapshot.set(snap)`, add:
```ts
    const anyEstop = this.rig.boxEstopped(0) || this.rig.boxEstopped(1);
    diag.set({
      uptimeMs: this.now, freeHeap: 140000, apClients: this.connected ? 1 : 0,
      fired: 0, acked: 0, failed: 0, retries: 0, lastAckMs: 0,
    });
    fault.set(anyEstop ? { active: true, msg: "ESTOP" } : { active: false, msg: "" });
```

- [ ] **Step 4: Build + test**

Run (Git Bash): `cd webui && npm run build && npm test`
Expected: clean build, all tests green.

- [ ] **Step 5: Commit**

```bash
git add webui/src/core/connection.ts
git commit -m "feat(web): SimConnection emits synthetic events/diag/fault for the panels"
```

---

### Task 4: Components + wire-in + re-embed

**Files:**
- Create: `webui/src/components/FaultBanner.svelte`, `webui/src/components/LogPanel.svelte`, `webui/src/components/DiagPanel.svelte`
- Modify: `webui/src/App.svelte`, `webui/src/components/ControllerPanel.svelte`
- Rebuild: `controller/main/www/index.html.gz` (artifact)

**Interfaces:**
- Consumes: `events`/`diag`/`fault` stores (Task 1).

- [ ] **Step 1: FaultBanner**

`webui/src/components/FaultBanner.svelte`:
```svelte
<script lang="ts">
  import { fault } from "../stores";
</script>

{#if $fault.active}
  <div class="fault">⚠ {$fault.msg || "FAULT"}</div>
{/if}

<style>
  .fault { background: #b00; color: #fff; font-weight: bold; padding: 8px 14px; border-radius: 6px; margin-bottom: 12px; text-align: center; }
</style>
```

- [ ] **Step 2: LogPanel**

`webui/src/components/LogPanel.svelte`:
```svelte
<script lang="ts">
  import { afterUpdate } from "svelte";
  import { events } from "../stores";
  let box: HTMLDivElement;
  let stick = true;
  function onScroll() {
    stick = box.scrollHeight - box.scrollTop - box.clientHeight < 30;
  }
  afterUpdate(() => { if (stick && box) box.scrollTop = box.scrollHeight; });
  const sevClass = (s: number) => (s === 2 ? "err" : s === 1 ? "warn" : "info");
</script>

<div class="log" bind:this={box} on:scroll={onScroll}>
  {#each $events as ev (ev.seq)}
    <div class="row {sevClass(ev.sev)}"><span class="t">{(ev.t / 1000).toFixed(1)}s</span> {ev.msg}</div>
  {/each}
  {#if $events.length === 0}<div class="row info">no events yet…</div>{/if}
</div>

<style>
  .log { height: 180px; overflow-y: auto; background: #0c0c12; border: 1px solid #333; border-radius: 6px; padding: 6px; font-family: monospace; font-size: 12px; }
  .row { padding: 1px 2px; white-space: nowrap; }
  .t { color: #667; margin-right: 6px; }
  .info { color: #9aa; }
  .warn { color: #e5bb33; }
  .err  { color: #ff6b6b; }
</style>
```

- [ ] **Step 3: DiagPanel**

`webui/src/components/DiagPanel.svelte`:
```svelte
<script lang="ts">
  import { diag } from "../stores";
  const fmtUptime = (ms: number) => {
    const s = Math.floor(ms / 1000), m = Math.floor(s / 60);
    return m > 0 ? `${m}m ${s % 60}s` : `${s}s`;
  };
</script>

{#if $diag}
  <div class="diag">
    <div><b>uptime</b> {fmtUptime($diag.uptimeMs)}</div>
    <div><b>heap</b> {Math.round($diag.freeHeap / 1024)}k</div>
    <div><b>clients</b> {$diag.apClients}</div>
    <div><b>ack</b> {$diag.lastAckMs}ms</div>
    <div><b>fired</b> {$diag.fired}</div>
    <div><b>acked</b> {$diag.acked}</div>
    <div><b>failed</b> {$diag.failed}</div>
    <div><b>retries</b> {$diag.retries}</div>
  </div>
{/if}

<style>
  .diag { display: grid; grid-template-columns: repeat(4, 1fr); gap: 6px; font-family: monospace; font-size: 12px; color: #cdd; margin-top: 10px; }
  .diag div { background: #15151f; border: 1px solid #2c2c3a; border-radius: 5px; padding: 6px; }
  .diag b { color: #889; display: block; font-size: 10px; text-transform: uppercase; }
</style>
```

- [ ] **Step 4: Wire into App + ControllerPanel**

In `webui/src/App.svelte`, import and place the fault banner at the top of `<main>` (above `<h1>`):
```svelte
  import FaultBanner from "./components/FaultBanner.svelte";
```
and in the markup, first child of `<main>`:
```svelte
  <FaultBanner />
```
In `webui/src/components/ControllerPanel.svelte`, import the panels:
```svelte
  import LogPanel from "./LogPanel.svelte";
  import DiagPanel from "./DiagPanel.svelte";
```
and add them after the `<p class="status">…</p>` line (inside the `{#if conn}` block):
```svelte
  <h3 class="sec">Controller activity</h3>
  <LogPanel />
  <DiagPanel />
```
Add to ControllerPanel's `<style>`:
```css
  .sec { color: #99a; font-size: 13px; margin: 14px 0 6px; }
```

- [ ] **Step 5: Build, test, re-embed, rebuild controller**

Run (Git Bash):
```bash
cd webui && npm run build && npm test && gzip -9 -c dist/index.html > ../controller/main/www/index.html.gz
```
Expected: clean build, all vitest green, `.gz` regenerated. Then rebuild the controller (PowerShell):
```powershell
& 'C:\esp\v6.0.1\esp-idf\export.ps1'; Set-Location 'C:\Users\zombo\Desktop\Programming\RemoteFireworkCueSystem\RemoteFireworkCueSystem\controller'; idf.py build
```
Expected: `Project build complete.`

- [ ] **Step 6: Commit**

```bash
git add webui/src/components/FaultBanner.svelte webui/src/components/LogPanel.svelte webui/src/components/DiagPanel.svelte webui/src/App.svelte webui/src/components/ControllerPanel.svelte
git commit -m "feat(web): fault banner + activity log + diagnostics panels"
```
(`controller/main/www/index.html.gz` is git-ignored — do not add it.)

---

## Self-Review Notes
- **Spec coverage:** log panel (Task 4) fed incrementally (Task 2) with dedup (Task 1); diag panel (Task 4) from `/api/status.diag` (Task 2); fault banner (Task 4) from `fault` (Task 2); sim demonstrates all three (Task 3); pure merge logic vitest-tested (Task 1). Read-only.
- **Type consistency:** `LogEvent`/`Diag`/`Fault` identical across stores (Task 1), LiveConnection (Task 2), SimConnection (Task 3), components (Task 4); `mergeEvents`/`maxSeq` signatures match Task 1 defs.
- **Backward compat:** `StatusResponse.diag`/`fault` optional — older firmware (no diag) leaves `diag` null and the panel hidden; existing snapshot mapping unchanged.
