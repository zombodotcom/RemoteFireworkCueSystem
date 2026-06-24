<script lang="ts">
  import { onMount, onDestroy } from "svelte";
  import { snapshot, config, connection } from "../stores";
  import { createConnection } from "../core/connection";
  import type { SystemConnection } from "../core/connection";
  import BoxPanel from "./BoxPanel.svelte";
  import TransportControls from "./TransportControls.svelte";
  import LogPanel from "./LogPanel.svelte";
  import DiagPanel from "./DiagPanel.svelte";

  let conn: SystemConnection | null = null;

  function labelsFor(boxId: number): string[] {
    const arr = new Array(16).fill("");
    for (const ch of $config.channels) if (ch.boxId === boxId) arr[ch.channel] = ch.label;
    return arr;
  }

  onMount(async () => {
    conn = await createConnection();
    conn.start();
    connection.set(conn);
  });
  onDestroy(() => { conn?.stop(); connection.set(null); });

  // Build a choreographed demo: an alternating left-to-right sweep across both
  // boxes' first 8 channels, then a simultaneous finale burst. Flat
  // [timeMs, boxId, channel] triples (the format rig_load_sequence expects).
  function buildDemo(): number[] {
    const steps: number[] = [];
    let t = 0;
    for (let ch = 0; ch < 8; ch++) {
      steps.push(t, 0, ch); t += 150;   // box 0 sweeps
      steps.push(t, 1, ch); t += 150;   // box 1 follows
    }
    t += 250;                            // brief pause, then finale
    for (const ch of [0, 2, 4, 6]) {
      steps.push(t, 0, ch);              // both boxes fire together
      steps.push(t, 1, ch + 1);
    }
    return steps;                        // 24 cues, well under MAX_SEQ_STEPS (64)
  }

  // The demo arms the rig first so one click reliably plays a show.
  function runDemo() {
    conn?.setSwitch(0, true);
    conn?.setSwitch(1, true);
    conn?.arm();
    conn?.loadSequence(buildDemo());
    conn?.startSequence();
  }
</script>

{#if conn}
  <TransportControls
    onArm={() => conn?.arm()}
    onDisarm={() => conn?.disarm()}
    onEstop={() => conn?.estop()}
    onClearEstop={() => conn?.clearEstop()}
    onRunDemo={runDemo}
    onToggleConnected={(on) => conn?.setConnected(on)}
  />
  <div class="boxes">
    {#each $snapshot.boxes as box (box.id)}
      <BoxPanel
        {box}
        labels={labelsFor(box.id)}
        onSwitch={(on) => conn?.setSwitch(box.id, on)}
        onFire={(ch) => conn?.fire(box.id, ch)}
      />
    {/each}
  </div>
  <p class="status">t={$snapshot.now}ms · sequence {$snapshot.seqRunning ? "running" : "idle"}</p>
  <h3 class="sec">Controller activity</h3>
  <LogPanel />
  <DiagPanel />
{:else}
  <p>Loading WASM core…</p>
{/if}

<style>
  .boxes { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
  .status { color: #888; font-family: monospace; }
  .sec { color: #99a; font-size: 13px; margin: 14px 0 6px; }
</style>
