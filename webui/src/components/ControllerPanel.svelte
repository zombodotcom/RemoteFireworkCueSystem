<script lang="ts">
  import { onMount, onDestroy } from "svelte";
  import { snapshot } from "../stores";
  import { SimConnection } from "../core/connection";
  import BoxPanel from "./BoxPanel.svelte";
  import TransportControls from "./TransportControls.svelte";

  let conn: SimConnection | null = null;

  onMount(async () => {
    conn = await SimConnection.create();
    conn.start();
  });
  onDestroy(() => conn?.stop());

  // A small demo sequence: box0 ch0 @0, box0 ch1 @600, box1 ch5 @1200 (flat [timeMs,box,ch] triples)
  const demo = [0, 0, 0,  600, 0, 1,  1200, 1, 5];
  function runDemo() { conn?.loadSequence(demo); conn?.startSequence(); }
</script>

{#if conn}
  <TransportControls
    onArm={() => conn!.arm()}
    onDisarm={() => conn!.disarm()}
    onEstop={() => conn!.estop()}
    onClearEstop={() => conn!.clearEstop()}
    onRunDemo={runDemo}
    onToggleConnected={(on) => conn!.setConnected(on)}
  />
  <div class="boxes">
    {#each $snapshot.boxes as box (box.id)}
      <BoxPanel
        {box}
        onSwitch={(on) => conn!.setSwitch(box.id, on)}
        onFire={(ch) => conn!.fire(box.id, ch)}
      />
    {/each}
  </div>
  <p class="status">t={$snapshot.now}ms · sequence {$snapshot.seqRunning ? "running" : "idle"}</p>
{:else}
  <p>Loading WASM core…</p>
{/if}

<style>
  .boxes { display: grid; grid-template-columns: 1fr 1fr; gap: 16px; }
  .status { color: #888; font-family: monospace; }
</style>
