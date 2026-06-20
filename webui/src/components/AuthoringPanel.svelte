<script lang="ts">
  import { config, connection } from "../stores";
  import { exportConfig, importConfig } from "../lib/config";
  import ChannelEditor from "./ChannelEditor.svelte";
  import GroupEditor from "./GroupEditor.svelte";
  import SequenceEditor from "./SequenceEditor.svelte";

  let tab: "channels" | "groups" | "sequences" = "channels";
  let importErr = "";
  let runWarning = "";

  function runTriples(triples: number[]) {
    const c = $connection;
    if (!c) return;
    // NOTE: setSwitch(true) here is a SIMULATOR-ONLY convenience so one click plays a show.
    // The real hardware transport MUST NOT assert the physical arm switch — that is the
    // operator's authority. Do not carry these setSwitch calls into a live LiveConnection.
    c.setSwitch(0, true); c.setSwitch(1, true); c.arm();
    const submitted = Math.floor(triples.length / 3);
    const loaded = c.loadSequence(triples);
    if (loaded < submitted) {
      runWarning = `Show truncated: only ${loaded} of ${submitted} cues fit the device limit.`;
    } else {
      runWarning = "";
    }
    c.startSequence();
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
    const file = (/** @type {HTMLInputElement} */ (e.target)).files?.[0];
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
{#if runWarning}<p class="err">{runWarning}</p>{/if}

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
