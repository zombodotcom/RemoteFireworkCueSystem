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
  <label class="name">Name <input value={selected.label} on:input={(e) => rename((/** @type {HTMLInputElement} */ (e.target)).value)} /></label>

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
