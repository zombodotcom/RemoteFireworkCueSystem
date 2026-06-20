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
          <input value={ch.label} on:input={(e) => setLabel(ch.id, (/** @type {HTMLInputElement} */ (e.target)).value)} />
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
