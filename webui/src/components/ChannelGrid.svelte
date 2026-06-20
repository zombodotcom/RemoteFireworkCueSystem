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
