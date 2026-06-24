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
