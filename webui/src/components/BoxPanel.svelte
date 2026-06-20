<script lang="ts">
  import type { BoxView } from "../stores";
  import ChannelGrid from "./ChannelGrid.svelte";
  export let box: BoxView;
  export let onSwitch: (on: boolean) => void;
  export let onFire: (ch: number) => void;
</script>

<div class="box">
  <header>
    <strong>Box {box.id}</strong>
    <span class="lamp" class:armed={box.armed} class:estop={box.estopped}>
      {box.estopped ? "E-STOP" : box.armed ? "ARMED" : "SAFE"}
    </span>
    <label><input type="checkbox" checked={box.switchOn} on:change={(e) => onSwitch(/** @type {HTMLInputElement} */ (e.target).checked)} /> arm switch</label>
  </header>
  <ChannelGrid {box} {onFire} />
</div>

<style>
  .box { border: 1px solid #555; padding: 10px; border-radius: 6px; background: #1a1a1a; }
  header { display: flex; gap: 10px; align-items: center; margin-bottom: 8px; }
  .lamp { padding: 2px 8px; border-radius: 4px; background: #722; color: #fff; }
  .lamp.armed { background: #161; }
  .lamp.estop { background: #b00; }
</style>
