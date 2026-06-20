<script lang="ts">
  export let onArm: () => void;
  export let onDisarm: () => void;
  export let onEstop: () => void;
  export let onClearEstop: () => void;
  export let onRunDemo: () => void;
  export let onToggleConnected: (on: boolean) => void;
  let connected = true;
</script>

<div class="bar">
  <button on:click={onArm}>ARM</button>
  <button on:click={onDisarm}>Disarm</button>
  <button class="estop" on:click={onEstop}>E-STOP</button>
  <button on:click={onClearEstop}>Clear E-STOP</button>
  <button on:click={onRunDemo}>Run demo sequence</button>
  <label><input type="checkbox" bind:checked={connected} on:change={() => onToggleConnected(connected)} /> phone connected</label>
</div>

<style>
  .bar { display: flex; gap: 8px; align-items: center; margin: 12px 0; flex-wrap: wrap; }
  .estop { background: #b00; color: #fff; font-weight: bold; }
</style>
