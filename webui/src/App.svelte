<script lang="ts">
  import ControllerPanel from "./components/ControllerPanel.svelte";
</script>

<main>
  <h1>🎆 Firework Simulator</h1>
  <p class="sub">Virtual rig running the real <code>fireworkcore</code> via WASM — no hardware.</p>
  <ControllerPanel />
</main>

<style>
  :global(body) { background: #111; color: #ddd; font-family: system-ui, sans-serif; margin: 0; }
  main { max-width: 900px; margin: 0 auto; padding: 24px; }
  .sub { color: #999; }
</style>
