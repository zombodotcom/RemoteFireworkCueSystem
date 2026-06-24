<script lang="ts">
  import ControllerPanel from "./components/ControllerPanel.svelte";
  import AuthoringPanel from "./components/AuthoringPanel.svelte";
  import FaultBanner from "./components/FaultBanner.svelte";
  let view: "sim" | "author" = "sim";
</script>

<main>
  <FaultBanner />
  <h1>🎆 Firework Simulator</h1>
  <nav>
    <button class:on={view === "sim"} on:click={() => (view = "sim")}>Simulator</button>
    <button class:on={view === "author"} on:click={() => (view = "author")}>Authoring</button>
  </nav>
  <!-- ControllerPanel stays mounted (owns the WASM rig + sim clock); hide it when authoring -->
  <div class:hidden={view !== "sim"}><ControllerPanel /></div>
  {#if view === "author"}<AuthoringPanel />{/if}
</main>

<style>
  :global(body) { background: #111; color: #ddd; font-family: system-ui, sans-serif; margin: 0; }
  main { max-width: 1000px; margin: 0 auto; padding: 24px; }
  nav { display: flex; gap: 6px; margin-bottom: 16px; }
  nav button { background: #222; color: #aaa; border: 1px solid #444; padding: 6px 14px; cursor: pointer; }
  nav button.on { background: #36c; color: #fff; }
  .hidden { display: none; }
</style>
