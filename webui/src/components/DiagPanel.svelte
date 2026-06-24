<script lang="ts">
  import { diag } from "../stores";
  const fmtUptime = (ms: number) => {
    const s = Math.floor(ms / 1000), m = Math.floor(s / 60);
    return m > 0 ? `${m}m ${s % 60}s` : `${s}s`;
  };
</script>

{#if $diag}
  <div class="diag">
    <div><b>uptime</b> {fmtUptime($diag.uptimeMs)}</div>
    <div><b>heap</b> {Math.round($diag.freeHeap / 1024)}k</div>
    <div><b>clients</b> {$diag.apClients}</div>
    <div><b>ack</b> {$diag.lastAckMs}ms</div>
    <div><b>fired</b> {$diag.fired}</div>
    <div><b>acked</b> {$diag.acked}</div>
    <div><b>failed</b> {$diag.failed}</div>
    <div><b>retries</b> {$diag.retries}</div>
  </div>
{/if}

<style>
  .diag { display: grid; grid-template-columns: repeat(4, 1fr); gap: 6px; font-family: monospace; font-size: 12px; color: #cdd; margin-top: 10px; }
  .diag div { background: #15151f; border: 1px solid #2c2c3a; border-radius: 5px; padding: 6px; }
  .diag b { color: #889; display: block; font-size: 10px; text-transform: uppercase; }
</style>
