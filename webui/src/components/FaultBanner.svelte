<script lang="ts">
  import { fault } from "../stores";
</script>

{#if $fault.active}
  <div class="fault">⚠ {$fault.msg || "FAULT"}</div>
{/if}

<style>
  .fault { background: #b00; color: #fff; font-weight: bold; padding: 8px 14px; border-radius: 6px; margin-bottom: 12px; text-align: center; }
</style>
