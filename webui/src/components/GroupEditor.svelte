<script lang="ts">
  import { config } from "../stores";
  let newName = "";
  let nextId = 1;
  function addGroup() {
    const name = newName.trim();
    if (!name) return;
    config.update((c) => ({ ...c, groups: [...c.groups, { id: `g${Date.now()}_${nextId++}`, label: name, members: [] }] }));
    newName = "";
  }
  function rename(id: string, label: string) {
    config.update((c) => ({ ...c, groups: c.groups.map((g) => g.id === id ? { ...g, label } : g) }));
  }
  function remove(id: string) {
    config.update((c) => ({ ...c, groups: c.groups.filter((g) => g.id !== id) }));
  }
  function toggleMember(gid: string, chId: string) {
    config.update((c) => ({
      ...c,
      groups: c.groups.map((g) => g.id !== gid ? g
        : { ...g, members: g.members.includes(chId) ? g.members.filter((m) => m !== chId) : [...g.members, chId] }),
    }));
  }
</script>

<div class="add">
  <input placeholder="New group name" bind:value={newName} on:keydown={(e) => e.key === "Enter" && addGroup()} />
  <button on:click={addGroup}>Add group</button>
</div>

{#each $config.groups as g (g.id)}
  <div class="group">
    <header>
      <input value={g.label} on:input={(e) => rename(g.id, (/** @type {HTMLInputElement} */ (e.target)).value)} />
      <span class="count">{g.members.length} ch</span>
      <button class="del" on:click={() => remove(g.id)}>✕</button>
    </header>
    <div class="members">
      {#each $config.channels as ch (ch.id)}
        <button class="chip" class:on={g.members.includes(ch.id)} on:click={() => toggleMember(g.id, ch.id)} title={ch.label}>
          {ch.boxId}:{ch.channel}
        </button>
      {/each}
    </div>
  </div>
{/each}

<style>
  .add { display: flex; gap: 8px; margin-bottom: 12px; }
  input { background: #1a1a1a; color: #ddd; border: 1px solid #444; padding: 4px; }
  .group { border: 1px solid #444; border-radius: 6px; padding: 8px; margin-bottom: 10px; }
  header { display: flex; gap: 8px; align-items: center; margin-bottom: 6px; }
  .count { color: #888; font-size: 12px; }
  .del { background: #722; color: #fff; border: none; cursor: pointer; }
  .members { display: grid; grid-template-columns: repeat(8, 1fr); gap: 3px; }
  .chip { font-size: 11px; background: #222; color: #888; border: 1px solid #444; cursor: pointer; padding: 2px; }
  .chip.on { background: #161; color: #fff; }
</style>
