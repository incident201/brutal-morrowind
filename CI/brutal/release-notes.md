## Melee Combat & Weapons
- **Two-handed weapons** are now functional and use custom attack animations created from the one-handed animations.
- **Spears and halberds** now have dedicated thrust animations for quick attacks and charged attacks with forward/backward steps. Side swings remain shared with staves.
- **Staff and spear attacks** can now hit multiple targets. The first target takes full damage, while subsequent targets take half base damage, mitigated by their defense.
- **Ground finishers:** Added an animated finishing move against downed NPCs, performed with a fully charged attack when the fallen target has less than 30% health.
- **Dual-dagger backstab:** While dual-wielding daggers, you can perform an animated backstab by sneaking up behind an enemy.
- **Blocking & Fatigue:** Holding block now slows down Fatigue regeneration. Both blocking attacks and hitting a blocking target consume additional Fatigue.
- **Adrenaline:** Reworked adrenaline gain formulas.

## Ranged Combat & Projectiles
- **Bows and thrown weapons** now feature animations and mechanics from *Dark Messiah*.
- **Physical projectiles:** Arrows, darts, and throwing stars are now real physical projectiles that fly in an arc, register hits on contact, and react differently to surfaces—sticking into wood, NPC bodies, ground, and fabric, or ricocheting off stone and metal.
- **Defending against projectiles:** Shields and parrying now work against ranged attacks. Arrows stick into wooden shields and bounce off metal ones. Arrows and bolts can also be deflected with a weapon during the parry window.
- **Adrenaline shot:** Arrows fired while in adrenaline mode fly straight for an NPC's head and kill in a single shot.

## Skills & Movement
- **Acrobatics:** Jump height now scales with the Acrobatics skill. The Scroll of Icarian Flight now works and grants the expected super-jump effect.
- **Athletics:** Fatigue regeneration rate and Fatigue drain while sprinting now scale with the Athletics skill.
- **Weapon skills:** Attack damage now scales with the corresponding weapon skill level.
- **Encumbrance:** While over-encumbered, the character can no longer run or jump, but can still walk (as if holding `Alt`).

## NPC AI & Behavior
- **Improved combat reactions:** NPCs respond better to player actions in combat—blocking with weapons or shields against flurries of quick attacks, and using charged attacks to break through the player's block.
- **Wounded state:** NPCs below 30% health become wounded, changing their walk and attack animations, weakening their guard, and making them lose balance more frequently.
- **Fleeing:** A hit that leaves an NPC with less than 10% health can cause them to flee.
- **Weapon pickup:** Disarmed NPCs will search for a suitable weapon nearby and pick it up.
- **Ranged NPC kicks:** NPCs armed with ranged weapons can kick the player away if they get too close.
- **Group flanking:** Groups of NPCs no longer crowd together when attacking the player; instead, they attempt to flank when possible so they don't obstruct each other's attacks.
- **Out-of-combat movement:** Restored the standard walking animation for NPCs when out of combat.

## Physics, Ragdoll & Interaction
- **Object throwing:** Small objects can now be picked up from the ground and thrown. Throw damage scales with momentum, and headshots can knock NPCs down.
- **Death drops:** Standard deaths now always cause NPCs to drop their held weapon, shield, and light source.
- **Dropped weapon physics:** Dropped weapons stay grounded better and scatter less erratically when knocked out of hands.
- **Exhaustion ragdoll:** Exhausted NPCs now collapse into ragdoll instead of playing Morrowind's default animation, getting back up once their Fatigue recovers.

## Camera, Visuals & Audio
- **Combat & movement camera:** Added *Dark Messiah* camera movements for walking, running, jumping, and various combat actions.
- **First-person body tracking:** The first-person camera now follows the visible body when moving and crouching.
- **First-person shadows:** The first-person player shadow now includes the head and arms when player shadows are enabled.
- **DeathCam:** Added a new DeathCam that shows the player's body falling upon death.
- **Screen blood effects:** Added *Dark Messiah* screen blood overlays for the player.
- **Environmental blood:** Slain NPCs bleed out, leaving a pool of blood on the ground, while combat leaves blood splatters on walls.
- **Bloody weapons:** Weapons now get stained with blood. One-handed swords also feature a blood-wiping animation when sheathing (only when no shield is equipped).
- **Player voice:** Added a separate toggle for the player's voice. Female characters now use female *Dark Messiah* vocalizations.

## Settings
- Added configuration options for physical projectiles, object grabbing, NPC wounding and fleeing, weapon pickup, finishers, camera sway, and blood.
- Merged body physics and dropped equipment settings into the **Ragdoll** section.

## Performance & Bug Fixes
- Optimized performance of weapon-to-wall collision checks.
- Fixed delayed impulses and dismemberment occurring after looting a corpse.
- Fixed slicing behavior on open armor.
- Fixed clinch participant positioning on uneven terrain.
- Fixed Argonian and Khajiit movement issues.
