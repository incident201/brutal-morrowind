# Brutal Morrowind

Brutal Morrowind brings Dark Messiah of Might and Magic's combat to Morrowind.
It uses Dark Messiah's first-person animations for melee, bows, thrown weapons
and other actions, and adds kicks, directional and charged attacks, parries,
sliding, adrenaline finishers, ragdolls and dismemberment. It is a fork of
[OpenMW 0.51.0](https://openmw.org).

No game files are included. Brutal Morrowind uses the games you own.

The development branch includes changes beyond the published 0.2.1 packages:
NPCs guard against repeated fast attacks, charged blows and kicks open their defence,
and a prolonged player guard invites a charged counterattack. Block impacts drain
both actors' stamina, with a greater cost for the attacker. Other living actors
intercept NPC melee attacks without taking redirected damage.

NPCs direct their weapon or shield towards the incoming attack while keeping
their grip. The first-person camera follows the visible body during movement
and crouching, including when holding a spell. The player's shadow includes
the head and hands in first person.

First-person claymores, battle axes and warhammers use adapted Dark Messiah
sword motion with both hands on the grip. They support directional charged
attacks, quick strikes, weapon parries, wall recoil and timed-parry clinches.

First-person spears and halberds rest on the right with the tip pointing
towards the crosshair. Quick clicks give sharp thrusts along different paths,
even when spaced apart. Both hands keep their grip through the thrust and
return. A charged thrust draws the shaft back, and attacks with a sidestep use
Dark Messiah's staff sweeps. The weapon sways while running and moves through
a wider arc while sprinting.

Staff swings and polearm side sweeps can hit multiple enemies. The first
target in the arc takes the full hit. Later targets take half the base damage
before their own defences apply.

Incoming damage and critical health use the original Dark Messiah vignette.
Damage determines how long the flash lasts, and the critical-health layer
starts below 5 HP.

Release a fully charged melee attack while aiming at a fallen enemy to perform
a ground finisher. Its first-person camera follows the strike's animation. The
Brutal Morrowind settings let you turn ground finishers off or change the health
threshold, which defaults to strictly below 30%.

Melee weapons keep the blood of the people and creatures they strike. It dries
over a few hours, washes off in water or rain, is cleaned by a repair and wears
away within a game day. In first person, sheathing a bloody one-handed sword
with `F` wipes the blade from guard to tip when the player is on the ground,
out of combat and has a free left hand. Blood sprays onto nearby surfaces
and pools around bodies and cuts.
Actors who step in a wet pool leave fading bloody footprints. Outdoor rain
washes the blood away. Weapon and environmental blood have their own switches
in the Brutal Morrowind settings.

Aim at small loose items and press `G` (rebindable) to grab and carry them in one
or two hands while your equipped weapon and shield are hidden and restored after
dropping (`G`) or throwing (`Use` / left click). First-person carry and throw
animations, camera motion and impact sounds come from Dark Messiah. The hands
reach towards the object before lifting it, and a severed head faces the player
when held. Hits stagger NPCs by hit location (head or body). Throwing with full
adrenaline spends the charge to knock living targets into a physical ragdoll.

A nearby NPC charges in response to a guard held for at least four seconds.
A frontal guard can absorb that hit with two ninths of base stamina available.
Stamina regenerates at one third of the normal rate while you hold guard.

NPCs below 30% health fight wounded with Dark Messiah's wounded guard animations.
Their guard gives way easily, blows stagger them, and a charged blow, kick or
shield bash can knock them down. A hit that leaves them below 10% makes them flee
for up to 20 seconds. Argonians and Khajiit keep their
own stance and walk. Both behaviours have switches in the settings.

An NPC left without a weapon in a fight runs to the weapon nearby that suits it
best, whoever owns it, crouches with Dark Messiah's weapon pickup animation (or
leans over a table) and fights on with it in hand. A shield lying right beside it
is taken too. A charged hit on a blocking weapon can stagger and disarm its
owner. Wounded NPCs are more likely to drop the weapon. NPCs release held
weapons and shields on death.

Equip two short blades and sneak up behind an unaware humanoid to ready a
backstab. Click attack or release a charged attack to execute it. Sneak, Short
Blade and both weapons determine its damage. Witnesses who detect the attack
can report it. The backstab has its own switch in Brutal Morrowind settings.

Bows use Dark Messiah's first-person draw, aim and release. Start drawing again
within 1.15 seconds of firing to nock the next arrow faster at any Marksman
skill. An NPC with a ranged weapon can kick a target that comes too close,
provided it has at least 30% stamina. The kick costs 15% of base stamina and
has a ten-second cooldown.

Arrows, bolts and thrown weapons fly in an arc and hit on contact. They can
stick in flesh and wood, or glance off or break against hard surfaces. A
raised shield blocks only the missiles that strike it. Embedded missiles
remain in corpses and dropped shields. You can recover them from the corpse
or pull them out of the shield. Picking up the shield also takes any
missiles still lodged in it.

Thrown weapons use the right-hand preparation, hold and release from Dark
Messiah's dagger throw. Darts and throwing stars use their own models and
consume one item on release.

With full adrenaline, bows and thrown weapons guide a shot to the visible head
nearest the crosshair within a radius of 7% of the screen height. A lock spends
the charge. Time slows just before contact, and a head hit kills with one shot.
Shields and cover can intercept the shot.

Arrows and thrown weapons leave a small blood mark on the NPC surface they hit.
Regular hits deal double damage to the head, normal damage to the torso, and
65% damage to the limbs.

This branch uses **Dark Messiah import recipe 33**. The published 0.2.1
packages use recipe 4. See the [changelog](CHANGELOG.md) for other changes.

## Requirements

* Morrowind.
* Dark Messiah of Might and Magic, for its animations, sounds and effects. Without it the game runs
  with Morrowind's animations and the features that need Dark Messiah's are turned off.
* Linux: Debian 13, Ubuntu 24.04 with updates, Fedora 45 or a newer distribution,
  x86-64 (glibc 2.38+, GCC 14-compatible C++ runtime and desktop OpenGL libraries).
* Windows: Windows 10 or newer, 64-bit.

## Installation

Download the package for your system from the Releases page.

* Linux: unpack `BrutalMorrowind-<version>-Linux-64Bit.tar.gz` anywhere and run
  `brutalmorrowind-launcher` in the unpacked folder.
* Windows: run `BrutalMorrowind-<version>-Windows-x64.exe` and start Brutal Morrowind from the
  Start menu.

On Steam Deck, add the launcher as a non-Steam game and select Steam Linux
Runtime 4.0 as its compatibility tool. The Linux package requires a newer glibc
than the scout, soldier and sniper containers provide. Keep the whole unpacked
folder together; Qt and Boost are bundled.

## First run

1. The launcher opens the installation wizard. Choose your Morrowind folder.
2. Open the **Dark Messiah** page. The launcher looks for the Steam
   single-player installation. If it cannot find it, use **Browse** to select
   the game folder. The importer accepts Steam VPK archives, older GCF archives
   (including archives inside `mm/`) and extracted `mm/` resources. VPK
   directory archives may have any name ending in `_dir.vpk`.
3. Press **Play** and select **Import and play** when prompted. The launcher
   builds the animation, sound and texture data in about a minute, then starts
   the game.

Settings, saves and the imported data are kept apart from OpenMW's: in `~/.config/brutalmorrowind`
and `~/.local/share/brutalmorrowind` on Linux, in `Documents\My Games\Brutal Morrowind` on Windows.

### Updating the Dark Messiah import

Each build loads the completed import for its own recipe from
`dm-assets/<recipe>` inside the user's data folder. When the launcher finds
an older import, choose **Import and play** on the next launch or **Import**
on the **Dark Messiah** page. After a successful import, the launcher removes
older recipe folders from that profile. If you choose **Play without**, the
game uses Morrowind's animations and switches off features that require Dark
Messiah assets.

To import from a terminal on Linux, run the importer from the package you plan
to play. Replace the example game paths with your own.

```sh
recipe=$(./brutalmorrowind-import --recipe)
./brutalmorrowind-import \
  --dark-messiah "/path/to/Dark Messiah Might and Magic Single Player" \
  --morrowind "/path/to/Morrowind/Data Files" \
  --output "${XDG_DATA_HOME:-$HOME/.local/share}/brutalmorrowind/dm-assets/$recipe" \
  --progress
```

## FAQ

### 1. Is this vibe-coded AI slop?

This mod was created with extensive use of AI. You can call it whatever you like if it amuses you.

It's just a small personal project, and I use the tools that I like and find suitable. I'm not trying to deceive anyone or convince anyone to use them.

So if you hate AI or believe that using AI during mod development is unacceptable, please just ignore this project.

### 2. How does it work? Is it one of those AI merge/passthrough mods where you run two games simultaneously?

This is a modified version of the open-source OpenMW engine that uses animations imported from Dark Messiah of Might and Magic. No passthrough.

It runs just like the normal OpenMW engine. Dark Messiah files are only used as a source of animations and resources.

### 3. Do I really need to have my own copy of Dark Messiah? Why?

You need your own installed copy of the game. Animations and resources from Dark Messiah are not included in this repository for obvious legal reasons.

On the first launch, select **Import and play** to build the required data
from your installed games.

### 4. Why do you need these engine modifications? Wasn't Lua scripting enough?

This is a conscious choice. OpenMW's Lua scripting capabilities give you a lot of possibilities, but they were not enough to implement everything planned (e.g. procedural dismemberment) for this mod.

If you think this is the wrong way to do it, feel free to create your own version in whatever way you like.

### 5. Are you planning an MWSE version?

Only OpenMW is supported.

MWSE is great, but it does not provide access to the proprietary Gamebryo engine itself. I chose OpenMW because it's open-source and can be modified directly.

### 6. Can you play through the entire game like in normal Morrowind, or is this just a staged demo?

Yes you can. The full Morrowind game is still here. You can start a new game, create a new character, and play normally.

Apart from the combat modifications, it's just OpenMW.

### 7. What about balance? Doesn't this mod just make your character OP? How does it integrate with Morrowind's RPG mechanics?

This mod definitely breaks the game's balance and makes your character OP.

In the current alpha release, I focused mostly on importing and integrating Dark Messiah's animations and physics. Balance is something to work on in future iterations. It's not there yet.

If you care about natural character progression and balance, it's better to use other combat mods for now.

### 8. I've never played Morrowind before and want to play it for the first time with this mod. Is that a good idea?

I would definitely not recommend that.

This is not some lore-friendly, perfectly balanced mod. Morrowind's RPG system was not designed for this kind of action-style combat. In its current state, this mod will break the game's balance.

Keep that in mind.

### 9. Is the magic system also changed? Do you have any plans for magic?

Melee and ranged combat use the new animations and rules, including
first-person two-handed weapon motion. Magic uses Morrowind's spellcasting
rules.

### 10. I only want ragdoll physics / FPV / (something else) from the mod, but I don't want the other features. Is that possible?

Yes. In the **Scripts → Brutal Morrowind** section of the game settings, you can enable or disable mod features independently.

Basically, with all features disabled, you can get just the normal vanilla openmw experience.

### 11. Is Brutal Morrowind compatible with X mod?

It depends. Brutal Morrowind is just regular OpenMW with some changes.

So content and graphics mods that are compatible with the original OpenMW should work just fine. However, if a mod changes game physics, animations, camera, combat, or controls, there is a risk of conflicts.

You'll have to check compatibility yourself.

## Building from source

Linux, in the same container as the release build (Docker or Podman):

```sh
docker build -t brutal-linux -f CI/brutal/linux.Containerfile CI/brutal
docker run --rm -v "$PWD:/src" -w /src brutal-linux bash CI/brutal/build-linux.sh /src/build-linux /src/dist
```

The package is written to `dist/`.

Windows: the installer is built by `.github/workflows/windows.yml`.

Any other build follows the [OpenMW build instructions](https://wiki.openmw.org/index.php?title=Development_Environment_Setup).
The Dark Messiah importer also needs its Python runtime:

```sh
python3 tools/dm-import/package_python.py <dir>
cmake -DBRUTAL_PYTHON_RUNTIME=<dir> ...
```

## Credits and license

Brutal Morrowind is licensed under the GNU General Public License version 3 (see `LICENSE`).
It is built on OpenMW (see `AUTHORS.md`) and uses code and libraries from the projects listed in
`CREDITS.md`. It was developed with the help of AI.

Morrowind is a trademark of Bethesda Softworks. Dark Messiah of Might and Magic is a trademark of
Ubisoft. Brutal Morrowind is not affiliated with either.
