#!/usr/bin/env python3
"""
check_multi_intent_scope.py — the repository's single scope gate.

This absorbs the old `check-relevant-scope.py`. The two hooks asked the same
question ("is this change one thing?") and answered it twice: one counted files
against an arbitrary threshold, the other clustered directories. A count is not
evidence of scope creep — a rename touches sixty files with one intent, and two
files in unrelated subsystems are two intents. So this hook does exactly two
things:

  * BLOCKS the one objective violation — staged edits to trees this repository
    vendors but does not own.
  * REPORTS the changed-file list, grouped by where those files live, and hands
    the judgement to the agent. No threshold, no guessing.
"""

import os
import re
import subprocess
import sys

# Hook may be invoked from .agents/ (Antigravity sets cwd to the hooks.json
# directory) — always operate from the repository root.
_ROOT = subprocess.run(
    ["git", "rev-parse", "--show-toplevel"], capture_output=True, text=True
).stdout.strip()
if _ROOT:
    os.chdir(_ROOT)

# --- this repository's layout, discovered at bootstrap (generated) ---------
# ONE source for the folder lists. Several hooks used to carry their own
# hardcoded copies of a vendor-directory list and of a default-branch list,
# which was both duplication and wrong: a firmware repository vendors into its
# own SDK directory and protects a release branch under a project-specific
# name, and no hardcoded copy could know either.
#
# Every value below comes from the investigation the bootstrap ran against THIS
# repository — not from a default list. Re-run the bootstrap with `--update`
# after the layout changes.

#: Trees this repository consumes but does not own. Never reformat or edit.
VENDORED_PREFIXES: tuple[str, ...] = ()

#: Branches nobody may commit to directly. Discovered from the remote's own
#: protection settings via `gh`, falling back to the detected base branch.
PROTECTED_BRANCHES: tuple[str, ...] = (
    '0.44',
    '10220-mental_support',
    '15.06',
    '15.10',
    '2.1',
    '2.3',
    '2.4',
    '2.5',
    '2.6',
    '2.7',
    '3.0',
    '3.1',
    '3.2',
    '3.3',
    '3.4',
    '3.5',
    '3.6',
    '4.0',
    '4.1',
    '4.10',
    '4.11',
    '4.12',
    '4.13',
    '4.2',
    '4.3',
    '4.4',
    '4.5',
    '4.6',
    '4.7',
    '4.8',
    '4.9',
    '5.0',
    '5.1',
    '5.10',
    '5.11',
    '5.12',
    '5.13',
    '5.14',
    '5.2',
    '5.3',
    '5.4',
    '5.5',
    '5.6',
    '5.7',
    '5.8',
    '5.9',
    'CURA-10201_fill-narrow-skin-with-walls',
    'CURA-10220_moral_support',
    'CURA-10255_add_unit_tests',
    'CURA-10255_voronoi_fixes',
    'CURA-10347_no_support_for_narrow_ridges',
    'CURA-10348_Revert_boost_fix',
    'CURA-10500_polygon_subdiv',
    'CURA-10683_no-interface-for-tree-support',
    'CURA-10748',
    'CURA-10854_tree_xyz_override_fix',
    'CURA-10914_allow_multiple_plugins_on_same_slot',
    'CURA-10993_simple_prime_tower_raft',
    'CURA-11019_speedup_ugly_version',
    'CURA-11157_remove_support_interface_skip_height_remove_all_references',
    'CURA-11360_test_wagyu_fix__dont_merge',
    'CURA-11361-svg-wkt-formatters',
    'CURA-11542_optimized_prime_tower',
    'CURA-11597_fix_multiple_support_lines',
    'CURA-11622_conan_v2',
    'CURA-11630_avoid_seam_locations',
    'CURA-11630_avoid_seam_locations_smarter',
    'CURA-11649',
    'CURA-11696-spiral-z-hop',
    'CURA-11735-support-interface-layer',
    'CURA-11830_smart_seam_unretract',
    'CURA-11834-ambient-occlusion',
    'CURA-11834-ambient-occlusion-faces',
    'CURA-11887_refuzzed_minispike',
    'CURA-11966_handover_code_reduce_sprint_overhang',
    'CURA-12065_proposed_fix_extruder_vs_mesh_retract',
    'CURA-12074_introduce-bambu-printers',
    'CURA-12080_better-seam',
    'CURA-12236_test_revert',
    'CURA-12250_refactor-the-post-processing-like-algorithms',
    'CURA-12304_Wipe-movements-should-have-a-limit-Wipe-nozzle-between-layers-moves-off-the-build-plate',
    'CURA-12446_top_bottom_wall_count',
    'CURA-12446_top_bottom_wall_count_new',
    'CURA-12446_top_bottom_wall_count_v3',
    'CURA-12449_handling-painted-models',
    'CURA-12460_dual_layer_time_spike_dont_merge',
    'CURA-12490_fix_spdlog_on_npm_package',
    'CURA-12580_paint-on-support',
    'CURA-12634_panda_painting_alpha',
    'CURA-12774_zits_when_intermittent_roofing',
    'CURA-12777',
    'CURA-12793_multi-material-overlapping-models-with-painted-data-is-not-handled-as-expected',
    'CURA-12851',
    'CURA-12873',
    'CURA-12890-Layer_time_message',
    'CURA-12961_auto_brim_spike',
    'CURA-12976_apply-inside-travel-avoid-distance-within-same-feature',
    'CURA-13172',
    'CURA-13183_sturdier-tree-support',
    'CURA-13215',
    'CURA-13279',
    'CURA-13280_fix_tool_path_line_width_generation',
    'CURA-13287_fix_microseg_regress',
    'CURA-13287_fix_microseg_regress_test',
    'CURA-13291_sharpen_bridging_conditions',
    'CURA-13293',
    'CURA-13335_flooring-over-support-not-bridging',
    'CURA-13353_wrong-bridging-lines-direction',
    'CURA-7557-inset-optimiser-use',
    'CURA-7948_experiments',
    'CURA-7948_remove_singular_nodes',
    'CURA-7970_voronoi_graph_integer_rounding_experiments',
    'CURA-8636_rdp_debug',
    'CURA-8737_debug',
    'CURA-9052_introuduce-maximum-travel-deviation',
    'CURA-9178_crash_voronoi',
    'CURA-9178_crash_voronoi_debug',
    'CURA-9178_port_external_voronoi_fixes',
    'CURA-9178_temp_sidebranch',
    'CURA-9295_Overextrusion_when_printing_with_gradual_infill_',
    'CURA-9296_Travel_trough_model_wehn_printing_PVA',
    'CURA-9377_debug',
    'CURA-9377_fix_transitions_out_of_range',
    'CURA-9540_insert_temp_mid_polygon',
    'CURA-9790-reconfigure-gradual-infill',
    'CURA-9839-add-bridging-and-overhang-line-type-color-scheme',
    'NP-1234-make-paint-from-neoprep-slicable',
    'NP-1347',
    'NP-1357_harden_ci_script_execution',
    'NP-1361',
    'NP-31_CuraEngine_Benchmark',
    'NP-334_run_CuraEngine_wasm_multithreaded',
    'NP-43_integrate_wasm_curaengine',
    'NP-637_conan_v2_curaengine_5_9',
    'NP-861',
    'PPQ-41_randomize_layer_speed',
    'PPQ-6',
    'UMH-2021_ribbed_vaults_infill__rerooting',
    'UMH-2022_litening_support',
    'UMH-2025_id_label_paint',
    'a_d_revert',
    'add-useful-variables-to-gcode',
    'add-useful-variables-to-gcode-v2',
    'add_more_variables_to_resolvable_gcode',
    'automerge_main_workflow',
    'brim_per_material',
    'combing-polygon-type',
    'copilot/cura-13353-wrong-bridging-lines-direction',
    'copilot/cura-13353-wrong-bridging-lines-direction-again',
    'cura_10724',
    'dependabot/pip/pyjwt-2.4.0',
    'feature/curaengine-update',
    'feature_estimate_dissolving_time',
    'flow_advance',
    'fractal_dithering',
    'gh-pages',
    'hackaton_color_power',
    'immediate_visualization',
    'interlocking_carving',
    'jellespijker-patch-1',
    'legacy',
    'main',
    'merge_crash_fix_into_polygon_rework',
    'optimize_polyline_order',
    'pva_support_fix',
    'sparselinegrid_insert_cleanup',
    'temp_umh2022_branch',
    'test-linter',
    'test-unit-test',
    'texture_processing_color_model',
    'volumetric_prop',
)

#: The PR base for this repository, recorded once so no script has to guess.
BASE_BRANCH: str = "main"

#: Directories holding a published interface whose docs must move with it.
INTERFACE_PREFIXES: tuple[str, ...] = (
    'src/gcode_export/',
)

#: Where this repository documents that interface.
API_DOC_PATHS: tuple[str, ...] = ()

#: Sources where a raw #RRGGBB literal belongs in a theme token instead.
#: Not QML-only: React, Python UIs and stylesheets hardcode colours too.
THEMEABLE_SUFFIXES: tuple[str, ...] = (
    '.qml',
    '.py',
    '.css',
    '.scss',
    '.less',
)

#: The theme/token definitions themselves — the one place literals belong.
THEME_DEFINITION_FILES: tuple[str, ...] = (
    'Theme.qml',
    'theme.ts',
    'tokens.css',
)


def is_vendored(path: str) -> bool:
    return any(path.startswith(prefix) for prefix in VENDORED_PREFIXES)


def is_themeable_source(path: str) -> bool:
    return (path.endswith(THEMEABLE_SUFFIXES)
            and not any(name in path for name in THEME_DEFINITION_FILES))



def _git_lines(*args) -> list:
    res = subprocess.run(["git", *args], capture_output=True, text=True)
    if res.returncode != 0:
        return []
    return [line.strip() for line in res.stdout.splitlines() if line.strip()]


def changed_files() -> list:
    """Staged first — that is what a pre-commit run is about to record."""
    for args in (("diff", "--cached", "--name-only"),
                 ("diff", "--name-only", "HEAD")):
        files = _git_lines(*args)
        if files:
            return files
    return []


def block_vendored(files: list) -> list:
    return [f for f in files if is_vendored(f)]


def group_by_area(files: list) -> dict:
    """Two path components deep: deep enough to separate `src/parser` from
    `src/transport`, shallow enough not to call every file its own area."""
    areas = {}
    for path in files:
        parts = path.split("/")
        area = "/".join(parts[:2]) if len(parts) > 1 else "(repository root)"
        areas.setdefault(area, []).append(path)
    return areas


def report(files: list) -> None:
    areas = group_by_area(files)
    print("\n" + "=" * 74)
    print("SCOPE REPORT — {} changed file(s) across {} area(s)".format(
        len(files), len(areas)))
    print("=" * 74)
    for area, paths in sorted(areas.items(), key=lambda kv: (-len(kv[1]), kv[0])):
        print("  {} ({} file(s))".format(area, len(paths)))
        for path in sorted(paths):
            print("      {}".format(path))

    jira_keys = sorted(set(re.findall(r"\b[A-Z]{2,10}-\d+\b",
                                      "\n".join(_git_lines("log", "-n", "5",
                                                           "--oneline")))))
    if len(jira_keys) > 1:
        print("\n  Recent commits reference more than one ticket: {}".format(
            ", ".join(jira_keys)))
        print("  One pull request should serve one ticket.")

    print("\n  JUDGE THIS YOURSELF — the hook deliberately does not decide:")
    print("    * Does every file above serve the ONE task this branch is for?")
    print("    * Is anything here an opportunistic fix or cleanup you noticed")
    print("      along the way ('boy scouting')? That belongs in a separate PR.")
    print("    * Files spread over unrelated areas are a signal, not a verdict:")
    print("      a rename legitimately touches many; two files in two subsystems")
    print("      may still be two intents.")
    print("=" * 74 + "\n")


def main() -> int:
    files = changed_files()
    if not files:
        return 0

    vendored = block_vendored(files)
    if vendored:
        print("SCOPE ERROR: this change edits vendored trees this repository "
              "consumes but does not own:")
        for path in sorted(vendored):
            print("  - {}".format(path))
        print("Vendored code is updated upstream, never patched in place. "
              "Unstage these files.")
        return 1

    report(files)
    return 0


if __name__ == "__main__":
    sys.exit(main())
