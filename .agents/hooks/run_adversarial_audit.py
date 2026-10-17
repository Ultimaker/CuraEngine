#!/usr/bin/env python3
"""
run_adversarial_audit.py
Automated Adversarial Security, Quality Gate & Intent Scope Audit Script.

Scans git diff and commit history for:
1. Hardcoded absolute paths (e.g. user home directories)
2. Private keys, API tokens, credentials
3. Python error swallowing
4. Raw hex colour literals in themeable sources — NOT just QML: React, Python
   UIs and stylesheets hardcode `#RRGGBB` just as readily
5. Interface changes that leave the API documentation behind
6. Edits to trees this repository vendors but does not own

Every folder list this script uses is discovered at bootstrap and rendered in
from ONE source (`hooks/partials/_repo_layout.py.j2`). Earlier revisions carried
private hardcoded copies of an interface directory, a vendor directory and a
default-branch list — literals lifted from one firmware repository, meaningless
in every other repository the bootstrap touched.
"""

import os
from pathlib import Path
import re
import subprocess
import sys

HOOKS_DIR = os.path.abspath(os.path.dirname(__file__))
if HOOKS_DIR not in sys.path:
    sys.path.insert(0, HOOKS_DIR)
from secret_scanner import SecretScanner  # noqa: E402
from path_scanner import PathScanner  # noqa: E402

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


HEX_COLOR_PATTERN = re.compile(r"#(?:[0-9a-fA-F]{3}){1,2}\b")


def _git_lines(*args):
    result = subprocess.run(["git", *args], capture_output=True, text=True)
    if result.returncode != 0:
        return []
    return [f.strip() for f in result.stdout.splitlines() if f.strip()]


def get_git_diff_files():
    """Everything this branch changes relative to its base, plus uncommitted
    work. Diffing only the working tree made this audit a no-op at pre-push
    time on a clean tree — committed changes were never audited at all."""
    files = set(_git_lines("diff", "--name-only", "HEAD"))
    files |= set(_git_lines("diff", "--cached", "--name-only"))
    merge_base = _git_lines("merge-base", "HEAD", f"origin/{BASE_BRANCH}")
    if merge_base:
        files |= set(_git_lines("diff", "--name-only", f"{merge_base[0]}..HEAD"))
    return sorted(files)


# Files where an absolute user path may legitimately appear as generated
# content rather than as something a human committed. Deliberately NOT
# `.md` wholesale: exempting every markdown file let absolute paths through
# in documentation, which the security-and-paths rule explicitly forbids, and
# documentation is exactly where a developer's home directory tends to be
# pasted from a terminal transcript.
_PATH_EXEMPT_PREFIXES = (".agents/rules/",)


def _path_exempt(filepath: str) -> bool:
    return filepath.startswith(_PATH_EXEMPT_PREFIXES)


def _check_line_patterns(filepath, idx, line, content, errors):
    if PathScanner.scan_line(line)[0] and not _path_exempt(filepath):
        errors.append(f"❌ [ABSOLUTE PATH] {filepath}:{idx}: {line.strip()}")

    if SecretScanner.scan_line(line):
        errors.append(f"❌ [SECRET DETECTED] {filepath}:{idx}")

    if filepath.endswith(".py"):
        c1 = "except Exception as e:" in line
        c2 = "except Exception:" in line
        if c1 or c2:
            w_start = max(0, idx - 1)
            w_end = min(len(content), idx + 5)
            window = "".join(content[w_start:w_end])
            has_exit = "sys.exit" in window or "file=sys.stderr" in window
            if not has_exit:
                errors.append(
                    f"⚠️ [PYTHON ERROR SWALLOWING] {filepath}:{idx}: "
                    "Exception caught without sys.exit or stderr output."
                )

    if is_themeable_source(filepath) and HEX_COLOR_PATTERN.search(line):
        errors.append(
            f"⚠️ [HARDCODED HEX COLOR] {filepath}:{idx}: "
            f"{line.strip()} (use this project's theme tokens instead)"
        )


def _check_architectural_limits(files, errors):
    # Only apply the API-doc coupling where those interface trees exist in
    # THIS repository; a foreign repo's layout is not evidence here.
    live_interfaces = [p for p in INTERFACE_PREFIXES if Path(p).is_dir()]
    interface_files = [f for f in files
                       if any(f.startswith(p) for p in live_interfaces)]
    api_doc_files = [f for f in files
                     if f in API_DOC_PATHS or "openapi" in f.lower()]
    if interface_files and API_DOC_PATHS and not api_doc_files:
        errors.append(
            f"❌ [API DOC DESYNC] Interface files modified "
            f"({len(interface_files)} files) but {', '.join(API_DOC_PATHS)} "
            "was not updated!"
        )

    vendor_files = [f for f in files if is_vendored(f)]
    if vendor_files:
        errors.append(
            f"❌ [VENDOR SDK MODIFIED] {len(vendor_files)} vendor files "
            f"modified (e.g. {vendor_files[0]}). Vendor code must remain untouched!"
        )


def _audit_single_file(filepath, errors):
    path = Path(filepath)
    if not path.exists() or path.is_dir():
        return

    # Guards whose own source must contain the patterns they detect, plus the
    # fire-proofing harness whose fixtures ARE violations by construction.
    # Without this the audit failed every bootstrap PR on the bootstrap's own
    # output, even on a clean tree. Exact filenames, never directory prefixes:
    # a blanket `.agents/hooks/` skip would be a place to hide a real secret.
    SELF_EXEMPT_NAMES = frozenset({
        "block-absolute-paths.py", "block-secrets.py", "path_scanner.py",
        "secret_scanner.py", "pretool_guard.py", "check_security_downgrades.py",
        "run_adversarial_audit.py", "verify_hooks_fire.py",
    })
    if path.name in SELF_EXEMPT_NAMES:
        return

    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            content = f.readlines()

        for idx, line in enumerate(content, 1):
            _check_line_patterns(filepath, idx, line, content, errors)
    except OSError:
        return


def audit_diff():
    sec_hook = Path(__file__).parent / "check_security_downgrades.py"
    if sec_hook.exists():
        res = subprocess.run([sys.executable, str(sec_hook)])
        if res.returncode != 0:
            return 1

    files = get_git_diff_files()
    if not files:
        print("==> Adversarial Audit: No modified files detected in git diff.")
        return 0

    errors = []
    print("==> Running Adversarial Security, Quality & Intent Audit on "
          f"{len(files)} modified files...")

    for filepath in files:
        _audit_single_file(filepath, errors)

    _check_architectural_limits(files, errors)

    # Scope judgement lives in check_multi_intent_scope.py — one hook, one
    # question. Delegating rather than re-deriving it here keeps the two from
    # disagreeing about what "too wide" means.
    scope_hook = Path(__file__).parent / "check_multi_intent_scope.py"
    if scope_hook.exists():
        res = subprocess.run([sys.executable, str(scope_hook)])
        if res.returncode != 0:
            return 1

    if errors:
        print("\n" + "=" * 74)
        print("🚨 ADVERSARIAL AUDIT FINDINGS & INTENT EVALUATION:")
        print("=" * 74)
        for err in errors:
            print(err)
        print("=" * 74 + "\n")
        crit_keys = ["ABSOLUTE PATH", "SECRET DETECTED", "API DOC DESYNC",
                     "VENDOR SDK MODIFIED"]
        critical_errors = [e for e in errors if any(ck in e for ck in crit_keys)]
        if critical_errors:
            print("❌ Critical security findings must be resolved.")
            return 1

    print("✅ Adversarial Security, Quality & Intent Audit Passed Cleanly!")
    return 0


if __name__ == "__main__":
    sys.exit(audit_diff())
