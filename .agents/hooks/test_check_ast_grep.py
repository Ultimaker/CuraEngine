#!/usr/bin/env python3
"""Executable fixtures for CuraEngine's ast-grep rules and changed-line gate."""

import importlib.util
import io
import json
import shutil
import subprocess
import sys
import tempfile
import unittest
from contextlib import redirect_stderr
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[2]
HOOK = ROOT / ".agents/hooks/check_ast_grep.py"
SPEC = importlib.util.spec_from_file_location("check_ast_grep", HOOK)
CHECK = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(CHECK)


class StructuralRulesTest(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(shutil.which("ast-grep"), "Install ast-grep-cli==0.45.1")
        self.temp = tempfile.TemporaryDirectory(prefix="ast-grep-fixture-", dir=ROOT / "include/geometry")
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name) / "fixture.h"

    def check_source(self, source: str, expected_id: str | None):
        self.path.write_text(source, encoding="utf-8")
        result = subprocess.run(
            [sys.executable, str(HOOK), "--working", str(self.path)],
            cwd=ROOT,
            capture_output=True,
            text=True,
        )
        if expected_id:
            self.assertEqual(result.returncode, 1, result.stderr)
            self.assertIn(f"[{expected_id}]", result.stderr)
        else:
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_squared_distance_tolerance(self):
        self.check_source(
            "bool close(Point2LL vec) { return vSize2(vec) <= EPSILON; }\n",
            "no-unsquared-geometry-tolerance",
        )
        self.check_source(
            "bool close(Point2LL vec) { return EPSILON > vSize2(vec); }\n",
            "no-unsquared-geometry-tolerance",
        )
        self.check_source("bool close(Point2LL vec) { return vSize2(vec) <= EPSILON_SQUARED; }\n", None)
        self.check_source("bool close(Point2LL vec) { return vec.vSize2() <= EPSILON; }\n",
                          "no-unsquared-geometry-tolerance")
        self.check_source("bool close(Point2LL vec) { return vec.vSize2() <= EPSILON_SQUARED; }\n", None)

    def test_core_header_include_direction(self):
        self.check_source('#include "communication/ArcusCommunication.h"\n', "no-transport-in-core-geometry-headers")
        self.check_source('#include "Application.h"\n', "no-transport-in-core-geometry-headers")
        self.check_source('#include "geometry/Polygon.h"\n', None)

    def test_only_changed_lines_are_checked(self):
        self.path.write_text(
            "bool close(Point2LL vec) { return vSize2(vec) <= EPSILON; }\n", encoding="utf-8"
        )
        with patch.object(CHECK, "added_lines", return_value={3}):
            self.assertEqual(CHECK.main(["--working", str(self.path)]), 0)

    def test_staged_new_header_is_checked(self):
        with tempfile.TemporaryDirectory(prefix="curaengine-ast-grep-repo-") as directory:
            repo = Path(directory)
            shutil.copy(ROOT / "sgconfig.yml", repo / "sgconfig.yml")
            shutil.copytree(ROOT / "rules/cpp", repo / "rules/cpp")
            header = repo / "include/geometry/new_geometry.h"
            header.parent.mkdir(parents=True)
            header.write_text("bool close(Point2LL vec) { return vSize2(vec) <= EPSILON; }\n", encoding="utf-8")
            subprocess.run(["git", "init", "-q"], cwd=repo, check=True)
            subprocess.run(["git", "add", "."], cwd=repo, check=True)
            stderr = io.StringIO()
            with patch.object(CHECK, "ROOT", repo), redirect_stderr(stderr):
                self.assertEqual(CHECK.main([str(header)]), 1)
            self.assertIn("[no-unsquared-geometry-tolerance]", stderr.getvalue())

    def test_post_edit_hook_runs_structural_rules(self):
        self.path.write_text('#include "Application.h"\n', encoding="utf-8")
        for payload in (
            {"tool_input": {"file_path": str(self.path)}},
            {"toolArgs": {"filePath": str(self.path)}},
            {"toolCall": {"args": {"TargetFile": str(self.path)}}},
        ):
            with self.subTest(payload=payload):
                result = subprocess.run(
                    [sys.executable, str(ROOT / ".agents/hooks/post_edit_check.py")],
                    cwd=ROOT,
                    input=json.dumps(payload),
                    capture_output=True,
                    text=True,
                )
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertIn("[no-transport-in-core-geometry-headers]", result.stderr)

    def test_scan_failure_does_not_pass(self):
        self.path.write_text("bool close(Point2LL vec) { return vSize2(vec) <= EPSILON; }\n", encoding="utf-8")
        stderr = io.StringIO()
        with patch.object(CHECK, "scan", side_effect=RuntimeError("invalid rule")), redirect_stderr(stderr):
            self.assertEqual(CHECK.main(["--working", str(self.path)]), 1)
        self.assertIn("invalid rule", stderr.getvalue())


if __name__ == "__main__":
    unittest.main()
