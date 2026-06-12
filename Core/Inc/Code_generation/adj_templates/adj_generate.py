#!/usr/bin/env python3
"""Generate DOF-specific adj JSONs from Jinja2 templates.

Usage:
  python3 adj_generate.py adj_5DOF [--auto-push] [--no-pull]
  python3 adj_generate.py adj_3DOF
  python3 adj_generate.py --all

Workflow per adj:
  1. Auto-clone: if deps/{adj}/ doesn't exist, clone from configured remote
  2. Auto-pull: if ADJ_AUTO_PULL=ON in .env, pull latest
  3. Generate: render Jinja2 templates into deps/{adj}/
  4. Auto-push: if --auto-push or ADJ_AUTO_PUSH=ON, commit + push changes
"""

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path

try:
    import jinja2
except ImportError:
    print("ERROR: jinja2 is not installed. Run: pip install jinja2", file=sys.stderr)
    sys.exit(1)

# --- Paths ---
REPO_ROOT = Path(__file__).resolve().parent.parent.parent.parent.parent  # LCU-Master-H11/
TEMPLATES_DIR = Path(__file__).resolve().parent  # adj_templates/
DEPS_DIR = REPO_ROOT / "deps"

# --- DOF configuration ---
DOF_CONFIGS = {
    "adj_1DOF": {"lpu_count": 1, "airgap_count": 1, "has_lpu_id": False},
    "adj_3DOF": {"lpu_count": 4, "airgap_count": 4, "has_lpu_id": True},
    "adj_5DOF": {"lpu_count": 10, "airgap_count": 8, "has_lpu_id": True},
}

# Default remotes (overridden by ADJ_REMOTE env var set by CMake)
DEFAULT_REMOTES = {
    "adj_1DOF": "",
    "adj_3DOF": "",
    "adj_5DOF": "",
}
DEFAULT_BRANCHES = {
    "adj_1DOF": "main",
    "adj_3DOF": "main",
    "adj_5DOF": "main",
}

# Files to generate (relative to adj dir, boards/LCU/ only)
OUTPUT_FILES = [
    "boards/LCU/LCU.json",
    "boards/LCU/sockets.json",
    "boards/LCU/LCU_measurements.json",
    "boards/LCU/packets.json",
    "boards/LCU/orders.json",
]

# Maps template path -> output path (boards/LCU/ files only)
TEMPLATE_MAP = {
    "boards/LCU/LCU.json.j2": "boards/LCU/LCU.json",
    "boards/LCU/sockets.json.j2": "boards/LCU/sockets.json",
    "boards/LCU/LCU_measurements.json.j2": "boards/LCU/LCU_measurements.json",
    "boards/LCU/packets.json.j2": "boards/LCU/packets.json",
    "boards/LCU/orders.json.j2": "boards/LCU/orders.json",
}


def load_env():
    """Load .env file from repo root. Returns dict of KEY=VALUE pairs."""
    env = {}
    env_path = REPO_ROOT / ".env"
    if not env_path.exists():
        return env
    with open(env_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            if "=" in line:
                key, _, value = line.partition("=")
                key = key.strip()
                value = value.strip().strip("\"'")
                env[key] = value
    return env


def get_remote(adj_name, env):
    """Get remote URL from environment (set by CMake)."""
    return os.environ.get("ADJ_REMOTE", "")


def get_branch(adj_name, env):
    """Get branch name from environment (set by CMake)."""
    key = f"{adj_name.upper()}_BRANCH"
    return os.environ.get(key, DEFAULT_BRANCHES.get(adj_name, "main"))


def get_bool_env(env, key, default="OFF"):
    """Read a boolean env var (ON/OFF)."""
    return env.get(key, default).strip().upper() == "ON"


def run(cmd, cwd=None, check=True):
    """Run a shell command."""
    result = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    if check and result.returncode != 0:
        print(f"  WARNING: {' '.join(cmd)} failed: {result.stderr.strip()}")
    return result


def auto_clone(adj_name, env):
    """Clone the adj repo if it doesn't exist."""
    repo_path = DEPS_DIR / adj_name
    remote = get_remote(adj_name, env)
    branch = get_branch(adj_name, env)

    if not remote:
        if not repo_path.exists():
            repo_path.mkdir(parents=True, exist_ok=True)
            print(f"  {adj_name}: No remote configured, creating local directory.")
        else:
            print(f"  {adj_name}: No remote configured, using existing local directory.")
        return True

    if repo_path.exists() and (repo_path / ".git").exists():
        current_remote = run(
            ["git", "-C", str(repo_path), "remote", "get-url", "origin"],
            check=False,
        )
        if current_remote.returncode == 0 and current_remote.stdout.strip() != remote:
            print(f"  Remote mismatch for {adj_name}, re-cloning...")
            import shutil
            shutil.rmtree(repo_path)
        else:
            return True

    if not repo_path.exists():
        print(f"  Cloning {adj_name} from {remote}...")
        run(["git", "clone", "-b", branch, remote, str(repo_path)], cwd=DEPS_DIR)
        return repo_path.exists()

    return True


def auto_pull(adj_name, env):
    """Pull latest changes if ADJ_AUTO_PULL=ON."""
    if not get_bool_env(env, "ADJ_AUTO_PULL", "ON"):
        print(f"  {adj_name}: Auto-pull disabled, skipping.")
        return
    repo_path = DEPS_DIR / adj_name
    if not (repo_path / ".git").exists():
        return
    remote = get_remote(adj_name, env)
    if not remote:
        return
    print(f"  Pulling {adj_name}...")
    run(["git", "-C", str(repo_path), "pull", "origin", get_branch(adj_name, env)])


def generate(adj_name):
    """Render all Jinja2 templates into the adj directory."""
    if adj_name not in DOF_CONFIGS:
        print(f"ERROR: Unknown adj config '{adj_name}'. Known: {list(DOF_CONFIGS)}")
        return False

    config = DOF_CONFIGS[adj_name]
    repo_path = DEPS_DIR / adj_name
    repo_path.mkdir(parents=True, exist_ok=True)
    (repo_path / "boards" / "LCU").mkdir(parents=True, exist_ok=True)

    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(str(TEMPLATES_DIR)),
        keep_trailing_newline=True,
    )

    for template_rel, output_rel in TEMPLATE_MAP.items():
        template = env.get_template(template_rel)
        rendered = template.render(**config)
        output_path = repo_path / output_rel
        output_path.write_text(rendered)

    print(f"  Generated {len(TEMPLATE_MAP)} files for {adj_name} "
          f"(LPUs={config['lpu_count']}, airgaps={config['airgap_count']}, "
          f"has_lpu_id={config['has_lpu_id']})")
    return True


def auto_push(adj_name, env, force=False):
    """Commit and push changes if auto-push is enabled."""
    if not force and not get_bool_env(env, "ADJ_AUTO_PUSH", "OFF"):
        return
    repo_path = DEPS_DIR / adj_name
    if not (repo_path / ".git").exists():
        print(f"  {adj_name}: Not a git repo, skipping push.")
        return
    remote = get_remote(adj_name, env)
    if not remote:
        print(f"  {adj_name}: No remote configured, skipping push.")
        return

    result = run(
        ["git", "-C", str(repo_path), "diff", "--quiet"],
        check=False,
    )
    if result.returncode == 0:
        staged = run(
            ["git", "-C", str(repo_path), "diff", "--cached", "--quiet"],
            check=False,
        )
        if staged.returncode == 0:
            return

    print(f"  Committing and pushing {adj_name}...")
    run(["git", "-C", str(repo_path), "add", "-A"])
    commit_result = run(
        ["git", "-C", str(repo_path), "commit", "-m", "auto: regenerate from templates"],
        check=False,
    )
    if commit_result.returncode != 0:
        return
    run(["git", "-C", str(repo_path), "push", "origin", get_branch(adj_name, env)])


def init_git_repo(adj_name):
    """Initialize a git repo for the adj directory if not already one."""
    repo_path = DEPS_DIR / adj_name
    if (repo_path / ".git").exists():
        return
    print(f"  Initializing git repo for {adj_name}...")
    run(["git", "-C", str(repo_path), "init"])
    run(["git", "-C", str(repo_path), "add", "-A"])
    run(["git", "-C", str(repo_path), "commit", "-m", "Initial adj generation from templates"])


def process_adj(adj_name, env, auto_push_flag=False):
    """Full workflow for one adj directory."""
    print(f"\n--- {adj_name} ---")
    if not auto_clone(adj_name, env):
        print(f"  SKIPPING {adj_name}: Could not clone.")
        return
    auto_pull(adj_name, env)
    if not generate(adj_name):
        return
    init_git_repo(adj_name)
    auto_push(adj_name, env, force=auto_push_flag)
    print(f"  {adj_name} DONE.")


def main():
    parser = argparse.ArgumentParser(description="Generate DOF-specific adj JSONs")
    parser.add_argument(
        "adj", nargs="?", default=None,
        help="Adj config name (e.g., adj_5DOF). Use --all for all configs.",
    )
    parser.add_argument(
        "--all", action="store_true",
        help="Generate all configured DOFs",
    )
    parser.add_argument(
        "--auto-push", action="store_true",
        help="Force commit+push even if ADJ_AUTO_PUSH is OFF",
    )
    parser.add_argument(
        "--no-pull", action="store_true",
        help="Skip git pull even if ADJ_AUTO_PULL is ON",
    )
    args = parser.parse_args()

    if not args.adj and not args.all:
        parser.error("Either specify an adj name or use --all")

    env = load_env()

    if args.auto_push:
        env["ADJ_AUTO_PUSH"] = "ON"
    if args.no_pull:
        env["ADJ_AUTO_PULL"] = "OFF"

    if args.all:
        for name in DOF_CONFIGS:
            process_adj(name, env, auto_push_flag=args.auto_push)
    else:
        process_adj(args.adj, env, auto_push_flag=args.auto_push)


if __name__ == "__main__":
    main()
