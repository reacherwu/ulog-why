#!/usr/bin/env sh
# One-command install: curl -fsSL https://raw.githubusercontent.com/reacherwu/ulog-why/main/install.sh | sh
set -e
command -v cargo >/dev/null 2>&1 || { echo "ulog-why needs Rust (cargo). Install it from https://rustup.rs and re-run."; exit 1; }
cargo install --locked --git https://github.com/reacherwu/ulog-why ulog-why
echo "installed. try: ulog-why demo"
