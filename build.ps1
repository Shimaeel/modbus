# Build script for the Modbus client.
# Usage:
#   .\build.ps1                  → configure + build all targets
#   .\build.ps1 modbus_ge_evt_test → build only one target

param(
    [string]$Target = ""
)

$ErrorActionPreference = "Stop"

# Configure (idempotent — re-runs are fast if nothing changed)
cmake -S . -B build
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# Build
if ($Target -ne "") {
    cmake --build build --target $Target
} else {
    cmake --build build
}
exit $LASTEXITCODE
