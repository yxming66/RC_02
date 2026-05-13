$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Resolve-Path (Join-Path $ScriptDir "..\..\..\..")
$OutDir = Join-Path $RepoRoot "build\host_tests"
$Exe = Join-Path $OutDir "arm_algorithm_checks.exe"

New-Item -ItemType Directory -Force -Path $OutDir | Out-Null

$Includes = @(
  "-I.",
  "-IUser",
  "-IUser/component",
  "-IUser/component/toolbox",
  "-IDrivers/CMSIS/DSP/Include",
  "-IDrivers/CMSIS/Include"
)

$Defines = @(
  "-DARM_MATH_CM4",
  "-D__FPU_PRESENT=1",
  "-DARM_MATH_MATRIX_CHECK"
)

$Sources = @(
  "User/robotics/arm/tests/host_cmsis_matrix_stub.cpp",
  "User/robotics/arm/tests/arm_algorithm_checks.cpp",
  "User/component/toolbox/matrix.cpp",
  "User/component/toolbox/utils.cpp",
  "User/component/toolbox/robotics.cpp",
  "User/robotics/arm/adapter/toolbox_adapter.cpp",
  "User/robotics/arm/model/joint.cpp",
  "User/robotics/arm/model/link.cpp",
  "User/robotics/arm/model/tool_frame.cpp",
  "User/robotics/arm/model/serial_chain.cpp",
  "User/robotics/arm/kinematics/pose_error.cpp",
  "User/robotics/arm/kinematics/analytic_solvers/ik_3r_planar.cpp",
  "User/robotics/arm/kinematics/analytic_solvers/ik_6r_pieper.cpp",
  "User/robotics/arm/application/three_pit_cartesian_app.cpp"
)

Push-Location $RepoRoot
try {
  & g++ -std=c++17 -O0 -g -fpermissive @Includes @Defines @Sources -o $Exe
  if ($LASTEXITCODE -ne 0) {
    throw "g++ failed with exit code $LASTEXITCODE"
  }
  & $Exe
  if ($LASTEXITCODE -ne 0) {
    throw "robotics/arm algorithm checks failed with exit code $LASTEXITCODE"
  }
} finally {
  Pop-Location
}
