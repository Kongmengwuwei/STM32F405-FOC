$ErrorActionPreference = 'Stop'
$previousDirectory = Get-Location
try {
    Set-Location (Join-Path $PSScriptRoot '..')
    $gimbalBuildDir = [System.IO.Path]::GetFullPath((Join-Path (Get-Location) '../../build'))
    [System.IO.Directory]::CreateDirectory($gimbalBuildDir) | Out-Null
    foreach ($policy in @(0, 1)) {
        $gimbalExe = Join-Path $gimbalBuildDir "test_gimbal_$policy.exe"
        $compilerArgs = @(
            '-std=c11', '-Wall', '-Wextra', '-Werror', '-O2',
            '-DFOC_DUAL', '-DFOC_GIMBAL=1', "-DFOC_PROTECTION_TRIP=$policy",
            '-DFOC_PORT_M0', '-DFOC_ENCODER_TLE5012B', '-DFOC_MOTOR_ZH3620_1', '-DFOC_INSTALLATION_ID=1',
            '-I', 'tests/dual_stubs', '-I', 'dual', '-I', 'App/Control', '-I', 'App/FOC',
            '-I', 'App/Hardware/bsp', '-I', 'App/Hardware/encoder', '-I', 'App/Config',
            'tests/test_gimbal.c', 'dual/app.c', 'dual/control0.c', 'dual/control1.c', 'dual/foc0.c', 'dual/foc1.c',
            '-lm', '-o', $gimbalExe
        )
        & gcc @compilerArgs
        if ($LASTEXITCODE -ne 0) { throw "Gimbal test compilation failed, policy=$policy" }
        foreach ($case in @('commands', 'response', 'braking', 'speed_foldback', 'zero', 'travel', 'cal_limit', 'negative_limit', 'cal_speed', 'sensor', 'protection', 'diagnostic', 'cal_settle', 'cal_damping', 'field_tracking')) {
            & $gimbalExe $case
            if ($LASTEXITCODE -ne 0) { throw "Gimbal test failed: $case, policy=$policy" }
        }
    }
} finally {
    Set-Location $previousDirectory
}
