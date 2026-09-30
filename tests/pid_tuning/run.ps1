param([string]$VcVars = 'D:\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat')
$ErrorActionPreference = 'Stop'
$testOutput = Join-Path ([IO.Path]::GetTempPath()) ('pid-tuning-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testOutput | Out-Null
Push-Location (Join-Path $PSScriptRoot '..\..')
try {
    foreach ($testName in @('test_tuning', 'test_vofa_esc', 'test_blackbox')) {
        $compileCommand = '"{0}" >nul && cl /nologo /TC /std:c11 /I tests\pid_tuning\stubs tests\pid_tuning\{2}.c /Fo:"{1}\{2}.o" /Fe:"{1}\{2}.exe" && "{1}\{2}.exe"' -f $VcVars, $testOutput, $testName
        & cmd /d /s /c $compileCommand
        if ($LASTEXITCODE -ne 0) { throw "$testName host tests failed." }
    }
} finally { Pop-Location }
