param([string]$VcVars='D:\Microsoft Visual Studio\18\BuildTools\VC\Auxiliary\Build\vcvars64.bat',
      [string]$OutputDirectory='', [string]$ReceiverDirectory='')
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot '../host_test_helpers.ps1')
if(!$OutputDirectory) { $OutputDirectory=Join-Path $PSScriptRoot ('../build/pid-tuning-'+[guid]::NewGuid().ToString('N')) }
if(!$ReceiverDirectory) { $ReceiverDirectory=Join-Path $PSScriptRoot '../../../references/STM32F407VET6黑匣子(弃用）' }
$suite=New-HostSuite 'pid_tuning' $OutputDirectory
$compilerProblem=if(!(Test-Path -LiteralPath $VcVars)){'MSVC vcvars64.bat not found: '+$VcVars}else{''}
Push-Location (Join-Path $PSScriptRoot '../..')
try {
    $crossDir=Join-Path $ReceiverDirectory 'tests/build'
    New-Item -ItemType Directory -Path $crossDir -Force | Out-Null
    $crossDir=(Resolve-Path $crossDir).Path
    foreach($name in @('test_tuning','test_notch','test_vofa_esc','test_blackbox',
        'test_blackbox_schedule','test_optical_flow','test_optical_diag','test_bmi088',
        'test_bmi088_gyro_filter','test_bmi088_filter_response','test_bmi088_heater','test_bmi088_attitude','test_blackbox_extra')) {
        $source=if($name -eq 'test_blackbox_extra'){'test_blackbox'}else{$name}
        $define=if($name -eq 'test_blackbox_extra'){'/DBLACKBOX_TEST_EXTRA_FIELDS'}else{''}
        $fixture=switch($name) {
            'test_blackbox' {Join-Path $crossDir 'h723_v3_wire.bin'}
            'test_blackbox_extra' {Join-Path $crossDir 'h723_v3_extra.bin'}
            'test_bmi088_filter_response' {Join-Path $suite.output 'bmi088-filter-response.csv'}
            default {Join-Path $crossDir 'h723_wire.bin'}
        }
        $compile='call "{0}" >nul && cl /nologo /TC /std:c11 /D_CRT_SECURE_NO_WARNINGS {1} /I tests\pid_tuning\stubs tests\pid_tuning\{2}.c /Fo:"{3}\{4}.o" /Fe:"{3}\{4}.exe"' -f $VcVars,$define,$source,$suite.output,$name
        $run='"{0}\{1}.exe" "{2}"' -f $suite.output,$name,$fixture
        $null=Invoke-HostCase $suite $name $compile $run -EnvironmentProblem $compilerProblem
    }
    # C0b is NOT implemented: do not migrate/run the obsolete safety test here.
    # Keep F4 visible and non-green until that separately approved batch resolves it.
    $suite.results.Add([pscustomobject]@{name='test_rc_safety (F4/C0b)';
        compile='NOT_EXECUTED';compile_exit=$null;run='NOT_EXECUTED';run_exit=$null;
        status='KNOWN_FAILURE_UNRESOLVED';
        note='Audit F4: standalone compilation fails (obsolete fields/stubs/contract). Not repaired or added to execution by C0a; C0b pending. This is a registered issue, not a fresh execution result.'})
} finally {Pop-Location}
exit (Complete-HostSuite $suite)
