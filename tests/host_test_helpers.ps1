# Shared reporting for the two host entry scripts. Commands stay in each entry.
function New-HostSuite([string]$Name,[string]$OutputDirectory) {
    New-Item -ItemType Directory -Path $OutputDirectory -Force | Out-Null
    [pscustomobject]@{name=$Name; output=(Resolve-Path $OutputDirectory).Path;
        results=[System.Collections.Generic.List[object]]::new()}
}
function Invoke-HostCase {
    param($Suite,[string]$Name,[string]$CompileCommand,[string]$RunCommand,
          [bool]$Prerequisite=$true,[string]$EnvironmentProblem='',[string]$Note='')
    $row=[pscustomobject]@{name=$Name;compile='NOT_EXECUTED';compile_exit=$null;
        run='NOT_EXECUTED';run_exit=$null;status='NOT_EXECUTED';note=$Note}
    if(!$Prerequisite) {
        $row.note="Prerequisite failed; no stale executable/fixture used. $Note"
    } elseif($EnvironmentProblem) {
        $row.status='ENVIRONMENT_BLOCKED';$row.note=$EnvironmentProblem
    } else {
        try {
            if($CompileCommand) {
                $CompileCommand | Set-Content (Join-Path $Suite.output "$Name.compile.command.txt") -Encoding utf8
                & cmd /d /s /c $CompileCommand *> (Join-Path $Suite.output "$Name.compile.log")
                $row.compile_exit=$LASTEXITCODE
                $row.compile=if($LASTEXITCODE -eq 0){'PASS'}else{'FAIL'}
            } else { $row.compile='NOT_REQUIRED' }
            if($row.compile -eq 'FAIL') { $row.status='COMPILE_FAIL' }
            elseif($RunCommand) {
                $RunCommand | Set-Content (Join-Path $Suite.output "$Name.run.command.txt") -Encoding utf8
                & cmd /d /s /c $RunCommand *> (Join-Path $Suite.output "$Name.run.log")
                $row.run_exit=$LASTEXITCODE
                $row.run=if($LASTEXITCODE -eq 0){'PASS'}else{'FAIL'}
                $row.status=if($LASTEXITCODE -eq 0){'PASS'}else{'RUN_FAIL'}
            } else { $row.run='NOT_REQUIRED';$row.status='PASS' }
        } catch { $row.status='ENVIRONMENT_BLOCKED';$row.note=$_.Exception.Message }
    }
    $Suite.results.Add($row)
    Write-Host "$Name : $($row.status) (compile=$($row.compile), run=$($row.run))"
    return $row.status -eq 'PASS'
}
function Complete-HostSuite($Suite) {
    $failed=@($Suite.results | Where-Object status -ne 'PASS').Count
    $exitCode=if($failed){1}else{0}
    [pscustomobject]@{suite=$Suite.name;exit_code=$exitCode;
        results=@($Suite.results.ToArray())} | ConvertTo-Json -Depth 5 |
        Set-Content (Join-Path $Suite.output 'summary.json') -Encoding utf8
    $table=$Suite.results | Format-Table name,compile,compile_exit,run,run_exit,status -AutoSize | Out-String -Width 200
    $table | Set-Content (Join-Path $Suite.output 'summary.txt') -Encoding utf8
    Write-Host $table
    Write-Host "Results: $($Suite.output); non-passing=$failed; exit=$exitCode"
    return $exitCode
}
