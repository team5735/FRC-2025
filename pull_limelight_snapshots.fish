function pull_snapshots_from
    set ip $argv[1]
    echo retrieving manifest for $ip
    set snapshotfiles (fetcher $ip:5807/snapshotmanifest - | jq .[] -r)
    for file in snapshotfiles
        fetcher $ip:5801/snapshots/$file snapshots/$counter.png
        set counter (math $counter + 1)
    end
end

if type -q wget
    functions -e fetcher
    function fetcher
        wget $argv[1] -O $argv[2]
    end
else if type -q curl
    functions -e fetcher
    function fetcher
        curl $argv[1] -o $argv[2]
    end
else
    echo you need wget or curl!
    exit 1
end

set counter 0
set limelight_ports 17 18
while true
    for subip in $limelight_ports
        pull_snapshots_from 10.57.35.$subip
    end
    if test $status -ne 0
        echo stopping!
        break
    end
end
