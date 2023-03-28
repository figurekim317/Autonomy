#!/bin/bash
sudoPW=neubility

declare -a ServiceRegexArray=("/etc/systemd/system/autonomy-*.service" "/etc/systemd/system/product-*.service")

for service_regex in ${ServiceRegexArray[@]}; do
    for file in $service_regex; do
        service_name=$(basename "$file")
        echo $service_name
        if [[ $service_name != *"recorder"* ]] && [ "$service_name" != "autonomy-update-binary.service" ]; then
            echo $sudoPW | sudo -S systemctl enable $service_name
            echo $sudoPW | sudo -S systemctl start $service_name
            echo $service_name - service enable/start
        fi
    done
done

echo $sudoPW | sudo -S docker compose -f /etc/monitoring/docker-compose.yaml up -d
