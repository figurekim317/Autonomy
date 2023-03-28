#!/bin/bash
sudoPW=neubility

declare -a ServiceRegexArray=("/etc/systemd/system/autonomy-*.service" "/etc/systemd/system/product-*.service")

for service_regex in ${ServiceRegexArray[@]}; do
    for file in $service_regex; do
        service_name=$(basename "$file")
        status=$(systemctl is-enabled $service_name)
        if [ "$status" = "enabled" ] && [ "$service_name" != "autonomy-update-binary.service" ] && [ "$service_name" != "autonomy-sensor-fusion-navigation-update-manager.service" ] && [ "$service_name" != "autonomy-sensor-fusion-navigation-stm.service" ] && [ "$service_name" != "autonomy-sensor-fusion-navigation-system-checker.service" ] && [ "$service_name" != "autonomy-sensor-fusion-navigation.service" ]; then
            echo $sudoPW | sudo -S systemctl stop $service_name
            echo $sudoPW | sudo -S systemctl disable $service_name
            echo $sudoPW | echo $service_name - service stop/disable
        fi
        if [ "$service_name" != "autonomy-sensor-fusion-navigation-update-manager.service" ] && [ "$service_name" != "autonomy-sensor-fusion-navigation-stm.service" ] && [ "$service_name" != "autonomy-sensor-fusion-navigation-system-checker.service" ] && [ "$service_name" != "autonomy-sensor-fusion-navigation.service" ]; then
            echo $sudoPW | sudo -S rm $file
        fi
    done
done

# docker-compose down
echo $sudoPW | sudo -S docker compose -f /etc/monitoring/docker-compose.yaml down

sleep 5s
